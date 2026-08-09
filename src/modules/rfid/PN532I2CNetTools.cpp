#ifndef LITE_VERSION
#include "PN532I2CNetTools.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/wifi/wifi_common.h"
#include "globals.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <algorithm>

// PN532 HSU ACK frame
static const uint8_t PN532_ACK_FRAME[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

static String getSecureApSsid() {
    String ssid = bruceConfig.wifiAp.ssid;
    if (ssid.isEmpty()) ssid = "BruceNet";
    return ssid;
}

static String getSecureApPassword() {
    String pwd = bruceConfig.wifiAp.pwd;
    // WPA2 requires at least 8 chars; fallback prevents accidental open AP.
    if (pwd.length() < 8) pwd = "brucenet";
    return pwd;
}

static bool waitForSoftApReady(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        wifi_mode_t mode = WiFi.getMode();
        if (mode == WIFI_AP || mode == WIFI_AP_STA) {
            IPAddress ip = WiFi.softAPIP();
            if (ip != IPAddress(0, 0, 0, 0)) {
                return true;
            }
        }
        delay(10);
    }
    return false;
}

static bool isAnyWifiInterfaceReadyForTcp() {
    wifi_mode_t mode = WiFi.getMode();
    bool apReady = (mode == WIFI_AP || mode == WIFI_AP_STA) &&
                   (WiFi.softAPIP() != IPAddress(0, 0, 0, 0));
    bool staReady = WiFi.isConnected();
    return apReady || staReady;
}

// ————— BLE globals (defined in ble_common.cpp) ————————————
extern BLEServer *pServer;
extern BLEService *pService;
extern BLECharacteristic *pTxCharacteristic;
extern BLECharacteristic *pRxCharacteristic;
extern bool bleDataTransferEnabled;

// Active instance pointer so the BLE callback can reach us
static std::atomic<PN532I2CNetTools *> s_activeI2CNet{nullptr};
static SemaphoreHandle_t s_callbackGuardMutex = nullptr;

// BLE stack persists across tool instances; deinit/init cycles corrupt
// NimBLE internal state (notifications stop working after re-init).
static bool s_bleStackReady = false;

// ————— BLE Callbacks ——————————————————————————————————————
class I2CNetRxCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
        std::string val = pCharacteristic->getValue();
        PN532I2CNetTools *target = s_activeI2CNet.load(std::memory_order_acquire);
        if (!val.empty() && target && target->tryEnterBleCallback()) {
            target->enqueueRx((const uint8_t *)val.data(), val.size());
            target->leaveBleCallback();
        }
        if (!BLEConnected) {
            BLEConnected = true;
            drawStatusBar();
        }
    }
};

class I2CNetServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer *pServer, NimBLEConnInfo &connInfo) override {
        BLEConnected = true;
        drawStatusBar();
    }
    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
        BLEConnected = false;
        drawStatusBar();
        // Only restart advertising if the bridge is still active;
        // during BLEDevice::deinit() the stack tears down connections
        // and this callback fires — restarting would fail with rc=30.
        if (bleDataTransferEnabled) {
            pServer->getAdvertising()->start();
        }
    }
};

// ————— Constructor / Destructor ————————————————————————————
PN532I2CNetTools::PN532I2CNetTools() {
    if (!s_callbackGuardMutex) {
        s_callbackGuardMutex = xSemaphoreCreateMutex();
    }
    _bridgeStoppedSem = xSemaphoreCreateBinary();
    _shuttingDown.store(false, std::memory_order_release);
    _callbacksInFlight.store(0, std::memory_order_release);
    s_activeI2CNet.store(this, std::memory_order_release);
    _rxMutex = xSemaphoreCreateMutex();
    displayInitialScreen();
    loop();
}

PN532I2CNetTools::~PN532I2CNetTools() {
    if (s_callbackGuardMutex && xSemaphoreTake(s_callbackGuardMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        _shuttingDown.store(true, std::memory_order_release);
        if (s_activeI2CNet.load(std::memory_order_acquire) == this) {
            s_activeI2CNet.store(nullptr, std::memory_order_release);
        }
        xSemaphoreGive(s_callbackGuardMutex);
    } else {
        _shuttingDown.store(true, std::memory_order_release);
        s_activeI2CNet.store(nullptr, std::memory_order_release);
    }

    // Let in-flight BLE callbacks complete before freeing resources.
    for (uint32_t i = 0; i < BRIDGE_TASK_STOP_WAIT_LOOPS; ++i) {
        if (_callbacksInFlight.load(std::memory_order_acquire) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(BRIDGE_TASK_WAIT_DELAY_MS));
    }

    stopBridgeTask();
    if (_udpEnabled) disableUdpDataTransfer();
    if (_tcpEnabled) disableTcpDataTransfer();

    // Stop BLE advertising but do NOT call BLEDevice::deinit().
    // Deinit/re-init cycles corrupt NimBLE internal state (notifications
    // stop working).  The BLE stack persists so the next tool instance
    // can reuse it without re-initialisation.
    bleDataTransferEnabled = false;
    BLEConnected = false;
    if (s_bleStackReady && NimBLEDevice::isInitialized() && pServer) {
        pServer->getAdvertising()->stop();
    }
    if (_rxMutex) {
        vSemaphoreDelete(_rxMutex);
        _rxMutex = nullptr;
    }
    if (_bridgeStoppedSem) {
        vSemaphoreDelete(_bridgeStoppedSem);
        _bridgeStoppedSem = nullptr;
    }
}

// ————— Public: enqueue bytes from BLE callback ————————————
void PN532I2CNetTools::enqueueRx(const uint8_t *data, size_t len) {
    if (!_rxMutex) return;
    if (xSemaphoreTake(_rxMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    _rxBuffer.insert(_rxBuffer.end(), data, data + len);
    xSemaphoreGive(_rxMutex);
}

bool PN532I2CNetTools::tryEnterBleCallback() {
    if (s_callbackGuardMutex && xSemaphoreTake(s_callbackGuardMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool canEnter = !_shuttingDown.load(std::memory_order_acquire) &&
                        s_activeI2CNet.load(std::memory_order_acquire) == this;
        if (canEnter) {
            _callbacksInFlight.fetch_add(1, std::memory_order_acq_rel);
        }
        xSemaphoreGive(s_callbackGuardMutex);
        return canEnter;
    }
    return false;
}

void PN532I2CNetTools::leaveBleCallback() {
    _callbacksInFlight.fetch_sub(1, std::memory_order_acq_rel);
}

void PN532I2CNetTools::processPendingRx() {
    if (_bridgeBusy) return;
    if (!_rxMutex) return;
    if (xSemaphoreTake(_rxMutex, 0) != pdTRUE) return;
    if (_rxBuffer.empty()) {
        xSemaphoreGive(_rxMutex);
        return;
    }
    _bridgeBusy = true;
    drainNetToI2C();
    _bridgeBusy = false;
    xSemaphoreGive(_rxMutex);
}

void PN532I2CNetTools::bridgeTaskEntry(void *arg) {
    auto *self = static_cast<PN532I2CNetTools *>(arg);
    while (self && self->_bridgeTaskRunning.load(std::memory_order_acquire)) {
        self->bridgeIoTick();
        vTaskDelay(pdMS_TO_TICKS(BRIDGE_IO_TICK_PERIOD_MS));
    }

    if (self) {
        self->_bridgeTaskRunning.store(false, std::memory_order_release);
        self->_bridgeTaskHandle = nullptr;
        if (self->_bridgeStoppedSem) {
            xSemaphoreGive(self->_bridgeStoppedSem);
        }
    }

    vTaskDelete(nullptr);
}

void PN532I2CNetTools::startBridgeTask() {
    if (_bridgeTaskHandle) return;
    _bridgeTaskRunning.store(true, std::memory_order_release);
    if (_bridgeStoppedSem) {
        xSemaphoreTake(_bridgeStoppedSem, 0);
    }
    BaseType_t created = xTaskCreatePinnedToCore(
        bridgeTaskEntry,
        "pn532_i2c_bridge",
        BRIDGE_TASK_STACK_SIZE,
        this,
        BRIDGE_TASK_PRIORITY,
        &_bridgeTaskHandle,
        1
    );
    if (created != pdPASS) {
        _bridgeTaskRunning.store(false, std::memory_order_release);
        _bridgeTaskHandle = nullptr;
    }
}

void PN532I2CNetTools::stopBridgeTask() {
    if (!_bridgeTaskHandle) return;
    _bridgeTaskRunning.store(false, std::memory_order_release);

    TickType_t waitTicks = pdMS_TO_TICKS(BRIDGE_TASK_STOP_WAIT_LOOPS * BRIDGE_TASK_WAIT_DELAY_MS);
    if (_bridgeStoppedSem && xSemaphoreTake(_bridgeStoppedSem, waitTicks) == pdTRUE) {
        return;
    }

    // Fallback path if task did not signal stop in time.
    _bridgeTaskHandle = nullptr;
}

void PN532I2CNetTools::bridgeIoTick() {
    if (!_initialized) return;

    if (bleDataTransferEnabled) { processPendingRx(); }

    if (_udpEnabled) {
        int pktSize = _udp.parsePacket();
        if (pktSize > 0) {
            if (!_udpHasRemote) {
                _udpRemoteIP = _udp.remoteIP();
                _udpRemotePort = _udp.remotePort();
                _udpHasRemote = true;
                _udpLastPacketMs = millis();
                printCenterFootnote(String("Remote: ") + _udpRemoteIP.toString());
            } else {
                _udpLastPacketMs = millis();
            }
            uint8_t buf[256];
            int len = _udp.read(buf, sizeof(buf));
            if (len > 0) {
                enqueueRx(buf, len);
                processPendingRx();
            }
        } else if (_udpHasRemote && millis() - _udpLastPacketMs > UDP_REMOTE_TIMEOUT_MS) {
            _udpHasRemote = false;
            printCenterFootnote("Waiting for UDP client...");
        }
    }

    if (_tcpEnabled) {
        if (_tcpListenerStarting) {
            if (isAnyWifiInterfaceReadyForTcp()) {
                if (_tcpListenerReadyChecks < 2) {
                    _tcpListenerReadyChecks++;
                }
            } else {
                _tcpListenerReadyChecks = 0;
            }

            if (_tcpListenerReadyChecks >= 2) {
                _tcpListenerStarting = false;
            } else {
                return;
            }
        }

        if (!_tcpHasClient) {
            WiFiClient newClient = _tcpServer.accept();
            if (newClient) {
                _tcpClient.stop();
                _tcpClient = newClient;
                _tcpHasClient = true;
                _tcpLastPacketMs = millis();
                printCenterFootnote(String("TCP:") + _tcpClient.remoteIP().toString());
            }
        } else if (!_tcpClient.connected()) {
            _tcpClient.stop();
            _tcpHasClient = false;
            printCenterFootnote("Waiting TCP client...");
        } else {
            while (_tcpClient.connected() && _tcpClient.available()) {
                uint8_t buf[256];
                int r = _tcpClient.read(buf, sizeof(buf));
                if (r > 0) {
                    _tcpLastPacketMs = millis();
                    enqueueRx(buf, r);
                    processPendingRx();
                }
            }
            if (_tcpHasClient && millis() - _tcpLastPacketMs > TCP_REMOTE_TIMEOUT_MS) {
                _tcpClient.stop();
                _tcpHasClient = false;
                printCenterFootnote("Waiting TCP client...");
            }
        }
    }
}

// ————— Init ——————————————————————————————————————————————
bool PN532I2CNetTools::initPN532() {
    gpio_num_t sda = bruceConfigPins.i2c_bus.sda;
    gpio_num_t scl = bruceConfigPins.i2c_bus.scl;

    // Fallback to board defaults when runtime pin config is not available.
    if (sda == GPIO_NUM_NC) sda = (gpio_num_t)GROVE_SDA;
    if (scl == GPIO_NUM_NC) scl = (gpio_num_t)GROVE_SCL;

    // Hardware-reset the PN532 BEFORE touching I2C.  After the previous
    // session's Wire.end() the PN532 may still be holding the bus from an
    // interrupted transaction.  Resetting it releases SDA/SCL so that
    // Wire.begin() won't deadlock on ESP32-S3.
#if defined(PN532_RF_REST)
    pinMode(PN532_RF_REST, OUTPUT);
    digitalWrite(PN532_RF_REST, LOW);
    delay(1);   // datasheet: min 20 ns
    digitalWrite(PN532_RF_REST, HIGH);
    delay(5);   // datasheet: max 2 ms boot
#endif

    Wire.begin((int)sda, (int)scl);
    Wire.setClock(100000);

#if defined(PN532_IRQ) && defined(PN532_RF_REST)
    Adafruit_PN532 probe = Adafruit_PN532(PN532_IRQ, PN532_RF_REST);
#else
    Adafruit_PN532 probe = Adafruit_PN532();
#endif
    probe.setInterface((int)sda, (int)scl);
    probe.begin();

    uint32_t versiondata = probe.getFirmwareVersion();
    if (!versiondata) {
        displayError("PN532 not found");
        return false;
    }
    probe.SAMConfig();
    return true;
}

// ————— Display helpers ————————————————————————————————————
void PN532I2CNetTools::displayBanner() {
    drawMainBorderWithTitle(_titleName.c_str());
    delay(UI_MED_DELAY_MS);
}

void PN532I2CNetTools::displayInitialScreen() {
    _titleName = "PN532 Net Bridge";
    drawMainBorderWithTitle(_titleName.c_str());
    tft.setTextSize(FP);
    int baseY = tftHeight / 2;
    String l1 = "Bridge built-in PN532";
    String l2 = "via BLE / WiFi (TCP/UDP)";
    String l3 = "Press OK to initialise";
    tft.setCursor((tftWidth - l1.length() * 6 * FP) / 2, baseY);
    tft.println(l1);
    tft.setCursor((tftWidth - l2.length() * 6 * FP) / 2, baseY + FP * 12);
    tft.println(l2);
    tft.setCursor((tftWidth - l3.length() * 6 * FP) / 2, baseY + FP * 24);
    tft.println(l3);
    delay(UI_INFO_DELAY_MS);
}

// ————— Main loop ——————————————————————————————————————————
void PN532I2CNetTools::loop() {
    while (true) {
        if (check(EscPress)) {
            disableAllBridges();
            return;
        }

        if (!_initialized) {
            if (check(SelPress)) {
                displayInfo("Initialising PN532...");
                if (initPN532()) {
                    _initialized = true;
                    startBridgeTask();
                    displaySuccess("PN532 ready");
                    delay(UI_SUCCESS_DELAY_MS);
                    netMenu();
                    _selGuardUntilMs = millis() + SEL_DEBOUNCE_DELAY_MS;
                    while (check(SelPress)) delay(UI_SHORT_DELAY_MS);
                }
            }
            delay(UI_SHORT_DELAY_MS);
            continue;
        }

        if (millis() >= _selGuardUntilMs && check(SelPress)) {
            netMenu();
            _selGuardUntilMs = millis() + SEL_DEBOUNCE_DELAY_MS;
            while (check(SelPress)) delay(UI_SHORT_DELAY_MS);
        }

        if (!_bridgeTaskHandle) bridgeIoTick();

        if (returnToMenu) {
            disableAllBridges();
            returnToMenu = false;
            break;
        }
    }
}

// ————— I2C bridge helpers —————————————————————————————————

bool PN532I2CNetTools::waitIRQ(uint32_t timeoutMs) {
#if defined(PN532_IRQ)
    uint32_t start = millis();
    while (digitalRead(PN532_IRQ) != LOW) {
        if (check(EscPress)) {
            returnToMenu = true;
            return false;
        }
        if (millis() - start > timeoutMs) return false;
        delay(1);
        yield();
    }
    return true;
#else
    // Fallback: poll status byte via I2C
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (check(EscPress)) {
            returnToMenu = true;
            return false;
        }
        Wire.requestFrom((uint8_t)PN532_I2C_ADDR, (uint8_t)1);
        if (Wire.available()) {
            uint8_t status = Wire.read();
            if (status & 0x01) return true;  // IRQ asserted (frame ready)
        }
        delay(1);
    }
    return false;
#endif
}

// Read raw bytes from PN532 I2C (including the leading status byte which is discarded).
// Returns the frame bytes (without status byte) up to maxBytes.
std::vector<uint8_t> PN532I2CNetTools::readI2CData(uint32_t timeoutMs, int maxBytes) {
    std::vector<uint8_t> result;
    if (maxBytes < 2) return result;

    uint32_t start = millis();
    int req = min(maxBytes + 1, I2C_REQUEST_LIMIT); // +1 for status byte

    while (millis() - start < timeoutMs) {
        if (check(EscPress)) {
            returnToMenu = true;
            break;
        }

        int recv = Wire.requestFrom((uint8_t)PN532_I2C_ADDR, (uint8_t)req);
        if (recv >= 2) {
            Wire.read(); // discard status byte
            int remaining = recv - 1;
            result.reserve(remaining);
            while (Wire.available() && (int)result.size() < remaining) {
                result.push_back(Wire.read());
            }
            while (Wire.available()) Wire.read();
            return result;
        }

        while (Wire.available()) Wire.read();
        delay(1);
    }

    return result;
}

// ————— HSU frame detection ————————————————————————————————

bool PN532I2CNetTools::findCompleteFrame(size_t &frameStart, size_t &frameEnd) {
    const auto &b = _rxBuffer;
    for (size_t i = 0; i + 4 < b.size(); i++) {
        if (b[i] != 0x00 || b[i+1] != 0x00 || b[i+2] != 0xFF) continue;

        // Extended length frame?
        if (b[i+3] == 0xFF && i + 7 < b.size() && b[i+4] == 0xFF) {
            uint16_t extLen = ((uint16_t)b[i+5] << 8) | b[i+6];
            // Validate extended LCS: LEN_H + LEN_L + LCS == 0 mod 256
            if ((uint8_t)(b[i+5] + b[i+6] + b[i+7]) != 0) continue;
            size_t end = i + 8 + extLen + 1 + 1; // header(8) + data + DCS + postamble
            if (end <= b.size()) { frameStart = i; frameEnd = end; return true; }
        } else {
            uint8_t len = b[i+3];
            uint8_t lcs = b[i+4];
            if ((uint8_t)(len + lcs) != 0) continue;
            size_t end = i + 3 + 1 + 1 + len + 1 + 1; // 00 00 FF + LEN + LCS + data + DCS + 00
            if (end <= b.size()) { frameStart = i; frameEnd = end; return true; }
        }
    }
    return false;
}

// ————— Process one HSU frame via I2C ——————————————————————

void PN532I2CNetTools::processFrame(const uint8_t *frame, size_t len) {
    // 1. Write HSU frame directly to I2C PN532
    Wire.beginTransmission(PN532_I2C_ADDR);
    Wire.write(frame, (uint8_t)len);
    int writeErr = Wire.endTransmission();
    if (writeErr != 0) return;

    // 2. Wait for PN532 to assert IRQ (ACK ready)
    if (!waitIRQ(PN532_ACK_TIMEOUT_MS)) return;

    // 3. Drain ACK from I2C (status + 6 ACK bytes), but don't process it further
    Wire.requestFrom((uint8_t)PN532_I2C_ADDR, (uint8_t)ACK_FRAME_SIZE);
    while (Wire.available()) Wire.read();

    // 4. Forward ACK frame to network (mimic UART transparent behaviour)
    sendToNetwork(PN532_ACK_FRAME, sizeof(PN532_ACK_FRAME));

    // 5. Wait for response ready (IRQ asserts LOW again)
    if (!waitIRQ(PN532_RESPONSE_TIMEOUT_MS)) return;

    // 6. Read response frame from I2C
    auto resp = readI2CData(PN532_READ_TIMEOUT_MS, I2C_MAX_FRAME_SIZE);
    if (resp.empty()) return;

    // Find frame start in response (00 00 FF ...)
    size_t fStart = 0;
    for (size_t i = 0; i + 2 < resp.size(); i++) {
        if (resp[i] == 0x00 && resp[i+1] == 0x00 && resp[i+2] == 0xFF) { fStart = i; break; }
    }

    // Determine actual frame length from LEN byte
    size_t frameLen = resp.size(); // default: send all
    if (fStart + 4 < resp.size()) {
        uint8_t lenByte = resp[fStart + 3];
        size_t computed = fStart + 3 + 1 + 1 + lenByte + 1 + 1;
        if (computed <= resp.size()) frameLen = computed;
    }

    // 7. Forward response to network
    sendToNetwork(resp.data() + fStart, frameLen - fStart);
}

// ————— Drain buffered network data through I2C —————————————

void PN532I2CNetTools::drainNetToI2C() {
    size_t fs, fe;
    int frameCount = 0;
    while (frameCount < DRAIN_MAX_FRAMES && findCompleteFrame(fs, fe)) {
        if (check(EscPress)) {
            returnToMenu = true;
            break;
        }
        processFrame(_rxBuffer.data() + fs, fe - fs);
        _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + fe);
        ++frameCount;
    }
    // Remove garbage bytes before next preamble if buffer is large
    if (_rxBuffer.size() > RX_BUFFER_CLEANUP_THRESHOLD) _rxBuffer.clear();
}

// ————— Send data to all active network channels ———————————

void PN532I2CNetTools::sendToNetwork(const uint8_t *data, size_t len) {
    if (!len) return;

    if (bleDataTransferEnabled && pTxCharacteristic) {
        pTxCharacteristic->setValue((uint8_t *)data, len);
        pTxCharacteristic->notify();
    }
    if (_udpEnabled && _udpHasRemote) {
        _udp.beginPacket(_udpRemoteIP, _udpRemotePort);
        _udp.write(data, len);
        _udp.endPacket();
        _udpLastPacketMs = millis();
    }
    if (_tcpEnabled && _tcpHasClient && _tcpClient.connected()) {
        _tcpClient.write(data, len);
        _tcpLastPacketMs = millis();
    }
}

// ————— Menus ——————————————————————————————————————————————

void PN532I2CNetTools::netMenu() {
    bool done = false;
    int idx = 0;
    while (!done) {
        std::vector<Option> netOptions;
        netOptions.push_back({bleDataTransferEnabled ? "BLE:ON" : "BLE:OFF", [&]() {
            if (bleDataTransferEnabled) disableBleDataTransfer();
            else enableBleDataTransfer();
        }});
        netOptions.push_back({_udpEnabled ? "UDP:ON" : "UDP:OFF", [&]() {
            if (_udpEnabled) disableUdpDataTransfer();
            else if (WiFi.isConnected() || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)
                enableUdpDataTransfer();
            else wifiConnectMenu(TransportType::UDP);
        }});
        netOptions.push_back({_tcpEnabled ? "TCP:ON" : "TCP:OFF", [&]() {
            if (_tcpEnabled) disableTcpDataTransfer();
            else if (WiFi.isConnected() || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)
                enableTcpDataTransfer();
            else wifiConnectMenu(TransportType::TCP);
        }});
        netOptions.push_back({"Return", [&]() {
            disableAllBridges();
            returnToMenu = true;
            done = true;
        }});

        int selected = loopOptions(netOptions, MENU_TYPE_SUBMENU, _titleName.c_str(), idx, false);
        if (selected < 0 || done) break;
        idx = selected;
        while (check(SelPress)) delay(10);
    }
}

void PN532I2CNetTools::disableAllBridges() {
    if (bleDataTransferEnabled) disableBleDataTransfer();
    if (_udpEnabled) disableUdpDataTransfer();
    if (_tcpEnabled) disableTcpDataTransfer();
    maybeCloseOwnedAp();
}

void PN532I2CNetTools::maybeCloseOwnedAp() {
    if (_transportSwitchInProgress) return;
    if (_udpEnabled || _tcpEnabled) return;

    if (_staStartedByThisTool) {
        wifiDisconnect();
        _staStartedByThisTool = false;
    }

    if (!_apStartedByThisTool) return;

    WiFi.softAPdisconnect(false);

    // If AP was the only active mode, shut WiFi down completely.
    wifi_mode_t mode = WiFi.getMode();
    if (mode == WIFI_MODE_AP) {
        WiFi.mode(WIFI_OFF);
    }

    _apStartedByThisTool = false;
}

// ————— BLE enable / disable ———————————————————————————————

bool PN532I2CNetTools::enableBleDataTransfer() {
    if (bleDataTransferEnabled) return true;

    // I2C is half-duplex: only one network source at a time to prevent
    // interleaved commands and misrouted responses.
    if (_udpEnabled) disableUdpDataTransfer();
    if (_tcpEnabled) disableTcpDataTransfer();

    // Initialise the NimBLE stack only once; it persists across tool
    // instances.  Repeated deinit/init cycles corrupt internal state
    // and break BLE notifications.
    // Use NimBLEDevice::isInitialized() to detect if another module
    // called deinit() behind our back.
    if (!s_bleStackReady || !NimBLEDevice::isInitialized()) {
        s_bleStackReady = false;
        BLEDevice::init("BRUCE-PN532-BLE");
        NimBLEDevice::setSecurityAuth(true, true, true);
        pServer = BLEDevice::createServer();
        if (!pServer) { displayError("BLE Server Fail"); return false; }

        // Don't use advertiseOnDisconnect — it fires during deinit() and
        // causes rc=30 errors. Our callback handles reconnect instead.
        pServer->advertiseOnDisconnect(false);
        pServer->setCallbacks(new I2CNetServerCallbacks());

        pService = pServer->createService(BLE_SERVICE_UUID);
        if (!pService) { displayError("BLE Service Fail"); return false; }

        pTxCharacteristic = pService->createCharacteristic(
            BLE_TX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        if (!pTxCharacteristic) { displayError("BLE TX Fail"); return false; }

        pRxCharacteristic = pService->createCharacteristic(
            BLE_RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
        );
        if (!pRxCharacteristic) { displayError("BLE RX Fail"); return false; }

        pRxCharacteristic->setCallbacks(new I2CNetRxCallbacks());

        // Add service UUID to advertising data once during init;
        // clear first to remove stale data surviving BLEDevice::deinit().
        BLEAdvertising *pAdvert = pServer->getAdvertising();
        pAdvert->clearData();
        pAdvert->addServiceUUID(pService->getUUID());
        s_bleStackReady = true;
    }

    pServer->getAdvertising()->start();

    bleDataTransferEnabled = true;
    BLEConnected = false;
    printCenterFootnote("BLE Enabled");
    return true;
}

bool PN532I2CNetTools::disableBleDataTransfer() {
    if (!bleDataTransferEnabled) return true;

    if (pServer) {
        pServer->getAdvertising()->stop();
    }

    if (_rxMutex && xSemaphoreTake(_rxMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _rxBuffer.clear();
        xSemaphoreGive(_rxMutex);
    }
    bleDataTransferEnabled = false;
    BLEConnected = false;
    printCenterFootnote("BLE Disabled");
    return true;
}

// ————— UDP enable / disable ———————————————————————————————

bool PN532I2CNetTools::enableUdpDataTransfer() {
    if (_udpEnabled) return true;

    // I2C is half-duplex: only one network source at a time.
    if (bleDataTransferEnabled) disableBleDataTransfer();
    if (_tcpEnabled) {
        _transportSwitchInProgress = true;
        disableTcpDataTransfer();
        _transportSwitchInProgress = false;
    }

    if (!(WiFi.isConnected() || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)) {
        displayError("No WiFi"); return false;
    }
    if (!_udp.begin(UDP_LISTEN_PORT)) { displayError("UDP Fail"); return false; }
    _udpEnabled = true;
    _udpHasRemote = false;
    _udpLastPacketMs = millis();

    displayBanner();
    printSubtitle("UDP Bridge Mode");

    IPAddress ip = WiFi.isConnected() ? WiFi.localIP() :
                   (WiFi.getMode() != WIFI_OFF ? WiFi.softAPIP() : IPAddress(0,0,0,0));
    tft.setTextSize(FM);
    int margin = max(6, min(24, tftWidth / 16));
    int baseY = tftHeight / 2 - 10;
    tft.fillRect(margin - 4, baseY - 4, tftWidth - margin*2 + 8, FM*24 + 8, TFT_BLACK);
    tft.setCursor(margin, baseY);
    tft.print("UDP:" + ip.toString());
    tft.setCursor(margin, baseY + FM * 12);
    tft.print("Port: " + String(UDP_LISTEN_PORT));
    printCenterFootnote("Press OK to continue");
    while (!check(SelPress) && !check(EscPress)) delay(UI_SHORT_DELAY_MS);
    printCenterFootnote("Waiting for UDP client...");
    return true;
}

bool PN532I2CNetTools::disableUdpDataTransfer() {
    if (!_udpEnabled) return true;
    _udp.stop();
    _udpEnabled = false;
    _udpHasRemote = false;
    maybeCloseOwnedAp();
    displayInfo("UDP Off");
    delay(100);
    return true;
}

// ————— TCP enable / disable ———————————————————————————————

bool PN532I2CNetTools::enableTcpDataTransfer() {
    if (_tcpEnabled) return true;

    // I2C is half-duplex: only one network source at a time.
    if (bleDataTransferEnabled) disableBleDataTransfer();
    if (_udpEnabled) {
        _transportSwitchInProgress = true;
        disableUdpDataTransfer();
        _transportSwitchInProgress = false;
    }

    if (!(WiFi.isConnected() || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)) {
        displayError("No WiFi"); return false;
    }
    _tcpServer.stop();
    _tcpClient.stop();
    _tcpServer.begin();
    _tcpServer.setNoDelay(true);
    _tcpEnabled = true;
    _tcpHasClient = false;
    _tcpListenerStarting = true;
    _tcpListenerReadyChecks = 0;
    _tcpLastPacketMs = millis();

    displayBanner();
    printSubtitle("TCP Bridge Mode");
    IPAddress ip = WiFi.isConnected() ? WiFi.localIP() : WiFi.softAPIP();
    tft.setTextSize(FM);
    int margin = max(6, min(24, tftWidth / 16));
    int baseY = tftHeight / 2 - 10;
    tft.fillRect(margin - 4, baseY - 4, tftWidth - margin*2 + 8, FM*24 + 8, TFT_BLACK);
    tft.setCursor(margin, baseY);
    tft.print("TCP:" + ip.toString());
    tft.setCursor(margin, baseY + FM * 12);
    tft.print("Port: " + String(TCP_LISTEN_PORT));
    printCenterFootnote("Press OK to continue");
    while (!check(SelPress) && !check(EscPress)) delay(UI_SHORT_DELAY_MS);
    printCenterFootnote("Waiting TCP client...");
    return true;
}

bool PN532I2CNetTools::disableTcpDataTransfer() {
    if (!_tcpEnabled) return true;
    if (_tcpClient) _tcpClient.stop();
    _tcpServer.stop();
    _tcpEnabled = false;
    _tcpHasClient = false;
    _tcpListenerStarting = false;
    _tcpListenerReadyChecks = 0;
    maybeCloseOwnedAp();
    displayInfo("TCP Off");
    delay(100);
    return true;
}

// ————— WiFi connect menu ———————————————————————————————————

void PN532I2CNetTools::wifiConnectMenu(TransportType transport) {
    auto enableTransport = [&]() {
        if (transport == TransportType::TCP) enableTcpDataTransfer();
        else enableUdpDataTransfer();
    };

    if (WiFi.isConnected() || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        enableTransport();
        return;
    }
    bool done = false;
    int idx = 0;
    while (!done) {
        std::vector<Option> sel;
        sel.push_back({"Connect to Wifi", [&]() {
            displayInfo("Connecting...");
            bool startedByThisMenu = ::wifiConnectMenu(WIFI_STA);
            if (!WiFi.isConnected()) {
                displayError("Fail WiFi");
                return;
            }

            _staStartedByThisTool = startedByThisMenu;
            _apStartedByThisTool = false;
            enableTransport();
            done = true;
        }});
        sel.push_back({"Start WiFi AP", [&]() {
            displayInfo("Starting AP...");
            WiFi.mode(WIFI_AP);
            IPAddress apGateway(192, 168, 4, 1);
            WiFi.softAPConfig(apGateway, apGateway, IPAddress(255, 255, 255, 0));

            String apSsid = getSecureApSsid();
            String apPwd = getSecureApPassword();
            if (!WiFi.softAP(apSsid.c_str(), apPwd.c_str(), 6, 0, 4, false)) {
                displayError("AP start fail");
                return;
            }

            if (!waitForSoftApReady(SOFTAP_READY_TIMEOUT_MS)) {
                WiFi.softAPdisconnect(false);
                displayError("AP not ready");
                return;
            }

            _apStartedByThisTool = true;
            _staStartedByThisTool = false;

            printCenterFootnote("AP: " + apSsid);
            enableTransport();
            done = true;
        }});
        sel.push_back({"Return", [&]() { done = true; }});

        int selected = loopOptions(sel, MENU_TYPE_SUBMENU, "WiFi", idx, false);
        if (selected < 0 || done) break;
        idx = selected;
        while (check(SelPress)) delay(10);
    }
}

#endif // LITE_VERSION
