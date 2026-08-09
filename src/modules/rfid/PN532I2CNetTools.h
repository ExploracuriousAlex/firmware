#ifndef __PN532I2CNETTOOLS_H__
#define __PN532I2CNETTOOLS_H__
#ifndef LITE_VERSION

#include <WiFi.h>
#include <WiFiUdp.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include <vector>

// PN532 protocol constants
#define PN532_I2C_ADDR 0x24  // 7-bit I2C address: 0x24 (0x48 with write bit, shifted right by 1 for I2C spec)

// Network port configuration
#define UDP_LISTEN_PORT 18888U
#define TCP_LISTEN_PORT 18889U

// PN532 protocol timing
#define PN532_ACK_TIMEOUT_MS 400UL
#define PN532_RESPONSE_TIMEOUT_MS 1200UL
#define PN532_READ_TIMEOUT_MS 500UL

// BLE/Network initialization delays
#define UI_SHORT_DELAY_MS 50UL
#define UI_MED_DELAY_MS 100UL
#define UI_LONG_DELAY_MS 150UL
#define UI_INFO_DELAY_MS 200UL
#define UI_SUCCESS_DELAY_MS 400UL
#define SEL_DEBOUNCE_DELAY_MS 600UL
#define SOFTAP_READY_TIMEOUT_MS 1500UL

// Bridge task configuration
#define BRIDGE_TASK_STACK_SIZE 6144U
#define BRIDGE_TASK_PRIORITY 1U
#define BRIDGE_IO_TICK_PERIOD_MS 2UL
#define BRIDGE_TASK_STOP_WAIT_LOOPS 50U
#define BRIDGE_TASK_WAIT_DELAY_MS 2UL

// Buffer and frame sizes
#define I2C_BUFFER_SIZE 256
#define I2C_MAX_FRAME_SIZE 127
#define I2C_REQUEST_LIMIT 128
#define RX_BUFFER_CLEANUP_THRESHOLD 512
#define DRAIN_MAX_FRAMES 16
#define ACK_FRAME_SIZE 7

// Transport timeouts
#define UDP_REMOTE_TIMEOUT_MS 60000UL
#define TCP_REMOTE_TIMEOUT_MS 60000UL

// BLE service/characteristic UUIDs
#define BLE_SERVICE_UUID      "0000fff0-0000-1000-8000-00805f9b34fb"
#define BLE_TX_CHAR_UUID      "0000fff1-0000-1000-8000-00805f9b34fb"
#define BLE_RX_CHAR_UUID      "0000fff2-0000-1000-8000-00805f9b34fb"

class PN532I2CNetTools {
public:
    PN532I2CNetTools();
    ~PN532I2CNetTools();

    // Called by BLE callback to enqueue received bytes
    void enqueueRx(const uint8_t *data, size_t len);
    bool tryEnterBleCallback();
    void leaveBleCallback();

private:
    void processPendingRx();
    String _titleName = "PN532 Net Bridge";
    std::atomic<bool> _initialized{false};
    uint32_t _selGuardUntilMs = 0;
    std::atomic<bool> _bridgeBusy{false};
    std::atomic<bool> _shuttingDown{false};
    std::atomic<uint32_t> _callbacksInFlight{0};
    TaskHandle_t _bridgeTaskHandle = nullptr;
    std::atomic<bool> _bridgeTaskRunning{false};
    SemaphoreHandle_t _rxMutex = nullptr;
    SemaphoreHandle_t _bridgeStoppedSem = nullptr;

    // Receive buffer: accumulates bytes from BLE/UDP/TCP
    std::vector<uint8_t> _rxBuffer;

    // Network state
    std::atomic<bool> _udpEnabled{false};
    WiFiUDP _udp;
    IPAddress _udpRemoteIP;
    uint16_t _udpRemotePort = 0;
    std::atomic<bool> _udpHasRemote{false};
    uint32_t _udpLastPacketMs = 0;

    std::atomic<bool> _tcpEnabled{false};
    WiFiServer _tcpServer = WiFiServer(TCP_LISTEN_PORT);
    WiFiClient _tcpClient;
    std::atomic<bool> _tcpHasClient{false};
    std::atomic<bool> _tcpListenerStarting{false};
    uint8_t _tcpListenerReadyChecks = 0;
    uint32_t _tcpLastPacketMs = 0;
    std::atomic<bool> _apStartedByThisTool{false};
    std::atomic<bool> _staStartedByThisTool{false};
    std::atomic<bool> _transportSwitchInProgress{false};

    void loop();
    bool initPN532();

    void displayBanner();
    void displayInitialScreen();
    void netMenu();

    // I2C HSU bridge
    bool findCompleteFrame(size_t &frameStart, size_t &frameEnd);
    void processFrame(const uint8_t *frame, size_t len);
    bool waitIRQ(uint32_t timeoutMs);
    std::vector<uint8_t> readI2CData(uint32_t timeoutMs, int maxBytes = 128);
    void sendToNetwork(const uint8_t *data, size_t len);

    // Network control
    bool enableBleDataTransfer();
    bool disableBleDataTransfer();
    bool enableUdpDataTransfer();
    bool disableUdpDataTransfer();
    bool enableTcpDataTransfer();
    bool disableTcpDataTransfer();
    void maybeCloseOwnedAp();
    enum class TransportType { UDP, TCP };
    void disableAllBridges();
    void wifiConnectMenu(TransportType transport);

    void startBridgeTask();
    void stopBridgeTask();
    static void bridgeTaskEntry(void *arg);
    void bridgeIoTick();

    void drainNetToI2C();
};

// Wrapper function for menu integration (same pattern as Chameleon(), Pn532ble(), etc.)
inline void PN532I2CNetBridge() {
    PN532I2CNetTools();
}

#endif // LITE_VERSION
#endif // __PN532I2CNETTOOLS_H__
