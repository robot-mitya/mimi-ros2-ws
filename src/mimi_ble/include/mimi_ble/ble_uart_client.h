#ifndef BLE_UART_CLIENT_H
#define BLE_UART_CLIENT_H

#include <string>
#include <vector>
#include <functional>
#include <queue>
#include <sstream>
#include <sdbus-c++/IConnection.h>
#include <sdbus-c++/IProxy.h>

namespace mimi {

template<typename... Args>
std::string str(Args&&... args)
{
    std::ostringstream oss;
    (oss << ... << std::forward<Args>(args));
    return oss.str();
}

struct PairedDevice {
    std::string alias;
    std::string address;
    std::string path;
};

class BleUartClient {
public:
    using ConnectCallback = std::function<void(const std::string&, const std::string&, bool afterFailure)>;
    using DisconnectCallback = std::function<void(const std::string&, const std::string&, bool isFailure)>;
    using ErrorCallback = std::function<void(const std::string&, const std::string&, const std::string&, bool isConnected)>;
    using ReceiveCallback = std::function<void(const std::string&, const std::string&)>;

    BleUartClient(
        ConnectCallback connectCallback,
        DisconnectCallback disconnectCallback,
        ErrorCallback errorCallback,
        ReceiveCallback receiveCallback);
    ~BleUartClient();

    static std::vector<PairedDevice> listPairedDevices();
    bool connect(const std::string& alias, bool keepConnection);
    bool disconnect();
    [[nodiscard]] bool isConnected() const { return isConnected_; }
    [[nodiscard]] bool send(const std::string& text);
    void processCallbacks();
private:
    ConnectCallback connectCallback_;
    DisconnectCallback disconnectCallback_;
    ErrorCallback errorCallback_;
    ReceiveCallback receiveCallback_;

    std::string deviceAlias_;
    bool keepConnection_ = false;
    bool isConnected_ = false;

    std::atomic<bool> reconnecting_ = false;
    std::thread reconnectThread_;
    void startReconnectLoop();
    static constexpr int ReconnectIntervalInSeconds = 30;

    sdbus::IConnection* connection_ = nullptr;
    std::unique_ptr<sdbus::IProxy> deviceProxy_;
    std::unique_ptr<sdbus::IProxy> rxProxy_;
    std::string txCharPath_;
    std::string rxCharPath_;
    std::string rxAssembleBuffer_;

    std::queue<std::function<void()>> callbackQueue_;
    std::mutex callbackQueueMutex_;
    void postConnect(const std::string& message, bool afterFailure);
    void postDisconnect(const std::string& message, bool isFailure);
    void postReceive(const std::string& message);
    void postError(const std::string& message, const std::string& sdbusErrorName, bool isConnected);

    bool doConnect();
    bool findDevice(std::vector<PairedDevice> pairedDevices, PairedDevice& pairedDevice);
    bool connectGatt(const PairedDevice &pairedDevice);
    bool discoverCharacteristics();
    bool setupReceiveNotifications();
    void setupConnectionMonitor();
};

} // namespace mimi

#endif //BLE_UART_CLIENT_H
