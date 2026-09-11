#pragma once
#include <cstdint>
#include <string>

// Small cross-platform (Winsock / BSD) UDP socket wrapper.
class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    bool open(uint16_t localPort, std::string* err = nullptr);      // localPort 0 = ephemeral
    void close();
    bool isOpen() const { return sock_ != kInvalid; }
    bool sendTo(const std::string& host, uint16_t port, const void* data, size_t len);
    // returns number of bytes, 0 on timeout, -1 on error
    int recvFrom(void* buf, size_t len, int timeoutMs, std::string* fromHost = nullptr, uint16_t* fromPort = nullptr);

private:
#ifdef _WIN32
    using Handle = uintptr_t;
    static constexpr Handle kInvalid = ~(uintptr_t)0;
#else
    using Handle = int;
    static constexpr Handle kInvalid = -1;
#endif
    Handle sock_ = kInvalid;
};
