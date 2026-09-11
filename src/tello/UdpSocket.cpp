#include "tello/UdpSocket.h"

#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {
#ifdef _WIN32
struct WsaInit {
    WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
    ~WsaInit() { WSACleanup(); }
};
void ensureWsa() { static WsaInit init; }
#endif
}

UdpSocket::UdpSocket() {
#ifdef _WIN32
    ensureWsa();
#endif
}

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::open(uint16_t localPort, std::string* err) {
    close();
#ifdef _WIN32
    SOCKET s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) { if (err) *err = "socket() failed: " + std::to_string(WSAGetLastError()); return false; }
#else
    int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) { if (err) *err = "socket() failed"; return false; }
#endif
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
    int rcvbuf = 1 << 20;
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(localPort);
    if (::bind(s, (sockaddr*)&addr, sizeof(addr)) != 0) {
#ifdef _WIN32
        if (err) *err = "bind(" + std::to_string(localPort) + ") failed: " + std::to_string(WSAGetLastError());
        closesocket(s);
#else
        if (err) *err = "bind(" + std::to_string(localPort) + ") failed";
        ::close(s);
#endif
        return false;
    }
    sock_ = (Handle)s;
    return true;
}

void UdpSocket::close() {
    if (sock_ == kInvalid) return;
#ifdef _WIN32
    closesocket((SOCKET)sock_);
#else
    ::close(sock_);
#endif
    sock_ = kInvalid;
}

bool UdpSocket::sendTo(const std::string& host, uint16_t port, const void* data, size_t len) {
    if (sock_ == kInvalid) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) return false;
    int n = ::sendto((
#ifdef _WIN32
        SOCKET
#else
        int
#endif
        )sock_, (const char*)data, (int)len, 0, (sockaddr*)&addr, sizeof(addr));
    return n == (int)len;
}

int UdpSocket::recvFrom(void* buf, size_t len, int timeoutMs, std::string* fromHost, uint16_t* fromPort) {
    if (sock_ == kInvalid) return -1;
    fd_set fds;
    FD_ZERO(&fds);
#ifdef _WIN32
    SOCKET s = (SOCKET)sock_;
#else
    int s = sock_;
#endif
    FD_SET(s, &fds);
    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    int r = ::select((int)s + 1, &fds, nullptr, nullptr, &tv);
    if (r == 0) return 0;
    if (r < 0) return -1;
    sockaddr_in from{};
#ifdef _WIN32
    int fromLen = sizeof(from);
#else
    socklen_t fromLen = sizeof(from);
#endif
    int n = ::recvfrom(s, (char*)buf, (int)len, 0, (sockaddr*)&from, &fromLen);
    if (n < 0) return -1;
    if (fromHost) {
        char ip[64] = {0};
        inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
        *fromHost = ip;
    }
    if (fromPort) *fromPort = ntohs(from.sin_port);
    return n;
}
