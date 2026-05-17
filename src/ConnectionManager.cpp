#include "ConnectionManager.h"
#include "MAVLinkManager.h"
#if IBM
#include <winsock2.h>
#include <ws2tcpip.h> // For inet_pton
#pragma comment(lib, "Ws2_32.lib")
#endif
#if LIN || APL
#include <unistd.h>
#include <fcntl.h>      // For fcntl(), F_GETFL, F_SETFL, O_NONBLOCK
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // For TCP_NODELAY
#include <arpa/inet.h>
#endif
#include "XPLMUtilities.h"
#include <cstring>
#include <string>
#include <errno.h> 


#include <fstream>
#include <sstream>
#include "DataRefManager.h"
#include <ConfigManager.h>

#if LIN || APL
#define INVALID_SOCKET -1
#endif


static bool connected = false;
std::map<int, int> ConnectionManager::motorMappings;
int ConnectionManager::sockfd = -1;
int ConnectionManager::newsockfd = -1;
bool ConnectionManager::s_ppp_connection = false;

int ConnectionManager::sitlPort = 4560;
std::string ConnectionManager::status = "Disconnected";
std::string ConnectionManager::lastMessage = "";

#if IBM
bool ConnectionManager::initializeWinSock() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        XPLMDebugString("px4xplane: Could not initialize Winsock.\n");
        return false;
    }
    return true;
}
#endif


void ConnectionManager::setupServerSocket() {
    XPLMDebugString("px4xplane: Setting up server socket...\n");

    if (sockfd != -1 || connected) {
        XPLMDebugString("px4xplane: Server socket already set up or connected, aborting new setup attempt.\n");
        return;
    }

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        XPLMDebugString("px4xplane: Error opening socket.\n");
        status = "Socket Error";
        setLastMessage("Failed to create socket. System error.");
        XPLMSpeakString("Socket creation failed");
        return;
    }

    // CRITICAL FIX (January 2025): Allow immediate port reuse after disconnect
    // Without this, port 4560 enters TIME_WAIT state and remains unavailable for 30-60s
    // This caused "nothing happens" bug when user tried to reconnect quickly
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
                   (const char*)&reuse, sizeof(reuse)) < 0) {
        XPLMDebugString("px4xplane: Warning - could not set SO_REUSEADDR\n");
        // Continue anyway - not critical enough to abort
    }

#ifdef SO_REUSEPORT  // Linux/Mac have this, Windows doesn't
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT,
                   (const char*)&reuse, sizeof(reuse)) < 0) {
        // Ignore - not available on all platforms
    }
#endif

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;

    const std::string& bindIp = ConfigManager::sitl_ip;
    if (bindIp.empty() || bindIp == "0.0.0.0") {
        serv_addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, bindIp.c_str(), &serv_addr.sin_addr) != 1) {
            XPLMDebugString(("px4xplane: Invalid sitl_ip '" + bindIp + "', falling back to 0.0.0.0\n").c_str());
            serv_addr.sin_addr.s_addr = INADDR_ANY;
        }
    }

    const int bindPort = ConfigManager::sitl_port;
    serv_addr.sin_port = htons(static_cast<uint16_t>(bindPort));

    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        char errMsg[128];
        snprintf(errMsg, sizeof(errMsg),
            "px4xplane: Error on binding %s:%d.\n", bindIp.c_str(), bindPort);
        XPLMDebugString(errMsg);

        status = "Bind Error";
        char userMsg[128];
        snprintf(userMsg, sizeof(userMsg),
            "Failed to bind %s:%d. Port may be in use.", bindIp.c_str(), bindPort);
        setLastMessage(userMsg);
        XPLMSpeakString("Port bind failed");

        closeSocket(sockfd);
        sockfd = -1;
        return;
    }

    if (listen(sockfd, 5) < 0) {
        XPLMDebugString("px4xplane: Error on listen.\n");
        closeSocket(sockfd);
        return;
    }

    // CRITICAL FIX (January 2025): Make socket non-blocking
    // BEFORE: accept() was blocking → X-Plane froze until PX4 connected
    // AFTER: Non-blocking socket → poll in flight loop → no freezing
#if IBM
    u_long mode = 1;  // 1 = non-blocking, 0 = blocking
    if (ioctlsocket(sockfd, FIONBIO, &mode) != 0) {
        XPLMDebugString("px4xplane: Error setting socket to non-blocking mode.\n");
        closeSocket(sockfd);
        return;
    }
#elif LIN || APL
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0 || fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        XPLMDebugString("px4xplane: Error setting socket to non-blocking mode.\n");
        closeSocket(sockfd);
        return;
    }
#endif

    // UX FIX (January 2025): Update status for user visibility
    status = "Waiting for PX4 SITL...";
    {
        char readyMsg[160];
        snprintf(readyMsg, sizeof(readyMsg),
            "Server socket ready on %s:%d. Start PX4 SITL to connect.",
            ConfigManager::sitl_ip.c_str(), ConfigManager::sitl_port);
        setLastMessage(readyMsg);
        snprintf(readyMsg, sizeof(readyMsg),
            "px4xplane: Server socket ready on %s:%d, waiting for PX4 SITL to connect...\n",
            ConfigManager::sitl_ip.c_str(), ConfigManager::sitl_port);
        XPLMDebugString(readyMsg);
    }
    XPLMSpeakString("Waiting for PX4 connection");  // Audio feedback

    // NOTE: Don't call acceptConnection() here anymore - poll in flight loop instead

}


/**
 * @brief Non-blocking poll for incoming PX4 connection.
 *
 * CRITICAL FIX (January 2025): Non-blocking connection accept
 *
 * BEFORE: acceptConnection() used blocking accept() → X-Plane froze
 * AFTER: tryAcceptConnection() polls non-blocking socket → no freeze
 *
 * This function is called every flight loop frame when waiting for connection.
 * Returns immediately if no connection available (EWOULDBLOCK/EAGAIN).
 * Only accepts and initializes when PX4 actually connects.
 */
void ConnectionManager::tryAcceptConnection() {

    if (connected || sockfd == -1) {
        return;  // Already connected or no socket
    }

    sockaddr_in cli_addr{};
    socklen_t clilen = sizeof(cli_addr);

    newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);

    if (newsockfd < 0) {
        // Check if it's "no connection yet" (not an error) or real error
#if IBM
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK) {
            // No connection pending, not an error - just return and try next frame
            return;
        }
        XPLMDebugString("px4xplane: Error on accept (Windows error code: ");
        char errBuf[32];
        snprintf(errBuf, sizeof(errBuf), "%d)\n", err);
        XPLMDebugString(errBuf);
#elif LIN || APL
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // No connection pending, not an error - just return and try next frame
            return;
        }
        XPLMDebugString("px4xplane: Error on accept.\n");
#endif
        return;
    }

    // Set accepted socket non-blocking so sendData() never stalls the flight loop
    // (server socket sockfd is already non-blocking; accepted socket must be set separately)
#if IBM
    u_long nbMode = 1;
    ioctlsocket(newsockfd, FIONBIO, &nbMode);
#elif LIN || APL
    {
        int fl = fcntl(newsockfd, F_GETFL, 0);
        fcntl(newsockfd, F_SETFL, fl | O_NONBLOCK);
    }
#endif

    // Disable Nagle's algorithm so each MAVLink message is sent as its own TCP segment.
    // Required for PPP/fmu-v3 HIL mode: NuttX USART3 RX buffer is 1024 bytes at 115200 baud.
    // Without TCP_NODELAY, Nagle can batch consecutive HIL messages into one segment that
    // exceeds the NuttX TCP receive window, stalling delivery and causing QGC command-ack losses.
    {
        int noDelay = 1;
#if IBM
        setsockopt(newsockfd, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&noDelay), sizeof(noDelay));
#elif LIN || APL
        setsockopt(newsockfd, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
#endif
    }

    // Detect whether the peer is on the PPP tunnel (10.0.x.x) or a direct connection.
    // PPP link runs at 115200 baud over a physical UART — keep conservative buffers.
    // Direct connections (same-host SITL or LAN) support full throughput — maximize buffers.
    {
        char peer_ip[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &cli_addr.sin_addr, peer_ip, sizeof(peer_ip));
        s_ppp_connection = (strncmp(peer_ip, "10.0.", 5) == 0);

        char logBuf[128];
        if (s_ppp_connection) {
            snprintf(logBuf, sizeof(logBuf),
                "px4xplane: PPP peer %s — conservative socket buffers\n", peer_ip);
        } else {
            // Non-PPP: set large kernel send/receive buffers so the OS never stalls
            // the flight loop waiting for the TCP stack to drain a slow link.
            // SO_SNDBUF 4 MB: absorbs a full-rate burst of HIL sensor frames.
            // SO_RCVBUF 2 MB: ensures actuator messages are never dropped on the receive side.
            int sndbuf = 4 * 1024 * 1024;
            int rcvbuf = 2 * 1024 * 1024;
#if IBM
            setsockopt(newsockfd, SOL_SOCKET, SO_SNDBUF,
                       reinterpret_cast<const char*>(&sndbuf), sizeof(sndbuf));
            setsockopt(newsockfd, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));
#elif LIN || APL
            setsockopt(newsockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
            setsockopt(newsockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
#endif
            snprintf(logBuf, sizeof(logBuf),
                "px4xplane: direct peer %s — SO_SNDBUF 4MB SO_RCVBUF 2MB\n", peer_ip);
        }
        XPLMDebugString(logBuf);
    }

    // Successfully connected!
    XPLMDebugString("px4xplane: PX4 SITL connected successfully!\n");
    connected = true;

    // UX FIX (January 2025): Update status and notify user
    status = "Connected";
    setLastMessage("PX4 SITL connected successfully!");
    XPLMSpeakString("PX4 connected");  // Audio feedback

    // CRITICAL: Update menu to show "Disconnect from SITL"
    extern void updateMenuItems();  // Defined in px4xplane.cpp
    updateMenuItems();

    DataRefManager::enableOverride();

     DataRefManager::initializeMagneticField();
     XPLMDebugString("px4xplane: Init Magnetic Done.\n");

     // Initialize magnetic field with current aircraft position
     GeodeticPosition initialPosition = {
         DataRefManager::getFloat("sim/flightmodel/position/latitude"),
         DataRefManager::getFloat("sim/flightmodel/position/longitude"),
         DataRefManager::getFloat("sim/flightmodel/position/elevation")
     };

     DataRefManager::updateEarthMagneticFieldNED(initialPosition);
     DataRefManager::lastPosition = initialPosition;
     XPLMDebugString("px4xplane: Magnetic field initialized at current position.\n");

     // Load motor mappings from config.ini
     ConfigManager::loadConfiguration();
     XPLMDebugString("px4xplane: Motor mappings loaded from config.ini.\n");

     // Debug: Log loaded configuration
     std::string debugMsg = "Config Name: " + ConfigManager::getConfigName();
     XPLMDebugString(debugMsg.c_str());

}


//not working now ... cannot read ini file...
std::map<int, int> ConnectionManager::loadMotorMappings(const std::string& filename) {
    std::map<int, int> motorMappings;
    std::ifstream file(filename);

    if (!file) {
        XPLMDebugString("Error: Unable to open file ");
        XPLMDebugString(filename.c_str());
        XPLMDebugString("\n");
        return motorMappings;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key, equals, value;

        if (!(iss >> key >> equals >> value) || equals != "=") {
            XPLMDebugString("Warning: Ignoring malformed line: ");
            XPLMDebugString(line.c_str());
            XPLMDebugString("\n");
            continue;
        }

        // Check if the key is a PX4 motor
        if (key.substr(0, 4) == "PX4_") {
            int px4Motor = std::stoi(key.substr(4));
            int xplaneMotor = std::stoi(value);
            motorMappings[px4Motor] = xplaneMotor;
            XPLMDebugString("Loaded mapping: PX4 motor ");
            XPLMDebugString(std::to_string(px4Motor).c_str());
            XPLMDebugString(" -> X-Plane motor ");
            XPLMDebugString(std::to_string(xplaneMotor).c_str());
            XPLMDebugString("\n");
        }
    }

    if (motorMappings.empty()) {
        XPLMDebugString("Warning: No motor mappings loaded from ");
        XPLMDebugString(filename.c_str());
        XPLMDebugString("\n");
    }

    return motorMappings;
}




void ConnectionManager::disconnect() {
    const bool wasConnected = connected;
    const bool hadSocket = (sockfd != -1 || newsockfd != -1);
    if (!wasConnected && !hadSocket) {
        return;
    }

    // CRITICAL FIX (January 2025): Reset state BEFORE closing sockets
    // Order matters:
    //   1. Zero actuator values (prevents ghost commands)
    //   2. Clear MAVLink command history
    //   3. Disable override flags
    //   4. Close sockets

    if (wasConnected) {
        DataRefManager::resetActuatorValues();  // Zero all throttle/control surface datarefs
        MAVLinkManager::reset();                 // Clear actuator command history
        DataRefManager::disableOverride();       // Disable override flags
    }

    closeSocket(sockfd);
    sockfd = -1;
    closeSocket(newsockfd); // Close the newsockfd
    newsockfd = -1;

    connected = false;
    status = "Disconnected";
    setLastMessage(wasConnected ? "Disconnected from PX4 SITL." : "PX4 connection wait cancelled.");
    XPLMDebugString(wasConnected ? "px4xplane: Disconnected from SITL\n" : "px4xplane: PX4 connection wait cancelled\n");
    XPLMDebugString("px4xplane: Socket closed\n");

    // UX FIX (January 2025): Update menu and notify user
    if (wasConnected) {
        XPLMSpeakString("Disconnected");  // Audio feedback
    }
    extern void updateMenuItems();  // Defined in px4xplane.cpp
    updateMenuItems();  // Change menu back to "Connect to SITL"
}

void ConnectionManager::closeSocket(int& sockfd) {
    if (sockfd != INVALID_SOCKET) {
#if IBM
        closesocket(sockfd);
#elif LIN || APL
        close(sockfd);
#endif
        sockfd = -1; // set to -1 to indicate that the socket is no longer valid
    }



}
void ConnectionManager::sendData(const uint8_t* buffer, int len) {
    if (!connected) return;

    int totalBytesSent = 0;
    while (totalBytesSent < len) {
        int bytesSent = send(newsockfd, reinterpret_cast<const char*>(buffer) + totalBytesSent, len - totalBytesSent, 0);

        if (bytesSent < 0) {
#if IBM
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) {
                // TCP send buffer full (PX4 busy with QGC param download etc.) — drop this frame
                return;
            }
            char buf[256];
            snprintf(buf, sizeof(buf), "px4xplane: Error sending data: %d\n", err);
#elif LIN || APL
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // TCP send buffer full (PX4 busy with QGC param download etc.) — drop this frame
                return;
            }
            char buf[256];
            snprintf(buf, sizeof(buf), "px4xplane: Error sending data: %s\n", strerror(errno));
#endif
            XPLMDebugString(buf);
            return;
        }
        else if (bytesSent == 0) {
            XPLMDebugString("px4xplane: Peer has closed the connection\n");
            return;
        }

        totalBytesSent += bytesSent;
    }
}


bool ConnectionManager::isPppConnection()
{
    return s_ppp_connection;
}

void ConnectionManager::receiveData() {
    if (!connected) return;

    // PPP (10.0.x.x): 115200 baud → ~192 bytes/frame at 60 Hz — small budget.
    // Non-PPP: restore original budget (16 passes × 512 B) unchanged from
    // before PPP detection was added; SO_SNDBUF/SO_RCVBUF are still maximised
    // at accept() time (see tryAcceptConnection).
    const int max_passes    = s_ppp_connection ? 4   : 16;
    const int recv_buf_size = s_ppp_connection ? 256 : 512;
    uint8_t buffer[512];

    // Total MAVLink frames parsed this flight-loop frame, shared across all recv passes.
    // Both the recv loop (max_passes) and the MAVLink parser (max_frames) honour the
    // same budget so PPP gets ≤4 frames and non-PPP gets ≤16 frames end-to-end.
    int frames_parsed = 0;
    int passes = 0;
    for (; passes < max_passes && frames_parsed < max_passes; ++passes) {
        // Set up the read set and timeout for select
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(newsockfd, &readSet);
        struct timeval timeout;
        timeout.tv_sec = 0; // Zero seconds
        timeout.tv_usec = 0; // Zero microseconds

        // Use select to check if there is data available to read
        int result = select(newsockfd + 1, &readSet, NULL, NULL, &timeout);
        if (result < 0) {
            XPLMDebugString("px4xplane: Error in select\n");
            break;
        }
        if (result == 0 || !FD_ISSET(newsockfd, &readSet)) {
            break;
        }

        int bytesReceived = recv(newsockfd, reinterpret_cast<char*>(buffer), recv_buf_size, 0);
        if (bytesReceived < 0) {
            XPLMDebugString("px4xplane: Error receiving data\n");
            setLastMessage("Error receiving from PX4!"); // Store the received message
            break;
        }
        else if (bytesReceived == 0) {
            XPLMDebugString("px4xplane: PX4 closed the MAVLink socket\n");
            disconnect();
            return;
        }
        else if (bytesReceived > 0) {
            setLastMessage("Receiving from PX4!"); // Store the received message
            frames_parsed += MAVLinkManager::receiveHILActuatorControls(
                buffer, bytesReceived, max_passes - frames_parsed);
        }
    }

    if (frames_parsed >= max_passes && ConfigManager::debug_verbose_logging) {
        XPLMDebugString("px4xplane: MAVLink frame budget exhausted; remaining data will be processed next frame\n");
    }
}




bool ConnectionManager::isConnected() {
    return connected;
}

/**
 * @brief Check if socket is listening but not yet connected.
 *
 * Returns true when server socket is set up and waiting for PX4 to connect.
 * Used by flight loop to know when to poll for incoming connections.
 *
 * @return true if waiting for connection, false otherwise
 */
bool ConnectionManager::isWaitingForConnection() {
    return (sockfd != -1 && !connected);
}

const std::string& ConnectionManager::getStatus() {
    return status;
}

void ConnectionManager::setLastMessage(const std::string& message) {
    lastMessage = message;
}

const std::string& ConnectionManager::getLastMessage() {
    return lastMessage;
}
#if IBM
void ConnectionManager::cleanupWinSock() {
    WSACleanup();
}
#endif
