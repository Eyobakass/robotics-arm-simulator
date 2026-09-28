/*
 * c_sender.c — Minimal C UDP sender for the robotics simulator.
 *
 * Demonstrates exactly what an embedded C application would send
 * to control the robot arm over UDP.
 *
 * Protocol: UTF-8 JSON string sent as a single UDP datagram.
 *
 * Compile (Windows - MSVC):
 *     cl c_sender.c /link ws2_32.lib
 *
 * Compile (Windows - MinGW/GCC):
 *     gcc c_sender.c -o c_sender.exe -lws2_32
 *
 * Compile (Linux/macOS):
 *     gcc c_sender.c -o c_sender -lm
 *
 * Run:
 *     ./c_sender            (sends to 127.0.0.1:9999)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── Platform-specific socket headers ─────────────────────────── */
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")

    /* Windows doesn't have usleep, use Sleep (milliseconds) */
    #include <windows.h>
    #define SLEEP_MS(ms) Sleep(ms)
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
    #define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

#define TARGET_HOST "127.0.0.1"
#define TARGET_PORT 9999
#define SEND_INTERVAL_MS 33   /* ~30 Hz */
#define PACKET_BUF_SIZE 512

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Build a JSON packet string — this is the core of the C protocol.
 *
 * In a real embedded system, this function would read joint encoders
 * and format the data for transmission.
 */
static int build_packet(char *buf, size_t buf_size,
                        const char *robot_name,
                        double base_rotation,
                        double shoulder,
                        double elbow)
{
    return snprintf(buf, buf_size,
        "{"
            "\"robot\":\"%s\","
            "\"timestamp\":%ld,"
            "\"joints\":{"
                "\"base_rotation\":%.2f,"
                "\"shoulder\":%.2f,"
                "\"elbow\":%.2f"
            "}"
        "}",
        robot_name,
        (long)time(NULL),
        base_rotation,
        shoulder,
        elbow
    );
}

int main(void)
{
    SOCKET sock;
    struct sockaddr_in dest;
    char packet[PACKET_BUF_SIZE];
    int packet_count = 0;
    double t = 0.0;
    double dt = SEND_INTERVAL_MS / 1000.0;

#ifdef _WIN32
    /* Initialize Winsock on Windows */
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
        return 1;
    }
#endif

    /* Create UDP socket */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "Failed to create socket\n");
        return 1;
    }

    /* Set up destination address */
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_HOST, &dest.sin_addr);

    printf("C UDP Sender -> %s:%d\n", TARGET_HOST, TARGET_PORT);
    printf("Sending joint angles at ~%.0f Hz...\n", 1000.0 / SEND_INTERVAL_MS);
    printf("Press Ctrl+C to stop.\n\n");

    /* ── Main send loop ─────────────────────────────────────────── */
    while (1) {
        /* Compute sinusoidal joint angles (same as Python wave mode) */
        double base_rot = 45.0 * sin(t * 0.5);
        double shoulder = 30.0 * sin(t * 0.7 + 1.0);
        double elbow    = 50.0 * sin(t * 1.1 + 2.0);

        /* Build the JSON packet */
        int len = build_packet(packet, sizeof(packet),
                               "demo_arm", base_rot, shoulder, elbow);

        /* Send via UDP */
        if (sendto(sock, packet, len, 0,
                   (struct sockaddr *)&dest, sizeof(dest)) == SOCKET_ERROR) {
            fprintf(stderr, "sendto() failed\n");
            break;
        }

        packet_count++;
        if (packet_count % 30 == 0) {
            printf("[%6.1fs] base: %+7.1f  shoulder: %+7.1f  elbow: %+7.1f\n",
                   t, base_rot, shoulder, elbow);
        }

        SLEEP_MS(SEND_INTERVAL_MS);
        t += dt;
    }

    /* Cleanup */
    closesocket(sock);
#ifdef _WIN32
    WSACleanup();
#endif

    printf("\nSent %d packets.\n", packet_count);
    return 0;
}
