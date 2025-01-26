#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"

#define WIFI_SSID "your_SSID"
#define WIFI_PASSWORD "your_PASSWORD"
#define UDP_SERVER_PORT 12345

void udp_receive() {
    struct netconn *conn;
    struct netbuf *buf;
    ip_addr_t src_ip;
    u16_t src_port;
    err_t err;
    char *data;
    u16_t len;

    // UDPソケットの作成
    conn = netconn_new(NETCONN_UDP);
    if (conn == NULL) {
        printf("UDP connection creation failed\n");
        return;
    }

    // ポートをバインド
    err = netconn_bind(conn, IP_ADDR_ANY, UDP_SERVER_PORT);
    if (err != ERR_OK) {
        printf("Failed to bind UDP socket\n");
        netconn_delete(conn);
        return;
    }

    printf("Listening on UDP port %d...\n", UDP_SERVER_PORT);

    // メッセージ受信のループ
    while (true) {
        // メッセージの受信
        err = netconn_recv(conn, &buf);
        if (err == ERR_OK) {
            // 受信データを取り出す
            netbuf_data(buf, (void**)&data, &len);
            printf("Received message: %.*s\n", len, data);
            netbuf_delete(buf);
        }
    }

    // 接続の削除（実際にはコードがここに到達しません）
    netconn_delete(conn);
}

int main() {
    stdio_init_all();

    // Wi-Fi接続の初期化
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed\n");
        return -1;
    }

    // Wi-Fi接続
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect(WIFI_SSID, WIFI_PASSWORD)) {
        printf("Wi-Fi connection failed\n");
        return -1;
    }

    printf("Connected to Wi-Fi\n");

    // UDP受信
    udp_receive();

    return 0;
}