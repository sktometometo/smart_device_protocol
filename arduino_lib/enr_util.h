#include <esp_now.h>
#include <esp_system.h>
#include <WiFi.h>

esp_now_peer_info_t peer;

template <typename T>
bool init_esp_now(T &logprinter, uint8_t *device_mac_address, esp_now_recv_cb_t recv_callback = NULL, esp_now_send_cb_t send_callback = NULL)
{
    esp_read_mac(device_mac_address, ESP_MAC_WIFI_STA);
    logprinter.print("MAC Address: ");
    logprinter.println(WiFi.macAddress());

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK)
    {
        logprinter.println("ESPNow Init Failed");
        ESP.restart();
        return false;
    }

    if (recv_callback != NULL)
        esp_now_register_recv_cb(recv_callback);
    if (send_callback != NULL)
        esp_now_register_send_cb(send_callback);

    return true;
}
