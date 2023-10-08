#ifndef SDP_UTIL_H
#define SDP_UTIL_H

/*
 * SDP (Smart Device Protocol) Utility
 *
 * To use these functions, you need to include the following libraries:
 * - esp_now_ros/Packet.h
 * - packet_creator.h
 * - packet_parser.h
 */

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <esp_now_ros/Packet.h>
#include <packet_creator.h>
#include <packet_parser.h>

String _sdp_device_name;
esp_now_peer_info_t _peer_broadcast;

typedef void (*sdp_data_recv_cb_t)(std::vector<SDPData> &body);
typedef std::tuple<SDPInterfaceDescription, sdp_data_recv_cb_t> SDPInterfaceCallbackEntry;

std::vector<SDPInterfaceCallbackEntry> sdp_interface_callbacks;

void _OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    uint8_t packet_type = get_packet_type(data);
    if (packet_type == esp_now_ros::Packet::PACKET_TYPE_DATA)
    {
        auto packet = parse_packet_as_data_packet(data);
        SDPInterfaceDescription packet_description_and_serialization_format = std::get<0>(packet);
        std::string packet_description = std::get<0>(packet_description_and_serialization_format);
        std::string serialization_format = std::get<1>(packet_description_and_serialization_format);
        std::vector<SDPData> body = std::get<1>(packet);

        for (auto &entry : sdp_interface_callbacks)
        {
            if (packet_description == std::get<0>(std::get<0>(entry)) and
                serialization_format == std::get<1>(std::get<0>(entry)))
            {
                std::get<1>(entry)(body);
            }
        }
    }
}

bool _broadcast_sdp_meta_packet(const SDPInterfaceDescription &packet_description_and_serialization_format)
{
    uint8_t buf[245];
    const std::string &packet_description = std::get<0>(packet_description_and_serialization_format);
    const std::string &serialization_format = std::get<1>(packet_description_and_serialization_format);
    generate_meta_frame(buf, _sdp_device_name.c_str(), packet_description.c_str(), serialization_format.c_str(), "", "", "", "");
    bool success = esp_now_send(_peer_broadcast.peer_addr, buf, sizeof(buf)) == ESP_OK;
    return success;
}

void _meta_frame_broadcast_task(void *parameter)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        for (auto &entry : sdp_interface_callbacks)
        {
            const SDPInterfaceDescription &packet_description_and_serialization_format = std::get<0>(entry);
            _broadcast_sdp_meta_packet(packet_description_and_serialization_format);
        }
    }
}

bool init_sdp(uint8_t *mac_address, const String &device_name)
{
    if (mac_address != NULL)
    {
        esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
    }
    _sdp_device_name = device_name;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (not esp_now_init() == ESP_OK)
    {
        return false;
    }
    memset(&_peer_broadcast, 0, sizeof(_peer_broadcast));
    for (int i = 0; i < 6; i++)
    {
        _peer_broadcast.peer_addr[i] = (uint8_t)0xff;
    }
    esp_err_t addStatus = esp_now_add_peer(&_peer_broadcast);
    if (addStatus != ESP_OK)
    {
        return false;
    }
    xTaskCreate(_meta_frame_broadcast_task, "meta_frame_broadcast_task", 16384, NULL, 1, NULL);
    esp_now_register_recv_cb(_OnDataRecv);
    return true;
}

bool register_sdp_interface_callback(SDPInterfaceDescription packet_description_and_serialization_format, sdp_data_recv_cb_t callback)
{
    sdp_interface_callbacks.push_back(std::make_tuple(packet_description_and_serialization_format, callback));
    return true;
}

bool send_sdp_data_packet(std::string &packet_description, std::vector<SDPData> &body)
{
    uint8_t buf[240];
    generate_data_frame(buf, packet_description.c_str(), body);
    return esp_now_send(_peer_broadcast.peer_addr, buf, sizeof(buf)) == ESP_OK;
}

#endif // SDP_UTIL_H