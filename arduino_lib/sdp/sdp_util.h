#ifndef ESP_NOW_ROS_SDP_UTIL_H
#define ESP_NOW_ROS_SDP_UTIL_H

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

#include "sdp/packet_util.h"
#include "sdp/packet_creator.h"
#include "sdp/packet_parser.h"

String _sdp_device_name;
esp_now_peer_info_t _peer_broadcast;

typedef void (*sdp_data_if_recv_cb_t)(const uint8_t *mac_addr, const std::vector<SDPData> &body);
typedef void (*sdp_data_recv_cb_t)(const uint8_t *mac_addr, const SDPInterfaceDescription &interface_description, const std::vector<SDPData> &body);
typedef void (*sdp_meta_recv_cb_t)(const uint8_t *mac_addr, const std::string &device_name, const std::vector<SDPInterfaceDescription> &interfaces);
typedef std::tuple<SDPInterfaceDescription, sdp_data_if_recv_cb_t> SDPInterfaceCallbackEntry;

std::vector<SDPInterfaceCallbackEntry> _sdp_interface_data_callbacks;
std::vector<sdp_data_recv_cb_t> _sdp_data_callbacks;
std::vector<sdp_meta_recv_cb_t> _sdp_meta_callbacks;

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

        for (auto &entry : _sdp_data_callbacks)
        {
            entry(mac_addr, packet_description_and_serialization_format, body);
        }

        for (auto &entry : _sdp_interface_data_callbacks)
        {
            if (packet_description == std::get<0>(std::get<0>(entry)) and
                serialization_format == std::get<1>(std::get<0>(entry)))
            {
                std::get<1>(entry)(mac_addr, body);
            }
        }
    }
    else if (packet_type == esp_now_ros::Packet::PACKET_TYPE_META)
    {
        auto packet = parse_packet_as_meta_packet(data);
        std::string device_name = std::get<0>(packet);
        std::vector<SDPInterfaceDescription> interfaces = std::get<1>(packet);

        for (auto &entry : _sdp_meta_callbacks)
        {
            entry(mac_addr, device_name, interfaces);
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
        for (auto &entry : _sdp_interface_data_callbacks)
        {
            const SDPInterfaceDescription &packet_description_and_serialization_format = std::get<0>(entry);
            _broadcast_sdp_meta_packet(packet_description_and_serialization_format);
        }
        if (_sdp_interface_data_callbacks.size() == 0)
        {
            _broadcast_sdp_meta_packet(std::make_tuple("", ""));
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

bool register_sdp_interface_callback(SDPInterfaceDescription packet_description_and_serialization_format, sdp_data_if_recv_cb_t callback)
{
    // Check if the callback is already registered
    for (auto &entry : _sdp_interface_data_callbacks)
    {
        if (std::get<0>(std::get<0>(entry)) == std::get<0>(packet_description_and_serialization_format) and
            std::get<1>(std::get<0>(entry)) == std::get<1>(packet_description_and_serialization_format))
        {
            return false;
        }
    }
    _sdp_interface_data_callbacks.push_back(std::make_tuple(packet_description_and_serialization_format, callback));
    return true;
}

bool register_sdp_data_callback(sdp_data_recv_cb_t callback)
{
    _sdp_data_callbacks.push_back(callback);
    return true;
}

bool register_sdp_meta_callback(sdp_meta_recv_cb_t callback)
{
    _sdp_meta_callbacks.push_back(callback);
    return true;
}

bool send_sdp_data_packet(std::string &packet_description, std::vector<SDPData> &body)
{
    uint8_t buf[240];
    bool ret = generate_data_frame(buf, packet_description.c_str(), body);
    if (not ret)
    {
        return false;
    }
    else
    {
        return esp_now_send(_peer_broadcast.peer_addr, buf, sizeof(buf)) == ESP_OK;
    }
}

bool send_sdp_data_packet(const SDPInterfaceDescription &interface_description, std::vector<SDPData> &body)
{
    uint8_t buf[240];
    const std::string &packet_description = std::get<0>(interface_description);
    const std::string &serialization_format = std::get<1>(interface_description);
    bool ret = generate_data_frame(buf, packet_description.c_str(), serialization_format.c_str(), body);
    if (not ret)
    {
        return false;
    }
    else
    {
        return esp_now_send(_peer_broadcast.peer_addr, buf, sizeof(buf)) == ESP_OK;
    }
}

#endif // ESP_NOW_ROS_SDP_UTIL_H