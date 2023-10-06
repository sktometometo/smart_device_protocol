#include <vector>
#include <variant>

#include <M5Stack.h>
#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <esp_now_ros/Packet.h>
#include <packet_creator.h>
#include <packet_parser.h>
#include "uwb_module_util.h"
#include "iot_host_util.h"

#ifndef DEVICE_NAME
#define DEVICE_NAME "SDP_SESAMI_INTERFACE"
#endif

// ESP-NOW
uint8_t mac_address[6] = {0};
esp_now_peer_info_t peer_broadcast;

// Interface
std::string packet_description_operation = "Key control";
std::string serialization_format_operation = "s";
uint8_t buf_for_meta_packet[250];

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
uint8_t buf_for_uwb_packet[250];

// Switchbot Client Configuration
String wifi_ssid = "";
String wifi_password = "";
String sesami_device_uuid = "";
String sesami_secret_key = "";
String sesami_api_key = "";

// Other
uint8_t buf_for_data_packet[240];
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;
int loop_counter = 0;

void get_key_status_and_update_buf()
{
    Serial.printf("Get key status\n");

    String result = send_serial_command("{\"command\":\"status\"}\n");
    if (result.length() == 0)
    {
        Serial.println("Failed to get key status");
        return;
    }
    DeserializationError error = deserializeJson(result_json, result);
    if (error or (result_json.containsKey("success") and not result_json["success"].as<bool>()))
    {
        Serial.printf("deserializeJson() failed or Failed to get key status: %s\n", error.c_str());
        return;
    }
    else
    {
        String status = result_json["result"]["CHSesami2Status"].as<String>();
        bool locked = false;
        data.clear();
        data.push_back(SDPData(locked));
        generate_data_frame(buf_for_data_packet, packet_description_operation.c_str(), data);
        return;
    }
}

void show_device_config()
{
    String result = send_serial_command("{\"command\":\"get_device_config\"}\n");
    Serial.printf("Device config: %s\n", result.c_str());
}

bool load_config_from_FS(fs::FS &fs, String filename = "/config.json")
{
    StaticJsonDocument<1024> doc;

    auto file = fs.open(filename.c_str());
    if (!file)
    {
        Serial.printf("Failed to open config file from %s\n", filename.c_str());
        return false;
    }

    DeserializationError error = deserializeJson(doc, file.readString());
    if (error)
    {
        Serial.println("Failed to parse config file");
        return false;
    }

    if (not doc.containsKey("wifi_ssid") and not doc.containsKey("wifi_password") and not doc.containsKey("sesami_device_uuid") and not doc.containsKey("sesami_secret_key") and not doc.containsKey("sesami_api_key") and not doc.containsKey("uwb_id"))
    {
        Serial.println("Config file is invalid. It should have wifi_ssid, wifi_password, sesami_device_uuid, sesami_secret_key, sesami_api_key, and uwb_id");
        return false;
    }

    wifi_ssid = doc["wifi_ssid"].as<String>();
    wifi_password = doc["wifi_password"].as<String>();
    sesami_device_uuid = doc["sesami_device_uuid"].as<String>();
    sesami_secret_key = doc["sesami_secret_key"].as<String>();
    sesami_api_key = doc["sesami_api_key"].as<String>();
    uwb_id = doc["uwb_id"].as<int>();

    Serial.printf("wifi_ssid: %s\n", wifi_ssid.c_str());
    Serial.printf("wifi_password: %s\n", wifi_password.c_str());
    Serial.printf("sesami_device_uuid: %s\n", sesami_device_uuid.c_str());
    Serial.printf("sesami_secret_key: %s\n", sesami_secret_key.c_str());
    Serial.printf("sesami_api_key: %s\n", sesami_api_key.c_str());
    Serial.printf("uwb_id: %d\n", uwb_id);
    return true;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    uint8_t packet_type = get_packet_type(data);
    if (packet_type == esp_now_ros::Packet::PACKET_TYPE_DATA)
    {
        auto packet = parse_packet_as_data_packet(data);
        SDPInterfaceDescription packet_description_and_serialization_format = std::get<0>(packet);
        std::string packet_description = std::get<0>(packet_description_and_serialization_format);
        std::string serialization_format = std::get<1>(packet_description_and_serialization_format);
        std::vector<SDPData> body = std::get<1>(packet);

        if (packet_description == packet_description_operation and serialization_format == serialization_format_operation)
        {
            std::string operation_key = std::get<std::string>(body[0]);
            Serial.printf("operation_key: %s\n", operation_key.c_str());
            Serial.printf("operation_key length: %d\n", operation_key.length());
            if (operation_key == "lock")
            {
                Serial.printf("Lock the key\n");
                Serial2.print("{\"command\":\"lock\"}\n");
                auto timeout = millis() + 5000;
                while (millis() < timeout)
                {
                    if (Serial2.available())
                    {
                        String ret = Serial2.readStringUntil('\n');
                        Serial.printf("Response: %s\n", ret.c_str());
                        break;
                    }
                }
            }
            else if (operation_key == "unlock")
            {
                Serial.printf("Unlock the key\n");
                Serial2.print("{\"command\":\"unlock\"}\n");
                auto timeout = millis() + 5000;
                while (millis() < timeout)
                {
                    if (Serial2.available())
                    {
                        String ret = Serial2.readStringUntil('\n');
                        Serial.printf("Response: %s\n", ret.c_str());
                        break;
                    }
                }
            }
            else
            {
                Serial.printf("Unknown operation key\n");
            }
            Serial.printf("Key control command done\n");
        }
    }
}

void setup()
{
    esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

    M5.begin(true, false, true, false);
    M5.Lcd.printf("SDP SESAMI HOST DEVICE\n");
    M5.Lcd.printf("Name: %s\n", DEVICE_NAME);
    M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                  mac_address[4], mac_address[5]);

    // Load config from FS
    SPIFFS.begin();
    SD.begin();
    if (not load_config_from_FS(SD, "/config.json"))
    {
        if (not load_config_from_FS(SPIFFS, "/config.json"))
        {
            Serial.println("Failed to load config file");
            ESP.restart();
        }
    }

    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 16, 17);
    Serial2.begin(115200, SERIAL_8N1, 22, 21);

    // Initialization of ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (not esp_now_init() == ESP_OK)
    {
        ESP.restart();
    }
    memset(&peer_broadcast, 0, sizeof(peer_broadcast));
    for (int i = 0; i < 6; i++)
    {
        peer_broadcast.peer_addr[i] = (uint8_t)0xff;
    }
    esp_err_t addStatus = esp_now_add_peer(&peer_broadcast);
    if (addStatus != ESP_OK)
    {
        Serial.println("Failed to add peer");
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
    Serial.println("ESP NOW Initialized!");

    // SDP meta packet
    generate_meta_frame(buf_for_meta_packet, DEVICE_NAME, packet_description_operation.c_str(), serialization_format_operation.c_str(), "", "", "", "");

    // UWB module
    bool result = initUWB(false, uwb_id, Serial1);
    if (result)
    {
        Serial.println("UWB module initialized");
    }
    else
    {
        Serial.println("Failed to initialize UWB module");
    }
    data.clear();
    data.push_back(SDPData(uwb_id));
    generate_data_frame(
        buf_for_uwb_packet,
        packet_description_uwb.c_str(),
        data);

    // Sesami Client Configuration
    Serial2.printf("{\"command\":\"config_wifi\",\"ssid\":\"%s\",\"password\":\"%s\"}\n", wifi_ssid.c_str(), wifi_password.c_str());
    auto timeout = millis() + 20000;
    while (millis() < timeout)
    {
        if (Serial2.available())
        {
            String ret = Serial2.readStringUntil('\n');
            Serial.printf("Response for wifi config: %s\n", ret.c_str());
            break;
        }
    }
    Serial2.printf("{\"command\":\"config_sesami\",\"device_uuid\":\"%s\",\"secret_key\":\"%s\",\"api_key\":\"%s\"}\n", sesami_device_uuid.c_str(), sesami_secret_key.c_str(), sesami_api_key.c_str());
    timeout = millis() + 5000;
    while (millis() < timeout)
    {
        if (Serial2.available())
        {
            String ret = Serial2.readStringUntil('\n');
            Serial.printf("Response for switchbot config: %s\n", ret.c_str());
            break;
        }
    }

    //
    show_device_config();
}

void loop()
{
    delay(5000);

    // Run dummy callback if Serial available
    if (Serial.available())
    {
        uint8_t buf_dummy[240];
        data.clear();
        String str = Serial.readStringUntil('\n');
        Serial.printf("Input: %s\n", str.c_str());
        if (str.indexOf("unlock") != -1)
        {
            data.push_back(SDPData(std::string("unlock")));
        }
        else if (str.indexOf("lock") != -1)
        {
            data.push_back(SDPData(std::string("lock")));
        }
        else
        {
            Serial.println("Unknown command");
            return;
        }
        Serial.println("Generate data frame");
        std::string serialization_format = get_serialization_format(data);
        Serial.printf("serialization_format: %s\n", serialization_format.c_str());
        generate_data_frame(
            buf_dummy,
            packet_description_operation.c_str(),
            data);
        Serial.printf("Dummy callback calling\n");
        OnDataRecv(NULL, buf_dummy, sizeof(buf_dummy));
        Serial.printf("Dummy callback done\n");
        return;
    }

    if (loop_counter % 50 == 0)
    {
        get_key_status_and_update_buf();
    }

    // Send meta data
    esp_err_t result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf_for_meta_packet, sizeof(buf_for_meta_packet));
    if (result != ESP_OK)
    {
        Serial.printf("Send error: %d\n", result);
    }
    // Send UWB data
    result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf_for_uwb_packet, sizeof(buf_for_uwb_packet));
    if (result != ESP_OK)
    {
        Serial.printf("Send error: %d\n", result);
    }
    // Send SDP data
    result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf_for_data_packet, sizeof(buf_for_data_packet));
    if (result != ESP_OK)
    {
        Serial.printf("Send error: %d\n", result);
    }

    loop_counter++;
}
