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
#define DEVICE_NAME "SDP_SWITCHBOT_LIGHT_INTERFACE"
#endif

// ESP-NOW
uint8_t mac_address[6] = {0};
esp_now_peer_info_t peer_broadcast;

// Interface
std::string packet_description_control = "Light control";
std::string packet_description_status = "Light status";
std::string serialization_format_control = "?";
uint8_t buf_for_meta_packet[250];

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
uint8_t buf_for_uwb_packet[250];

// Switchbot Client Configuration
String wifi_ssid = "";
String wifi_password = "";
String switchbot_device_id = "";
String switchbot_token = "";
String switchbot_secret = "";

// Other
uint8_t buf_for_data_packet[240];
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;
int loop_counter = 0;

void get_bot_status_and_update_buf()
{
    Serial.printf("Get switchbot status\n");
    String result = send_serial_command(
        String("") +
            "{\"command\":\"get_device_status\"," +
            "\"device_id\":\"" + switchbot_device_id + "\"}\n",
        5000);
    DeserializationError error = deserializeJson(result_json, result);
    if (error or (result_json.containsKey("success") and not result_json["success"].as<bool>()))
    {
        Serial.printf("deserializeJson() failed or get_device_status failed: %s, result: %s\n", error.c_str(), result.c_str());
        return;
    }
    else
    {
        String power = result_json["result"]["body"]["power"];
        data.clear();
        data.push_back(SDPData(power == "on" ? true : false));
        generate_data_frame(buf_for_data_packet, packet_description_status.c_str(), data);
        return;
    }
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

    if (not doc.containsKey("wifi_ssid") or not doc.containsKey("wifi_password") or not doc.containsKey("switchbot_token") or not doc.containsKey("switchbot_secret") or not doc.containsKey("switchbot_device_id") or not doc.containsKey("uwb_id"))
    {
        Serial.println("Config file is invalid. It should have wifi_ssid, wifi_password, switchbot_token, switchbot_secret, switchbot_device_id, and uwb_id");
        return false;
    }

    wifi_ssid = doc["wifi_ssid"].as<String>();
    wifi_password = doc["wifi_password"].as<String>();
    switchbot_token = doc["switchbot_token"].as<String>();
    switchbot_secret = doc["switchbot_secret"].as<String>();
    switchbot_device_id = doc["switchbot_device_id"].as<String>();
    uwb_id = doc["uwb_id"].as<int>();

    Serial.printf("wifi_ssid: %s\n", wifi_ssid.c_str());
    Serial.printf("wifi_password: %s\n", wifi_password.c_str());
    Serial.printf("switchbot_token: %s\n", switchbot_token.c_str());
    Serial.printf("switchbot_secret: %s\n", switchbot_secret.c_str());
    Serial.printf("switchbot_device_id: %s\n", switchbot_device_id.c_str());
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

        if (packet_description == packet_description_control and serialization_format == serialization_format_control)
        {
            Serial.printf("Length of body: %d\n", body.size());
            bool control = std::get<bool>(body[0]);
            Serial.printf("Light Control Command: %s\n", control ? "ON" : "OFF");
            if (control)
            {
                Serial.printf("Turn On the light.\n");
                String ret = send_serial_command(
                    String("") +
                        "{\"command\":\"send_device_command\"," +
                        "\"device_id\":\"" + switchbot_device_id + "\"," +
                        "\"sb_command_type\":\"command\"," +
                        "\"sb_command\":\"turnOn\"}\n",
                    10000);
                Serial.printf("Response: %s\n", ret.c_str());
            }
            else
            {
                Serial.printf("Turn Off the light.\n");
                Serial2.printf("{\"command\":\"send_device_command\",\"device_id\":\"%s\",\"sb_command_type\":\"command\",\"sb_command\":\"turnOff\"}\n", switchbot_device_id.c_str());
                String ret = send_serial_command(
                    String("") +
                        "{\"command\":\"send_device_command\"," +
                        "\"device_id\":\"" + switchbot_device_id + "\"," +
                        "\"sb_command_type\":\"command\"," +
                        "\"sb_command\":\"turnOff\"}\n",
                    10000);
                Serial.printf("Response: %s\n", ret.c_str());
            }
            Serial.printf("Light Control Command Done\n");
            get_bot_status_and_update_buf();
        }
    }
}

void setup()
{
    esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

    M5.begin(true, false, true, false);
    M5.Lcd.printf("SDP SWITCHBOT LIGHT HOST\n");
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
    generate_meta_frame(buf_for_meta_packet, DEVICE_NAME, packet_description_control.c_str(), serialization_format_control.c_str(), "", "", "", "");

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

    // Wifi Configuration
    Serial.printf("Wifi Configuration\n");
    String ret = send_serial_command(
        String("") +
            "{\"command\":\"config_wifi\"," +
            "\"ssid\":\"" + wifi_ssid + "\"," +
            "\"password\":\"" + wifi_password + "\"}\n",
        20000);
    Serial.printf("Response for wifi config: %s\n", ret.c_str());

    // Switchbot Client Configuration
    Serial.printf("Switchbot Client Configuration\n");
    Serial2.printf("{\"command\":\"config_switchbot\",\"token\":\"%s\",\"secret\":\"%s\"}\n", switchbot_token.c_str(), switchbot_secret.c_str());
    ret = send_serial_command(
        String("") +
            "{\"command\":\"config_switchbot\"," +
            "\"token\":\"" + switchbot_token + "\"," +
            "\"secret\":\"" + switchbot_secret + "\"}\n",
        5000);
    Serial.printf("Response for switchbot config: %s\n", ret.c_str());
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
        if (str.indexOf("turnOn") != -1)
        {
            data.push_back(SDPData(true));
        }
        else if (str.indexOf("turnOff") != -1)
        {
            data.push_back(SDPData(false));
        }
        else
        {
            Serial.printf("Unknown command\n");
            return;
        }
        Serial.println("Generate data frame");
        std::string serialization_format = get_serialization_format(data);
        Serial.printf("Length of data: %d\n", data.size());
        Serial.printf("serialization_format: %s\n", serialization_format.c_str());
        generate_data_frame(
            buf_dummy,
            packet_description_control.c_str(),
            data);
        Serial.printf("Dummy callback calling\n");
        OnDataRecv(NULL, buf_dummy, sizeof(buf_dummy));
        Serial.printf("Dummy callback called\n");
        return;
    }

    // Get switchbot status
    if (loop_counter % 50 == 0)
    {
        get_bot_status_and_update_buf();
    }

    // Send meta packet
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
    // send data packet
    result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf_for_data_packet, sizeof(buf_for_data_packet));
    if (result != ESP_OK)
    {
        Serial.printf("Send error: %d\n", result);
    }

    loop_counter++;
}
