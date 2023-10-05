#include <vector>
#include <variant>

#include <M5Stack.h>
#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <ArduinoJson.h>

#include <esp_now_ros/Packet.h>
#include <packet_creator.h>
#include <packet_parser.h>

#define DEVICE_NAME "SDP_SWITCHBOT_LIGHT_INTERFACE"

// ESP-NOW
uint8_t mac_address[6] = {0};
esp_now_peer_info_t peer_broadcast;

// Interface
std::string packet_description_control = "Light control";
std::string packet_description_status = "Light status";
std::string serialization_format_control = "?";
uint8_t buf_for_meta_packet[250];

// Switchbot Client Configuration
std::string device_id = "";
std::string wifi_ssid = "";
std::string wifi_password = "";
std::string switchbot_token = "";
std::string switchbot_secret = "";

// Other
uint8_t buf[240];
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    uint8_t packet_type = get_packet_type(data);
    if (packet_type == esp_now_ros::Packet::PACKET_TYPE_DATA)
    {
        Serial.printf("Received data packet\n");
        Serial.print("data: ");
        for (int i = 0; i < data_len; ++i)
        {
            Serial.printf("%02x ", data[i]);
        }
        Serial.println();

        auto packet = parse_packet_as_data_packet(data);
        SDPInterfaceDescription packet_description_and_serialization_format = std::get<0>(packet);
        std::string packet_description = std::get<0>(packet_description_and_serialization_format);
        std::string serialization_format = std::get<1>(packet_description_and_serialization_format);
        std::vector<SDPData> body = std::get<1>(packet);

        Serial.printf("packet_description: %s\n", packet_description.c_str());
        Serial.printf("serialization_format: %s\n", serialization_format.c_str());
        if (packet_description == packet_description_control and serialization_format == serialization_format_control)
        {
            try
            {
                Serial.printf("Length of body: %d\n", body.size());
                bool control = std::get<bool>(body[0]);
                Serial.printf("Light Control Command: %s\n", control ? "ON" : "OFF");
                if (control)
                {
                    Serial.printf("Turn On the light.\n");
                    Serial2.printf("{\"command\":\"send_device_command\",\"device_id\":\"%s\",\"sb_command_type\":\"command\",\"sb_command\":\"turnOn\"}\n", device_id.c_str());
                    auto timeout = millis() + 10000;
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
                    Serial.printf("Turn Off the light.\n");
                    Serial2.printf("{\"command\":\"send_device_command\",\"device_id\":\"%s\",\"sb_command_type\":\"command\",\"sb_command\":\"turnOff\"}\n", device_id.c_str());
                    auto timeout = millis() + 10000;
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
                Serial.printf("Light Control Command Done\n");
            }
            catch (const std::bad_variant_access &e)
            {
                Serial.printf("Bad variant access: %s\n", e.what());
            }
        }
        else
        {
            Serial.printf("Unknown packet description or serialization format\n");
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

    // Switchbot Client Configuration
    Serial.printf("Switchbot Client Configuration\n");
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
    Serial2.printf("{\"command\":\"config_switchbot\",\"token\":\"%s\",\"secret\":\"%s\"}\n", switchbot_token.c_str(), switchbot_secret.c_str());
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
}

void loop()
{
    delay(5000);

    // Run dummy callback if Serial available
    if (Serial.available())
    {
        data.clear();
        String str = Serial.readStringUntil('\n');
        Serial.printf("Input: %s\n", str.c_str());
        if (str.indexOf("turnOn") != -1)
        {
            data.push_back(SDPData(true));
        }
        else
        {
            data.push_back(SDPData(false));
        }
        Serial.println("Generate data frame");
        std::string serialization_format = get_serialization_format(data);
        Serial.printf("Length of data: %d\n", data.size());
        Serial.printf("serialization_format: %s\n", serialization_format.c_str());
        generate_data_frame(
            buf,
            packet_description_control.c_str(),
            data);
        Serial.printf("Dummy callback calling\n");
        OnDataRecv(NULL, buf, sizeof(buf));
        Serial.printf("Dummy callback called\n");
        return;
    }

    // Get switchbot status
    Serial.printf("Status of the bot\n");
    Serial2.printf("{\"command\":\"get_device_status\",\"device_id\":\"%s\"}\n", device_id.c_str());
    auto timeout = millis() + 5000;
    while (millis() < timeout)
    {
        if (Serial2.available())
        {
            String ret = Serial2.readStringUntil('\n');
            DeserializationError error = deserializeJson(result_json, ret);
            if (error)
            {
                Serial.printf("deserializeJson() failed: %s\n", error.c_str());
            }
            else
            {
                String power = result_json["result"]["body"]["power"];
                Serial.printf("Power: %s\n", power.c_str());
                data.clear();
                data.push_back(SDPData(power == "on" ? true : false));
                generate_data_frame(buf, packet_description_status.c_str(), data);
                break;
            }
            break;
        }
    }

    // Send data
    esp_err_t result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf_for_meta_packet, sizeof(buf_for_meta_packet));
    if (result == ESP_OK)
    {
        Serial.println("Sent SDP meta packet");
    }
    else
    {
        Serial.printf("Send error: %d\n", result);
    }
    result = esp_now_send(peer_broadcast.peer_addr, (uint8_t *)buf, sizeof(buf));
    if (result == ESP_OK)
    {
        Serial.println("Sent SDP data packet");
    }
    else
    {
        Serial.printf("Send error: %d\n", result);
    }
}
