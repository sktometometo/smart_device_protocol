#include <M5Stack.h>
#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_now_ros/Packet.h>
#include <packet_creator.h>
#include <packet_parser.h>
#include <vector>
#include <variant>

#define DEVICE_NAME "SDP_SESAMI_INTERFACE"

// ESP-NOW
uint8_t mac_address[6] = {0};
esp_now_peer_info_t peer_broadcast;

// Interface
std::string packet_description_operation = "Operation Key";
std::string serialization_format_operation = "s";
uint8_t buf_for_meta_packet[250];

// Other
uint8_t buf[240];
std::vector<SDPData> data;

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
    M5.Lcd.printf("SDP SESAMI HOST\n");
    M5.Lcd.printf("Name: %s\n", DEVICE_NAME);
    M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                  mac_address[4], mac_address[5]);

    Serial.begin(115200);
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
}

void loop()
{
    delay(5000);
    if (Serial.available())
    {
        data.clear();
        String str = Serial.readStringUntil('\n');
        Serial.printf("Input: %s\n", str.c_str());
        if (str.indexOf("unlock") != -1)
        {
            data.push_back(SDPData(std::string("unlock")));
        }
        else
        {
            data.push_back(SDPData(std::string("lock")));
        }
        Serial.println("Generate data frame");
        std::string serialization_format = get_serialization_format(data);
        Serial.printf("serialization_format: %s\n", serialization_format.c_str());
        generate_data_frame(
            buf,
            packet_description_operation.c_str(),
            data);
        Serial.printf("Dummy callback calling\n");
        OnDataRecv(NULL, buf, sizeof(buf));
        return;
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
}
