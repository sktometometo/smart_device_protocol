#include <M5EPD.h>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <variant>

#include <esp_now_ros/Packet.h>

#include <packet_util.h>
#include <packet_creator.h>
#include <packet_parser.h>

#ifndef DEVICE_NAME
#define DEVICE_NAME "default_name"
#endif

const unsigned long duration_timeout = 1 * 60 * 1000;

const int packets_buffer_length = 3;

M5EPD_Canvas canvas(&M5.EPD);
M5EPD_Canvas canvas_message(&M5.EPD);
uint8_t mac_address[6] = {0};
esp_now_peer_info_t peer_broadcast;

std::vector<std::tuple<std::string, std::vector<std::tuple<std::string, std::string>>>> meta_packets;
std::vector<std::tuple<SDPInterfaceDescription, std::vector<SDPData>>> data_packets;
std::vector<unsigned long> meta_packets_timeouts;
std::vector<unsigned long> data_packets_timeouts;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  return;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  uint8_t packet_type = get_packet_type(data);
  Serial.printf("Received packet. type: %d\n", packet_type);
  if (get_packet_type(data) == esp_now_ros::Packet::PACKET_TYPE_META)
  {
    Serial.printf("Received meta packet\n");
    if (meta_packets.size() >= packets_buffer_length)
    {
      meta_packets.erase(meta_packets.begin());
      meta_packets_timeouts.erase(meta_packets_timeouts.begin());
    }
    meta_packets.push_back(parse_packet_as_meta_packet(data));
    meta_packets_timeouts.push_back(millis() + duration_timeout);
  }
  else if (get_packet_type(data) == esp_now_ros::Packet::PACKET_TYPE_DATA)
  {
    Serial.printf("Received data packet\n");
    Serial.print("data: ");
    for (int i = 0; i < data_len; ++i)
    {
      Serial.printf("%02x ", data[i]);
    }
    Serial.println();
    if (data_packets.size() >= packets_buffer_length)
    {
      data_packets.erase(data_packets.begin());
      data_packets_timeouts.erase(data_packets_timeouts.begin());
    }
    data_packets.push_back(parse_packet_as_data_packet(data));
    data_packets_timeouts.push_back(millis() + duration_timeout);
  }
  else
  {
    Serial.printf("Unknown packet type\n");
  }
}

void setup()
{
  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  M5.begin(false, false, true, true, false);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  M5.RTC.begin();
  Serial.println("Start init");

  canvas.createCanvas(540, 100);
  canvas.setTextSize(3);
  canvas.setCursor(0, 0);
  canvas.printf("SDP PACKET PRINTER\n");
  canvas.printf("Name: %s\n", DEVICE_NAME);
  canvas.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  canvas.pushCanvas(0, 0, UPDATE_MODE_DU4);
  canvas_message.createCanvas(540, 800);
  canvas_message.setTextSize(2);
  Serial.println("Display initialized.!");

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
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP NOW Initialized!");
}

void loop()
{
  Serial.println("Loop");
  uint8_t buf[250];
  canvas_message.clear();
  canvas_message.setCursor(0, 0);

  for (int i = 0; i < meta_packets.size(); ++i)
  {
    if (meta_packets_timeouts[i] < millis())
    {
      meta_packets.erase(meta_packets.begin() + i);
      meta_packets_timeouts.erase(meta_packets_timeouts.begin() + i);
      --i;
    }
  }
  for (int i = 0; i < data_packets.size(); ++i)
  {
    if (data_packets_timeouts[i] < millis())
    {
      data_packets.erase(data_packets.begin() + i);
      data_packets_timeouts.erase(data_packets_timeouts.begin() + i);
      --i;
    }
  }

  for (auto it = data_packets.begin(); it != data_packets.end(); ++it)
  {
    canvas_message.printf("== Data packet ==\n");
    auto packet_description_and_format_description = std::get<0>(*it);
    canvas_message.printf("Description: %s\n", std::get<0>(packet_description_and_format_description).c_str());
    canvas_message.printf("Serialization format: %s\n", std::get<1>(packet_description_and_format_description).c_str());
    canvas_message.printf("Data:\n");
    Serial.printf("== Data packet ==\n");
    Serial.printf("Description: %s\n", std::get<0>(packet_description_and_format_description).c_str());
    Serial.printf("Serialization format: %s\n", std::get<1>(packet_description_and_format_description).c_str());
    Serial.printf("Data:\n");

    for (auto it2 = std::get<1>(*it).begin(); it2 != std::get<1>(*it).end(); ++it2)
    {
      if (std::holds_alternative<int32_t>(*it2))
      {
        canvas_message.printf("  %d\n", std::get<int32_t>(*it2));
        Serial.printf("  %d\n", std::get<int32_t>(*it2));
      }
      else if (std::holds_alternative<float>(*it2))
      {
        canvas_message.printf("  %f\n", std::get<float>(*it2));
        Serial.printf("  %f\n", std::get<float>(*it2));
      }
      else if (std::holds_alternative<std::string>(*it2))
      {
        canvas_message.printf("  %s\n", std::get<std::string>(*it2).c_str());
        Serial.printf("  %s\n", std::get<std::string>(*it2).c_str());
      }
      else if (std::holds_alternative<bool>(*it2))
      {
        canvas_message.printf("  %s\n", std::get<bool>(*it2) ? "true" : "false");
        Serial.printf("  %s\n", std::get<bool>(*it2) ? "true" : "false");
      }
    }
  }
  for (auto it = meta_packets.begin(); it != meta_packets.end(); ++it)
  {
    canvas_message.printf("== Meta packet ==\n");
    canvas_message.printf("Description: %s\n", std::get<0>(*it).c_str());
    canvas_message.printf("Data:\n");
    for (auto it2 = std::get<1>(*it).begin(); it2 != std::get<1>(*it).end(); ++it2)
    {
      canvas_message.printf("  %s: %s\n", std::get<0>(*it2).c_str(), std::get<1>(*it2).c_str());
    }
    Serial.printf("== Meta packet ==\n");
    Serial.printf("Description: %s\n", std::get<0>(*it).c_str());
    Serial.printf("Data:\n");
    for (auto it2 = std::get<1>(*it).begin(); it2 != std::get<1>(*it).end(); ++it2)
    {
      Serial.printf("  %s: %s\n", std::get<0>(*it2).c_str(), std::get<1>(*it2).c_str());
    }
  }
  canvas_message.pushCanvas(0, 100, UPDATE_MODE_DU4);
  delay(1000);
}
