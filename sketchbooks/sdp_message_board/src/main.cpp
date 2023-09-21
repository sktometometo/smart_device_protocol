#include <M5EPD.h>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <variant>
#include <vector>

#include <esp_now_ros/Packet.h>

#include <packet_creator.h>
#include <packet_parser.h>

#include <message.h>

#ifndef DEVICE_NAME
#define DEVICE_NAME "default_message_board"
#endif

const unsigned long duration_timeout = 1 * 60 * 1000;

M5EPD_Canvas canvas(&M5.EPD);
M5EPD_Canvas canvas_message(&M5.EPD);
uint8_t mac_address[6] = { 0 };
std::vector<Message> message_board;
esp_now_peer_info_t peer_broadcast;

const std::string packet_description_write = std::string("Message Board to write");
const std::string serialization_format_write = std::string("siS");

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
  return;
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len)
{
  uint8_t packet_type = get_packet_type(data);
  Serial.printf("Received packet. type: %d\n", packet_type);
  if (get_packet_type(data) == esp_now_ros::Packet::PACKET_TYPE_DATA)
  {
    auto ret = parse_packet_as_data_packet(data);
    std::string packet_description = std::get<0>(ret);
    std::string serialization_format = std::get<1>(ret);
    std::vector<std::variant<int32_t, float, std::string, bool>> data = std::get<2>(ret);
    if (packet_description == packet_description_write and serialization_format == serialization_format_write)
    {
      std::string source_name = std::get<std::string>(data[0]);
      int32_t duration_until_deletion = std::get<int32_t>(data[1]);
      std::string message = std::get<std::string>(data[2]);
      message_board.push_back(Message(source_name, message, duration_timeout));
      Serial.printf("Push message\n");
    }
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
  canvas.printf("ENR MESSAGE BOARD\n");
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
  uint8_t buf[250];
  generate_meta_frame(buf, DEVICE_NAME, packet_description_write.c_str(), serialization_format_write.c_str(), "", "",
                      "", "");
  esp_now_send(peer_broadcast.peer_addr, (uint8_t*)buf, sizeof(buf));
  canvas_message.clear();
  canvas_message.setCursor(0, 0);
  for (auto m = message_board.begin(); m != message_board.end();)
  {
    if (millis() > m->deadline)
    {
      m = message_board.erase(m);
    }
    else
    {
      canvas_message.printf("From: %s\n", m->source_name);
      canvas_message.printf("Duration until deletion(sec): %d\n", (int)((m->deadline - millis()) / 1000));
      canvas_message.printf("Message: %s\n\n", m->message);
      m->to_packet(buf);
      auto result = esp_now_send(peer_broadcast.peer_addr, (uint8_t*)buf, sizeof(buf));
      m++;
    }
  }
  canvas_message.pushCanvas(0, 100, UPDATE_MODE_DU4);
  delay(1000);
}
