#include <M5EPD.h>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include <esp_now_ros/Packet.h>

#include <packet_creator.h>
#include <packet_parser.h>

#include <message.h>

#ifndef DEVICE_NAME
#define DEVICE_NAME "default_message_board"
#endif

const unsigned long duration_timeout = 10 * 60 * 1000;

M5EPD_Canvas canvas(&M5.EPD);
uint8_t mac_address[6] = { 0 };
std::vector<std::pair<unsigned long, Message>> message_board;
esp_now_peer_info_t peer_broadcast;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
  return;
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len)
{
  if (get_packet_type(data) == esp_now_ros::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA)
  {
    message_board.push_back(std::pair<unsigned long, Message>(millis() + duration_timeout, Message(data)));
  }
}

void setup()
{
  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  M5.begin(true, false, true, true, false);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  M5.RTC.begin();

  canvas.createCanvas(540, 960);
  canvas.setTextSize(3);
  canvas.drawString("Hello World", 45, 350);
  canvas.pushCanvas(0, 0, UPDATE_MODE_DU4);

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
}

void loop()
{
  uint8_t buf[250];
  create_device_message_board_meta_packet(buf, DEVICE_NAME);
  for (auto m = message_board.begin(); m != message_board.end();)
  {
    if (millis() > m->first)
    {
      m = message_board.erase(m);
    }
    else
    {
      m->second.to_packet(buf);
      auto result = esp_now_send(peer_broadcast.peer_addr, (uint8_t*)buf, sizeof(buf));
      m++;
    }
  }
}
