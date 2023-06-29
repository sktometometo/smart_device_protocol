#include <esp_now.h>
#include <esp_system.h>

#if defined(M5STACKFIRE)
#include <Arduino.h>
#define LGFX_USE_V1
#elif defined(M5STACKCORE2)
#include <Arduino.h>
#define LGFX_USE_V1
#elif defined(M5STACKATOMS3)
#include <Arduino.h>
#endif
#include <WiFi.h>

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "ros/node_handle.h"
#if defined(M5StackATOMS3)
#include "ArduinoAtomS3Hardware.h"
#else
#include "ArduinoHardware.h"
#endif
#include <esp_now_ros/Packet.h>

void messageCb(const esp_now_ros::Packet&);

static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_event_info(&lcd);

esp_now_ros::Packet msg_recv_packet;
uint8_t mac_address_for_msg[6];
uint8_t buffer_for_msg[256];

ros::NodeHandle_<ArduinoHardware> nh;
ros::Publisher publisher("~recv", &msg_recv_packet);
ros::Subscriber<esp_now_ros::Packet> subscriber("~send", &messageCb);

void messageCb(const esp_now_ros::Packet& msg)
{
  if (msg.mac_address_length != 6)
  {
    nh.logerror("MAC Address length have to be 6.");
    return;
  }

  // Register a peer
  esp_now_peer_info_t peer_temp;
  memset(&peer_temp, 0, sizeof(peer_temp));
  for (int i = 0; i < 6; i++)
  {
    peer_temp.peer_addr[i] = msg.mac_address[i];
  }
  esp_err_t add_status = esp_now_add_peer(&peer_temp);

  // Send data
  esp_err_t result = esp_now_send(peer_temp.peer_addr, (uint8_t*)msg.data, msg.data_length);

  // Unregister the peer
  esp_err_t del_status = esp_now_del_peer(peer_temp.peer_addr);

  // Display
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setCursor(0, 0);
  sprite_event_info.println("send packet");
  sprite_event_info.printf("target mac: %02X:%02X:%02X:%02X:%02X:%02X\n", msg.mac_address[0], msg.mac_address[1],
                           msg.mac_address[2], msg.mac_address[3], msg.mac_address[4], msg.mac_address[5]);
  sprite_event_info.print("data: ");
  for (int i = 0; i < msg.data_length; i++)
  {
    sprite_event_info.printf("%d ", msg.data[i]);
  }
  sprite_event_info.println("");
  sprite_event_info.pushSprite(0, 80);

  // Log
  nh.logdebug("Subscribe a message and send a packet.");
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len)
{
  for (int i = 0; i < 6; i++)
  {
    mac_address_for_msg[i] = mac_addr[i];
  }
  for (int i = 0; i < data_len; i++)
  {
    buffer_for_msg[i] = data[i];
  }
  msg_recv_packet.mac_address = mac_address_for_msg;
  msg_recv_packet.mac_address_length = 6;
  msg_recv_packet.data = buffer_for_msg;
  msg_recv_packet.data_length = data_len;
  publisher.publish(&msg_recv_packet);
  nh.spinOnce();

  // Display
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setCursor(0, 0);
  sprite_event_info.println("recieve packet");
  sprite_event_info.printf("src mac: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_address_for_msg[0], mac_address_for_msg[1],
                           mac_address_for_msg[2], mac_address_for_msg[3], mac_address_for_msg[4],
                           mac_address_for_msg[5]);
  sprite_event_info.print("data: ");
  for (int i = 0; i < data_len; i++)
  {
    sprite_event_info.printf("%d ", data[i]);
  }
  sprite_event_info.println("");
  sprite_event_info.pushSprite(0, 80);

  // Log
  nh.logdebug("Received a packet and publish a message.");
}

void setup()
{
  // Read device mac address
  uint8_t device_mac_address[6] = { 0 };
  esp_read_mac(device_mac_address, ESP_MAC_WIFI_STA);

  // LCD Initialization
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
  sprite_event_info.createSprite(lcd.width(), lcd.height() * 2 / 3);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
#if defined(M5STACKATOMS3)
  sprite_device_info.setTextSize(1.0, 1.0);
  sprite_event_info.setTextSize(1.0, 1.0);
#else
  sprite_device_info.setTextSize(1.5, 1.5);
#endif
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setTextColor(0x000000);

  sprite_device_info.println("ESP-NOW ROS Driver");
  sprite_device_info.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", device_mac_address[0], device_mac_address[1],
                            device_mac_address[2], device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_device_info.pushSprite(0, 0);

  // Rosserial Initialization
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(subscriber);
  sprite_device_info.println("ROSSERIAL connecting...");
  sprite_device_info.pushSprite(0, 0);
  while (not nh.connected())
  {
    delay(1000);
    nh.spinOnce();
  }
  sprite_device_info.println("ROSSERIAL Initialized.");
  sprite_device_info.pushSprite(0, 0);

  // ESP-NOW initialization
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    nh.loginfo("ESPNow Init Success");
  }
  else
  {
    nh.logerror("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
  sprite_device_info.println("ESP-NOW Initialized.");
  sprite_device_info.pushSprite(0, 0);
}

void loop()
{
  nh.spinOnce();
  delay(5);
}
