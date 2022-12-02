#include <esp_now.h>
#include <M5Stack.h>
#include <WiFi.h>

#include <ros.h>
#include <esp_now_ros/Packet.h>

ros::NodeHandle nh;
esp_now_ros::Packet msg_recv_packet;
ros::Publisher publisher("~recv", &msg_recv_packet);


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char mac_str[18];
  char buffer[256];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  msg_recv_packet.mac_address = mac_str;
  msg_recv_packet.mac_address_length = sizeof(mac_str);
  msg_recv_packet.data = buffer;
  msg_recv_packet.data_length = data_len;
  publisher.publish(&msg_recv_packet);
  nh.spinOnce();
}


void setup()
{
  nh.initNode("ESP_NOW_ROS");
  nh.advertise(publisher);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    nh.loginfo("ESPNow Init Success");
  } else {
    nh.logerror("ESPNow Init Failed");
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  while (nh.connected()) {
    nh.spinOnce();
  }
}
