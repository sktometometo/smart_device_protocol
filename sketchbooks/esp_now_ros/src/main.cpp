#include <esp_now.h>
#include <M5Stack.h>
#include <WiFi.h>

#include <ros.h>
#include <esp_now_ros/Packet.h>

void messageCb(const esp_now_ros::Packet&);

ros::NodeHandle nh;
esp_now_ros::Packet msg_recv_packet;
ros::Publisher publisher("~recv", &msg_recv_packet);
ros::Subscriber<esp_now_ros::Packet> subscriber("~send", &messageCb);

void messageCb(const esp_now_ros::Packet& msg) {
  // Register peer
  esp_now_peer_info_t peer_temp;
  memset(&peer_temp, 0, sizeof(peer_temp));
  for (int i=0; i<6; i++) {
      peer_temp.peer_addr[i] = msg.mac_address[i];
  }
  esp_err_t add_status = esp_now_add_peer(&peer_temp);

  // Send data
  esp_err_t result = esp_now_send(peer_temp.peer_addr, (uint8_t*)msg.data, msg.data_length);

  // Unregister peer
  esp_err_t del_status = esp_now_del_peer(peer_temp.peer_addr);
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char mac_str[18];
  uint8_t buffer[256];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  uint8_t mac_address[6];
  for ( int i=0; i<6; i++ ) {
      mac_address[i] = mac_addr[i];
  }
  msg_recv_packet.mac_address = mac_address;
  msg_recv_packet.mac_address_length = 6;
  msg_recv_packet.data = buffer;
  msg_recv_packet.data_length = data_len;
  publisher.publish(&msg_recv_packet);
  nh.spinOnce();
}


void setup()
{
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(subscriber);

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
