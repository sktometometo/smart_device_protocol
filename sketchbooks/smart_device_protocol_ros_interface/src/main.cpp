// Arduino
#if defined(M5STACKFIRE) or defined(M5STACKCORE2)
#include <Arduino.h>
#define LGFX_USE_V1
#else
#include <Arduino.h>
#endif

// LCD
#if defined(USE_DISPLAY)
#include "lcd.h"
#endif

// ROS
#include <sensor_msgs/Joy.h>
#include <smart_device_protocol/Packet.h>
#include <smart_device_protocol/UWBDistance.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/SetBool.h>

#include "ros/node_handle.h"
#if defined(M5STACKATOMS3)
#include "ArduinoAtomS3Hardware.h"
#else
#include "ArduinoHardware.h"
#endif

#if defined(M5STACKFIRE)
#include "m5stack_utils/m5stack.h"
#elif defined(M5ATOMLITE)
#include <M5Atom.h>

#include "m5stack_utils/m5atomlite.h"
CRGB dispColor(uint8_t r, uint8_t g, uint8_t b) {
  return (CRGB)((r << 16) | (g << 8) | b);
}
#elif defined(M5STICKCPLUS)
#include "m5stack_utils/m5stickcplus.h"
#elif defined(M5STACKCORE2)
#include "m5stack_utils/m5core2.h"
#elif defined(M5STACKATOMS3)
#include "m5stack_utils/m5atoms3.h"
#endif

#include <WiFi.h>

#include "devices/uwb_module_util.h"
#include "sdp/esp_now.h"

void uwb_toggle(const std_srvs::SetBoolRequest &, std_srvs::SetBoolResponse &);
void messageCb(const smart_device_protocol::Packet &);
void ledcolorCb(const std_msgs::ColorRGBA &);

// ESP-NOW
uint8_t device_mac_address[6] = {0};
uint8_t mac_address_for_msg[6];
uint8_t buffer_for_msg[256];

// UWB
bool uwb_initialized = false;

// Button Buf
constexpr int BUTTON_BUF_SIZE = 16;
int32_t button_buf[BUTTON_BUF_SIZE] = {0};

// ROSSerial
smart_device_protocol::Packet msg_recv_packet;
smart_device_protocol::UWBDistance msg_uwb;
sensor_msgs::Joy msg_joy;
ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;
ros::Publisher publisher("/smart_device_protocol/recv", &msg_recv_packet);
ros::Publisher publisher_uwb("/smart_device_protocol/uwb", &msg_uwb);
ros::Publisher publisher_joy("/smart_device_protocol/joy", &msg_joy);
ros::Subscriber<smart_device_protocol::Packet> subscriber("/smart_device_protocol/send", &messageCb);
ros::Subscriber<std_msgs::ColorRGBA> subscriber_color("/smart_device_protocol/led_color", &ledcolorCb);
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> uwb_toggle_service(
    "/smart_device_protocol/uwb_toggle", &uwb_toggle);

void uwb_toggle(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  if (req.data) {
    if (uwb_initialized) {
      res.success = true;
      res.message = "UWB is already enabled.";
    } else {
      uwb_initialized = initUWB(true, 1, Serial2);
      if (uwb_initialized) {
        res.success = true;
        res.message = "UWB is enabled.";
      } else {
        res.success = false;
        res.message = "UWB is not enabled.";
      }
    }
  } else {
    if (uwb_initialized) {
      resetUWB(Serial2);
      uwb_initialized = false;
      res.success = true;
      res.message = "UWB is disabled.";
    } else {
      res.success = true;
      res.message = "UWB is already disabled.";
    }
  }
}

void messageCb(const smart_device_protocol::Packet &msg) {
  if (msg.mac_address_length != 6) {
    nh.logerror("MAC Address length have to be 6.");
    return;
  }

  // Register a peer
  esp_now_peer_info_t peer_temp;
  memset(&peer_temp, 0, sizeof(peer_temp));
  for (int i = 0; i < 6; i++) {
    peer_temp.peer_addr[i] = msg.mac_address[i];
  }
  esp_err_t add_status = esp_now_add_peer(&peer_temp);
  esp_err_t result = esp_now_send(peer_temp.peer_addr, (uint8_t *)msg.data, msg.data_length);
  esp_err_t del_status = esp_now_del_peer(peer_temp.peer_addr);

  // Display
#if defined(USE_DISPLAY)
  clear_event_info();
  print_event_info("Send a packet");
  print_ros_message_info(msg);
  update_lcd();
#endif

  // ROS Logging
  nh.logdebug("Subscribe a message and send a packet.");
}

void ledcolorCb(const std_msgs::ColorRGBA &msg) {
  nh.loginfo("Receive a color message.");
  if (msg.r < 0.0 || msg.r > 1.0 || msg.g < 0.0 || msg.g > 1.0 || msg.b < 0.0 || msg.b > 1.0 || msg.a < 0.0 ||
      msg.a > 1.0) {
    nh.logerror("Color value have to be between 0.0 and 1.0.");
    return;
  }

#if defined(M5STACKATOMS3)
#elif defined(M5ATOMLITE)
  M5.dis.drawpix(0, dispColor(msg.r * 255, msg.g * 255, msg.b * 255));
#endif
}

// Display

#if defined(TESTING)
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
  uint8_t mac_addr[6];
  // copy esp_now_info->mac_addr to mac_addr
  for (int i = 0; i < 6; i++) {
    mac_addr[i] = esp_now_info->src_addr[i];
  }
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  for (int i = 0; i < 6; i++) {
    mac_address_for_msg[i] = mac_addr[i];
  }
  for (int i = 0; i < data_len; i++) {
    buffer_for_msg[i] = data[i];
  }
  msg_recv_packet.mac_address = mac_address_for_msg;
  msg_recv_packet.mac_address_length = 6;
  // msg_recv_packet.rssi = esp_now_info->rx_ctrl->rssi;
  msg_recv_packet.data = buffer_for_msg;
  msg_recv_packet.data_length = data_len;
  publisher.publish(&msg_recv_packet);
  nh.spinOnce();

  // Display
#if defined(USE_DISPLAY)
  clear_event_info();
  print_event_info("Receive a packet");
  print_ros_message_info(msg_recv_packet);
  update_lcd();
#endif

  // Log
  nh.logdebug("Received a packet and publish a message.");
}

void setup() {
#if defined(M5STACKFIRE)
  // M5.begin();
#elif defined(M5STACKCORE2)
  // M5.begin();
#elif defined(M5STACKATOMS3)
  // M5.begin();
#elif defined(M5STICKCPLUS)
  // M5.begin();
#elif defined(M5ATOMLITE)
  M5.begin(true, false, true);
#endif

  // UWB initialization
  Serial2.begin(115200, SERIAL_8N1, M5StackSerialPortInfoList[PORT_A].rx, M5StackSerialPortInfoList[PORT_A].tx);

  // Rosserial Initialization
  nh.initNode();
  while (not nh.connected()) {
    delay(1000);
    nh.spinOnce();
  }

  int tag_id = -1;
  nh.getParam("~tag_id", &tag_id, 1);

  // UWB initialization
  if (tag_id >= 0) {
    uwb_initialized = initUWB(true, tag_id, Serial2);
  } else {
    resetUWB(Serial2);
  }

  // Subscribe and Publish
  nh.advertise(publisher);
  nh.advertise(publisher_uwb);
  nh.advertise(publisher_joy);
  nh.advertiseService(uwb_toggle_service);
  nh.subscribe(subscriber);
  nh.subscribe(subscriber_color);
  while (not nh.connected()) {
    delay(1000);
    nh.spinOnce();
  }

  // ESP-NOW initialization
  if (not init_esp_now(device_mac_address, OnDataRecv)) {
    while (true) {
      delay(1000);
      nh.logerror("ESPNow Init Failed");
    }
  }

  // Log
  nh.loginfo("ESPNow Init Success");
  if (uwb_initialized) {
    char buf_str[128];
    sprintf(buf_str, "UWB Init Success. Tag ID: %d", tag_id);
    nh.loginfo(buf_str);
  } else {
    nh.loginfo("UWB Init Failed. Disabled.");
  }

  // LCD Initialization
#if defined(USE_DISPLAY)
  init_lcd();
#endif

  // Display
#if defined(USE_DISPLAY)
  clear_device_info();
  print_device_info(device_mac_address, uwb_initialized, tag_id);
  update_lcd();
#endif
}

void loop() {
#if defined(M5STACKFIRE)
#elif defined(M5ATOMLITE)
  M5.update();
  if (M5.Btn.isPressed()) {
    button_buf[0] = 1;
  } else {
    button_buf[0] = 0;
  }
#endif
  if (uwb_initialized) {
    auto ret = getDistanceUWB(Serial2);
    if (ret) {
      char buf_for_log[256];
      int id = std::get<0>(*ret);
      float distance = std::get<1>(*ret);
      msg_uwb.header.stamp = nh.now();
      msg_uwb.id = id;
      msg_uwb.distance = distance;
      publisher_uwb.publish(&msg_uwb);
      sprintf(buf_for_log, "id: %d, distance: %f", id, distance);
      nh.logdebug(buf_for_log);
    }
  }

  // Joy Publish
  msg_joy.header.stamp = nh.now();
  msg_joy.buttons_length = BUTTON_BUF_SIZE;
  msg_joy.buttons = button_buf;
  publisher_joy.publish(&msg_joy);

  nh.spinOnce();
  delay(100);
}
