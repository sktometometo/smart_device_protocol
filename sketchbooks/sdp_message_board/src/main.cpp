
#include <M5EPD.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <smart_device_protocol/Packet.h>

#include <vector>

#include "devices/uwb_module_util.h"
#include "epd.h"
#include "m5stack_utils/m5paper.h"
#include "message.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// CONFIG
String device_name;

// CANVAS
M5EPD_Canvas canvas_title(&M5.EPD);
M5EPD_Canvas canvas_status(&M5.EPD);
M5EPD_Canvas canvas_message(&M5.EPD);

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
SDPInterfaceDescription interface_description_uwb = std::make_tuple(packet_description_uwb, serialization_format_uwb);
std::vector<SDPData> data_for_uwb_data_packet;

// SDP
uint8_t mac_address[6] = {0};

// MSG BUFFER
std::vector<Message> message_board;

// Others
int loop_counter = 0;

void callback_for_v2(const uint8_t *mac_addr, const std::vector<SDPData> &body) {
  std::string source_name = std::get<std::string>(body[0]);
  int32_t duration_until_deletion = std::get<int32_t>(body[1]);
  std::string message = std::get<std::string>(body[2]);

  auto m = Message((char *)source_name.c_str(), (char *)message.c_str(), duration_until_deletion);
  message_board.push_back(m);
  Serial.printf("Push message from V2 Data\n");
}

bool load_config(fs::FS &fs, const String &filename) {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }
  if (not doc.containsKey("device_name") or
      not doc.containsKey("uwb_id")) {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();

  return true;
}

void setup() {
  // Init M5Paper
  M5.begin(false, true, true, true, false);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  M5.RTC.begin();
  init_epd(canvas_title, canvas_status, canvas_message);
  Serial.println("Start init");
  Serial1.begin(115200, SERIAL_8N1, M5StackSerialPortInfoList[PORT_C].rx, M5StackSerialPortInfoList[PORT_C].tx);

  // EPD
  canvas_title.printf("SDP MESSAGE BOARD\n");
  update_epd(canvas_title, canvas_status, canvas_message);

  // Load config
  if (not load_config(SD, "/config.json")) {
    Serial.println("Failed to load config");
    canvas_status.printf("Failed to load config\n");
    update_epd(canvas_title, canvas_status, canvas_message);
    while (true) {
      delay(1000);
    }
  }
  Serial.println("Config loaded");
  clear_canvas(canvas_status);
  canvas_status.printf("Config loaded\n");
  update_epd(canvas_title, canvas_status, canvas_message);

  // Show device name
  canvas_title.printf("Name: %s\n", device_name.c_str());
  update_epd(canvas_title, canvas_status, canvas_message);

  // Init SDP
  if (not init_sdp(mac_address, device_name)) {
    Serial.println("Failed to initialize SDP");
    canvas_status.printf("Failed to initialize SDP\n");
    update_epd(canvas_title, canvas_status, canvas_message);
    while (true) {
      delay(1000);
    }
  }
  if (not register_sdp_interface_callback(Message::get_interface_description(), callback_for_v2)) {
    Serial.println("Failed to register callback for V2");
    canvas_status.printf("Failed to register callback for V2\n");
    update_epd(canvas_title, canvas_status, canvas_message);
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SDP Initialized!");
  clear_canvas(canvas_status);
  canvas_status.printf("SDP Initialized!\n");
  update_epd(canvas_title, canvas_status, canvas_message);

  // Show Address
  canvas_title.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n",
                      mac_address[0], mac_address[1],
                      mac_address[2], mac_address[3],
                      mac_address[4], mac_address[5]);
  Serial.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n",
                mac_address[0], mac_address[1],
                mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  update_epd(canvas_title, canvas_status, canvas_message);

  // UWB Initialized
  if (uwb_id >= 0) {
    bool result = initUWB(false, uwb_id, Serial1);
    data_for_uwb_data_packet.clear();
    data_for_uwb_data_packet.push_back(SDPData(uwb_id));
    if (result) {
      Serial.println("Success for initialization of UWB");
      Serial.printf("UWB_ID: %d\n", uwb_id);
      canvas_title.printf("UWB_ID: %d\n", uwb_id);
    } else {
      uwb_id = -1;
      resetUWB(Serial1);
      Serial.println("Failed to initialize UWB");
      canvas_title.printf("Failed to initialize UWB\n");
    }
  } else {
    resetUWB(Serial1);
    Serial.println("UWB is not used");
    canvas_title.printf("UWB is not used\n");
  }
  update_epd(canvas_title, canvas_status, canvas_message);

  delay(3000);
}

void loop() {
  Serial.printf("Loop %d\n", loop_counter);
  uint8_t buf[250];

  // Show Battery voltage
  uint32_t battery_voltage = M5.getBatteryVoltage();
  clear_canvas(canvas_status);
  if (loop_counter % 2 == 0) {
    canvas_status.printf("+ Battery: %u\n", battery_voltage);
  } else {
    canvas_status.printf("x Battery: %u\n", battery_voltage);
  }

  // Show messages and send SDP packet
  clear_canvas(canvas_message);
  for (auto m = message_board.begin(); m != message_board.end();) {
    if (millis() > m->deadline) {
      m = message_board.erase(m);
    } else {
      m++;
    }
  }
  for (auto m = message_board.rbegin(); m != message_board.rend(); m++) {
    canvas_message.println("------------------------------------");
    canvas_message.printf("From: %s\n", m->source_name);
    canvas_message.printf("Duration until deletion(sec): %d\n", (int)((m->deadline - millis()) / 1000));
    canvas_message.printf("Message: %s\n\n", m->message);

    m->to_v2_packet(buf);
    broadcast_sdp_esp_now_packet((uint8_t *)buf, sizeof(buf));
    delay(10);
  }
  update_epd(canvas_title, canvas_status, canvas_message);

  // Send UWB packet
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(interface_description_uwb, data_for_uwb_data_packet)) {
      Serial.println("Failed to send UWB packet");
      canvas_status.printf("Failed to send UWB packet\n");
      update_epd(canvas_title, canvas_status, canvas_message);
    }
  }

  delay(100);
  loop_counter++;
}
