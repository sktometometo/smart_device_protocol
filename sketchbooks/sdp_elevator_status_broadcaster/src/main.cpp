#include <ArduinoJson.h>
#include <Dps310.h>
#if 1  // defined(M5STACK_CORE2)
#include <M5Core2.h>
#include <m5stack_utils/m5core2.h>
#elif defined(M5STACK_FIRE)
#include <M5Stack.h>
#include <m5stack_utils/m5stack.h>
#endif
#include <WiFi.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "devices/uwb_module_util.h"
#include "elevator_status.h"
#include "lcd.h"
#include "sdp/sdp.h"
#include "smart_device_protocol/Packet.h"
#include "utils.h"
#include "utils/calc_altitude.h"
#include "utils/config_loader.h"

LGFX lcd;
LGFX_Sprite sprite_header(&lcd);
LGFX_Sprite sprite_status(&lcd);
LGFX_Sprite sprite_info(&lcd);
LGFX_Sprite sprite_plot(&lcd);

Dps310 Dps310PressureSensor = Dps310();

/* Sensor */
float sensor_accX = 0.0F;
float sensor_accY = 0.0F;
float sensor_accZ = 0.0F;
float sensor_gyroX = 0.0F;
float sensor_gyroY = 0.0F;
float sensor_gyroZ = 0.0F;
float sensor_pitch = 0.0F;
float sensor_roll = 0.0F;
float sensor_yaw = 0.0F;
float sensor_temp_mpu = 0.0F;
float sensor_temp_dps = 0;
float sensor_pressure = 0;

/* Gravity */
float gravity_x = 0.0F;
float gravity_y = 0.0F;
float gravity_z = 0.0F;

/* Acceleration */
float accel_x_without_gravity = 0.0F;
float accel_y_without_gravity = 0.0F;
float accel_z_without_gravity = 0.0F;
float accel_on_gravity = 0.0F;

/* Elevator status */
float altitude = 0.;
int32_t current_floor = 0;
ElevatorMovingStatus current_status = HALT;

/* device */
uint8_t device_mac_address[6];
String device_name;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> body_uwb;

// SDP
// Current floor
std::string packet_description_floor = "Floor";
std::string serialization_format_floor = "i";
std::vector<SDPData> body_floor;
// Moving status
std::string packet_description_moving_status = "Moving Status";
std::string serialization_format_moving_status = "s";
std::vector<SDPData> body_moving_status;

// Port configuration
// Port A is used for Wire
M5StackSerialPortInfo port_info_uwb = M5StackSerialPortInfoList[PORT_C];

/* Elevator config */
std::vector<ElevatorConfig> elevator_config;
float moving_threshold;
float moving_status_timeout = 10.0;
float initial_altitude;
int32_t initial_floor;
long last_moving_stamp;

bool load_config_from_FS(fs::FS &fs, const String &filename) {
  StaticJsonDocument<1024> doc;
  if (!load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or
      not doc.containsKey("elevator_config") or
      not doc.containsKey("moving_status_threshold") or
      not doc.containsKey("uwb_id")) {
    sprite_status.println("Invalid config file");
    sprite_status.println("device_name and elevator_config and moving_status_threshold and uwb_id are required");
    sprite_status.pushSprite(0, lcd.height() / 3);
    return false;
  }

  if (doc.containsKey("moving_status_timeout")) {
    moving_status_timeout = doc["moving_status_timeout"].as<float>();
  }
  if (doc.containsKey("initial_floor")) {
    current_floor = doc["initial_floor"].as<int32_t>();
    initial_floor = current_floor;
  } else {
    current_floor = 7;
    initial_floor = 7;
  }

  uwb_id = doc["uwb_id"].as<int32_t>();
  device_name = doc["device_name"].as<String>();
  JsonArray elevator_config_json = doc["elevator_config"].as<JsonArray>();
  moving_threshold = doc["moving_status_threshold"].as<float>();
  for (auto itr = elevator_config_json.begin(); itr != elevator_config_json.end(); ++itr) {
    JsonObject e = *itr;
    if (e.containsKey("floor_num") and e.containsKey("floor_height")) {
      ElevatorConfig ec;
      ec.floor_num = e["floor_num"].as<int32_t>();
      ec.floor_height = e["floor_height"].as<float>();
      elevator_config.push_back(ec);
    }
  }
  return true;
}

void measure_sensors() {
  M5.IMU.getGyroData(&sensor_gyroX, &sensor_gyroY, &sensor_gyroZ);
  M5.IMU.getAccelData(&sensor_accX, &sensor_accY, &sensor_accZ);
  M5.IMU.getAhrsData(&sensor_pitch, &sensor_roll, &sensor_yaw);
  M5.IMU.getTempData(&sensor_temp_mpu);
  Dps310PressureSensor.measurePressureOnce(sensor_pressure);
  Dps310PressureSensor.measureTempOnce(sensor_temp_dps);
}

void postprocess_sensors() {
  auto accels_without_gravity = withdraw_gravity(sensor_accX, sensor_accY, sensor_accZ, gravity_x, gravity_y, gravity_z);
  accel_x_without_gravity = std::get<0>(accels_without_gravity);
  accel_y_without_gravity = std::get<1>(accels_without_gravity);
  accel_z_without_gravity = std::get<2>(accels_without_gravity);
  accel_on_gravity = calc_accel_on_gravity(accel_x_without_gravity, accel_y_without_gravity, accel_z_without_gravity, gravity_x, gravity_y, gravity_z);
}

void calc_elevator_status() {
  altitude = calc_altitude(sensor_pressure, sensor_temp_dps);
  if (fabs(accel_on_gravity) < moving_threshold) {
    if (millis() - last_moving_stamp > moving_status_timeout * 1000) {
      current_status = HALT;
      initial_floor = current_floor;
      initial_altitude = altitude;
      last_moving_stamp = millis();
      sprite_info.printf("Init floor updated: %d\n", initial_floor);
      sprite_info.printf("Init alt. updated: %.2f\n", initial_altitude);
      sprite_info.pushSprite(lcd.width() / 2, lcd.height() / 3);
    }
  } else {
    last_moving_stamp = millis();
  }
  auto floor = calc_floor(altitude, initial_altitude, initial_floor, elevator_config);
  Serial.printf("altitude: %.2f, floor: %d from initial_altitude: %.2f, initial_floor: %d\n", altitude, floor.has_value() ? floor.value() : -1, initial_altitude, initial_floor);
  // print_elevator_config_vector_serial(Serial, elevator_config);
  Serial.println("Elevator Config");
  for (const auto &entry : elevator_config) {
    Serial.printf("  Floor: %d, Height: %.2f\n", entry.floor_num, entry.floor_height);
  }
  if (floor.has_value()) {
    current_floor = floor.value();
  }
  current_status = calc_moving_status(accel_on_gravity, current_status, moving_threshold);
}

void setup() {
  // Device Initialization
  M5.begin(false, true, true, true);
  Serial.begin(115200);
  M5.IMU.Init();
  Dps310PressureSensor.begin(Wire, 0x77);
  Serial.println("Device initialized");

  // initialize LCD
  init_lcd();
  Serial.println("LCD initialized");

  // Load config
  SD.begin();
  SPIFFS.begin();
  if (not load_config_from_FS(SD, "/config.json")) {
    Serial.println("Failed to load config from SD");
    if (not load_config_from_FS(SPIFFS, "/config.json")) {
      Serial.println("Failed to load config from SPIFFS");
      sprite_status.println("Failed to load config");
      while (true) {
        delay(1000);
      }
    }
  }
  Serial.println("Config loaded!");

  Serial1.begin(115200, SERIAL_8N1, port_info_uwb.rx, port_info_uwb.tx);

  // Initialize ESP-NOW
  init_sdp(device_mac_address, device_name);

  // Print
  sprite_header.println("SDP ELV. STAT.");
  sprite_header.printf("device: %s\n", device_name.c_str());
  sprite_header.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       device_mac_address[0], device_mac_address[1], device_mac_address[2],
                       device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_header.pushSprite(0, 0);

  // UWB module
  if (uwb_id >= 0) {
    bool result = initUWB(false, uwb_id, Serial1);
    body_uwb.clear();
    body_uwb.push_back(SDPData(uwb_id));
    if (result) {
      sprite_header.printf("UWB ID: %d\n", uwb_id);
    } else {
      uwb_id = -1;
      sprite_header.printf("UWB ID: Failed to initialize\n");
    }
  } else {
    bool result = resetUWB(Serial1);
    sprite_header.printf("UWB ID: Not initialized\n");
  }
  sprite_header.pushSprite(0, 0);

  // Show elevator config
  print_elevator_config_vector(sprite_info, elevator_config, lcd.width() / 2, lcd.height() / 3);

  // Initialization
  std::vector<float> accels_x;
  std::vector<float> accels_y;
  std::vector<float> accels_z;
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("Gravity calibration will start in 5 second\n");
  sprite_status.pushSprite(0, lcd.height() / 3);
  delay(5000);
  measure_sensors();
  postprocess_sensors();
  altitude = calc_altitude(sensor_pressure, sensor_temp_dps);
  initial_altitude = altitude;
  Serial.printf("Initial altitude: %.2f, Initial floor: %d\n", initial_altitude, initial_floor);
  time_t duration_sec = 5.0;
  long deadline = millis() + (long)(duration_sec * 1000);
  while (millis() < deadline) {
    measure_sensors();
    accels_x.push_back(sensor_accX);
    accels_y.push_back(sensor_accY);
    accels_z.push_back(sensor_accZ);
    float left_sec = (deadline - millis()) / 1000.0;
    sprite_status.fillScreen(0xFFFFFF);
    sprite_status.setCursor(0, 0);
    sprite_status.printf("Gravity calibration running...\n");
    sprite_status.printf("%.2f secs remaining\n", left_sec);
    sprite_status.pushSprite(0, lcd.height() / 3);
  }
  std::tie(gravity_x, gravity_y, gravity_z) = calc_gravity_direction(accels_x, accels_y, accels_z);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("Gravity calibration finished\n");
  sprite_status.printf("Gravity: %.2f, %.2f, %.2f\n", gravity_x, gravity_y, gravity_z);
  delay(1000);
}

void loop() {
  measure_sensors();
  postprocess_sensors();
  calc_elevator_status();
  print_status();
  // Send SDP Data
  // Floor
  body_floor.clear();
  body_floor.push_back(SDPData(current_floor));
  if (not send_sdp_data_packet(packet_description_floor, body_floor)) {
    Serial.printf("Failed to send SDP data packet\n");
    Serial.printf("packet description is %s\n", packet_description_floor.c_str());
  }
  // Moving status
  body_moving_status.clear();
  body_moving_status.push_back(SDPData(moving_status_to_string(current_status)));
  if (not send_sdp_data_packet(packet_description_moving_status, body_moving_status)) {
    Serial.printf("Failed to send SDP data packet\n");
    Serial.printf("packet description is %s\n", packet_description_moving_status.c_str());
  }
  // UWB
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_uwb.c_str());
    }
  }
}
