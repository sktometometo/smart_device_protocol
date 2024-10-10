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

#include "elevator_status.h"
#include "sdp/sdp.h"
#include "smart_device_protocol/Packet.h"
#include "utils/calc_altitude.h"
#include "utils/config_loader.h"

static LGFX lcd;
static LGFX_Sprite sprite_header(&lcd);
static LGFX_Sprite sprite_status(&lcd);

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

/*  */

/* Elevator status */
float altitude = 0.;
int32_t current_floor = 0;

/* device */
uint8_t device_mac_address[6];
String device_name;

/* Elevator config */
std::vector<ElevatorConfig> elevator_config;

bool load_config_from_FS(fs::FS &fs, const String &filename) {
  StaticJsonDocument<1024> doc;
  if (!load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or not doc.containsKey("elevator_config")) {
    sprite_status.println("Invalid config file");
    sprite_status.println("device_name and elevator_config are required");
    sprite_status.pushSprite(0, lcd.height() / 3);
    return false;
  }

  device_name = doc["device_name"].as<String>();
  JsonArray elevator_config_json = doc["elevator_config"].as<JsonArray>();
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

void init_lcd() {
  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(lcd.width(), lcd.height() / 3);
  sprite_status.createSprite(lcd.width(), lcd.height() / 3);

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
}

void measure_sensors() {
  M5.IMU.getGyroData(&sensor_gyroX, &sensor_gyroY, &sensor_gyroZ);
  M5.IMU.getAccelData(&sensor_accX, &sensor_accY, &sensor_accZ);
  M5.IMU.getAhrsData(&sensor_pitch, &sensor_roll, &sensor_yaw);
  M5.IMU.getTempData(&sensor_temp_mpu);
  Dps310PressureSensor.measurePressureOnce(sensor_pressure);
  Dps310PressureSensor.measureTempOnce(sensor_temp_dps);
}

void calc_elevator_status() {
  altitude = calc_altitude(sensor_pressure, sensor_temp_dps);
}

std::tuple<float, float, float> calc_gravity_direction(std::vector<float> &accels_x, std::vector<float> &accels_y, std::vector<float> &accels_z) {
  float gravity_x = 0.0;
  float gravity_y = 0.0;
  float gravity_z = 0.0;
  for (auto &a : accels_x) {
    gravity_x += a;
  }
  for (auto &a : accels_y) {
    gravity_y += a;
  }
  for (auto &a : accels_z) {
    gravity_z += a;
  }
  gravity_x /= accels_x.size();
  gravity_y /= accels_y.size();
  gravity_z /= accels_z.size();
  return std::make_tuple(gravity_x, gravity_y, gravity_z);
}

void setup() {
  // Device Initialization
  M5.begin(false, false, true, true);
  Serial.begin(115200);
  M5.IMU.Init();
  Dps310PressureSensor.begin(Wire, 0x77);
  Serial.println("Device initialized");

  // initialize LCD
  init_lcd();
  Serial.println("LCD initialized");

  // Load config
  // SD.begin();
  // SPIFFS.begin();
  // if (not load_config_from_FS(SD, "/config.json")) {
  //   Serial.println("Failed to load config from SD");
  //   if (not load_config_from_FS(SPIFFS, "/config.json")) {
  //     Serial.println("Failed to load config from SPIFFS");
  //     while (true) {
  //       delay(1000);
  //     }
  //   }
  // }

  // Initialize ESP-NOW
  init_sdp(device_mac_address, String("elevator_status"));

  // Print
  sprite_header.println("SDP ELV. STAT.");
  sprite_header.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       device_mac_address[0], device_mac_address[1], device_mac_address[2],
                       device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_header.pushSprite(0, 0);
}

void loop() {
  measure_sensors();
  calc_elevator_status();
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("Acc: %.2f, %.2f, %.2f\n", sensor_accX, sensor_accY, sensor_accZ);
  sprite_status.printf("Gyro: %.2f, %.2f, %.2f\n", sensor_gyroX, sensor_gyroY, sensor_gyroZ);
  sprite_status.printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", sensor_pitch, sensor_roll, sensor_yaw);
  sprite_status.printf("Pressure: %.2f\n", sensor_pressure);
  sprite_status.printf("Temp: %.2f, %.2f\n", sensor_temp_mpu, sensor_temp_dps);
  sprite_status.printf("Altitude: %.2f\n", altitude);
  sprite_status.pushSprite(0, lcd.height() / 3);
  Serial.println("loop");
}
