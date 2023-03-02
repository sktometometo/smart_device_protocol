#include <M5EPD.h>

#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>

#include "M5_ENV.h"

uint8_t mac_address[6] = { 0 };
const char message[] = "Hello, World!";
esp_now_peer_info_t peer_broadcast;

QMP6988 qmp6988;

M5EPD_Canvas canvas(&M5.EPD);

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Sent to: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
                mac_addr[5]);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void broadcastMessage()
{
  esp_err_t result = esp_now_send(peer_broadcast.peer_addr, (uint8_t*)message, sizeof(message));
  Serial.print("Broadcasted: ");
  if (result == ESP_OK)
  {
    Serial.println("Success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

void setup()
{
  Serial.begin(115200);

  M5.begin(false, false, true, true, true);
  M5.EPD.SetRotation(90);
  M5.EPD.Clear(true);
  Wire1.begin(25, 32, (uint32_t)400000U);
  qmp6988.init((uint8_t)86U, &Wire1);

  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

  // Push title
  canvas.createCanvas(500, 100);
  canvas.setTextSize(3);
  canvas.drawString("Environment Broadcaster", 0, 0);
  canvas.pushCanvas(20, 20, UPDATE_MODE_DU);
  canvas.deleteCanvas();

  // Write MAC Address to Display
  canvas.createCanvas(500, 100);
  canvas.setTextSize(2);
  canvas.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  canvas.pushCanvas(20, 100, UPDATE_MODE_DU);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (not esp_now_init() == ESP_OK)
  {
    ESP.restart();
  }

  memset(&peer_broadcast, 0, sizeof(peer_broadcast));
  for (int i = 0; i < 6; ++i)
  {
    peer_broadcast.peer_addr[i] = (uint8_t)0xff;
  }
  esp_err_t addStatus = esp_now_add_peer(&peer_broadcast);

  esp_now_register_send_cb(OnDataSent);
}

void loop()
{
  float pressure = qmp6988.calcPressure();
  Serial.printf("Pressure: %f\n", pressure);

  canvas.createCanvas(540, 100);
  canvas.printf("pressure: %lf", pressure);

  delay(100);
}
