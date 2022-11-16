#include <esp_now.h>
#include <M5Stack.h>
#include <WiFi.h>


esp_now_peer_info_t slave;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  char msg[1];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: %s\n", macStr);
  Serial.printf("Last Packet Recv Data(%d): ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    msg[1] = data[i];
    Serial.print(msg[1]);
  }
  Serial.println("");
}
void setup() {
  Serial.begin(115200);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }

  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
  uint8_t data[13] = {72, 101, 108, 108, 111, 32, 69, 83, 80, 45, 78, 79, 87};
  esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
  delay(5000);
}
