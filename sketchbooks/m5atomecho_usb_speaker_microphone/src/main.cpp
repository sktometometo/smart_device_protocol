#include <Arduino.h>
#include <M5Atom.h>
#include <driver/i2s.h>

#include <vector>

// ROS
#include <audio_common_msgs/AudioData.h>

#include "ArduinoHardware.h"
#include "ros/node_handle.h"

// Microphone
constexpr int CONFIG_I2S_BCK_PIN = 19;
constexpr int CONFIG_I2S_LRCK_PIN = 33;
constexpr int CONFIG_I2S_DATA_PIN = 22;
constexpr int CONFIG_I2S_DATA_IN_PIN = 23;

constexpr i2s_port_t SPEAK_I2S_NUMBER = I2S_NUM_0;
constexpr int MODE_MIC = 0;
constexpr int MODE_SPK = 1;
constexpr int I2S_BUFFER_COUNT = 4;
constexpr int I2S_BUFFER_SIZE = 1024;

uint8_t buffer[I2S_BUFFER_SIZE];
size_t transBytes;
int16_t previous_x = 0;
int16_t previous_y = 0;
int16_t tmp_x;

TaskHandle_t i2sTaskHandle = NULL;

// ROS
ros::NodeHandle_<ArduinoHardware, 10, 10, 4096, 4096> nh;
audio_common_msgs::AudioData audio_data_msg;
ros::Publisher audio_pub("audio", &audio_data_msg);
constexpr int AUDIO_BUFF_LENGTH = 1024;
uint8_t audio_data_buff[AUDIO_BUFF_LENGTH];

bool InitI2SSpeakOrMic(int mode, int sample_rate, i2s_bits_per_sample_t bits_per_sample) {
  esp_err_t err = ESP_OK;

  i2s_driver_uninstall(SPEAK_I2S_NUMBER);
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER),
      .sample_rate = sample_rate,
      .bits_per_sample = bits_per_sample,
      .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = I2S_BUFFER_COUNT,
      .dma_buf_len = I2S_BUFFER_SIZE,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};
  if (mode == MODE_MIC) {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  } else {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.tx_desc_auto_clear = true;
  }

  err += i2s_driver_install(SPEAK_I2S_NUMBER, &i2s_config, 0, NULL);
  i2s_pin_config_t tx_pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
  tx_pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

  tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
  tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
  tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
  tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;
  err += i2s_set_pin(SPEAK_I2S_NUMBER, &tx_pin_config);
  err += i2s_set_clk(SPEAK_I2S_NUMBER, sample_rate, I2S_BITS_PER_SAMPLE_16BIT,
                     I2S_CHANNEL_MONO);
  return true;
}

void setup() {
  M5.begin(true, false, true);
  M5.dis.clear();
  M5.dis.drawpix(0, CRGB(255, 0, 0));

  // ROS ノードの初期化
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(audio_pub);

  InitI2SSpeakOrMic(MODE_MIC, 8000, I2S_BITS_PER_SAMPLE_16BIT);

  // メッセージデータの初期化
  audio_data_msg.data_length = AUDIO_BUFF_LENGTH;
  audio_data_msg.data = audio_data_buff;

  M5.dis.clear();
  M5.dis.drawpix(0, CRGB(255, 255, 255));
}

void loop() {
  i2s_read(I2S_NUM_0, (char*)buffer, I2S_BUFFER_SIZE, &transBytes, portMAX_DELAY);
  //   for (int i = 0; i < transBytes; i += 2) {
  //     uint16_t* val = (uint16_t*)&buffer[i];

  //     // Process the 12-bit audio sample, adjust and scale it to 16-bit range
  //     int16_t p = (((0x0fff - (*val & 0x0fff)) * 16) - 0x8000);
  //     int16_t tmp_x = p;

  //     /* https://stackoverflow.com/questions/77383602/do-these-two-dc-filter-algorithms-achieve-the-same-thing-and-is-one-better */
  //     /* apply DC filter */
  //     p = p - previous_x + 0.995 * previous_y;
  //     previous_y = p;
  //     previous_x = tmp_x;

  //     /* Set p value to audio_data_buff */
  //     ((int16_t*)audio_data_buff)[i / 2] = p;
  //   }
  audio_data_msg.data = buffer;
  audio_data_msg.data_length = transBytes;

  //   audio_data_msg.data = audio_data_buff;
  //   audio_data_msg.data_length = transBytes;
  audio_pub.publish(&audio_data_msg);
  char message[100];
  sprintf(message, "Published audio data length: %d", audio_data_msg.data_length);
  nh.loginfo(message);
  nh.spinOnce();
}
