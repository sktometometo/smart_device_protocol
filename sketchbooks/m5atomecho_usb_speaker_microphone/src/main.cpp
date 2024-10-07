/*Press button to record,released button to playback*/

#include <M5Atom.h>
#include <driver/i2s.h>

#define CONFIG_I2S_BCK_PIN 19
#define CONFIG_I2S_LRCK_PIN 33
#define CONFIG_I2S_DATA_PIN 22
#define CONFIG_I2S_DATA_IN_PIN 23

#define SPEAKER_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1
#define DATA_SIZE 1024

uint8_t microphonedata0[1024 * 80];
int data_offset = 0;
bool installed = false;

bool InitI2SSpeakerOrMic(int mode) {
  esp_err_t ret = ESP_OK;

  if (installed) {
    ret = i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
    if (ret != ESP_OK) {
      Serial.printf("i2s_driver_uninstall failed: %d\n", ret);
      return false;
    }
  }
  installed = true;
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
      .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 6,
      .dma_buf_len = 60,
  };
  if (mode == MODE_MIC) {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  } else {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = true;
  }

  ret = i2s_driver_install(SPEAKER_I2S_NUMBER, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", ret);
    return false;
  }

  i2s_pin_config_t tx_pin_config;
  tx_pin_config.mck_io_num = GPIO_NUM_0;
  tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
  tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
  tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
  tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;

  Serial.println("Init i2s_set_pin");
  ret = i2s_set_pin(SPEAKER_I2S_NUMBER, &tx_pin_config);
  if (ret != ESP_OK) {
    Serial.printf("i2s_set_pin failed: %d\n", ret);
    return false;
  }
  Serial.println("Init i2s_set_clk");
  ret = i2s_set_clk(SPEAKER_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (ret != ESP_OK) {
    Serial.printf("i2s_set_clk failed: %d\n", ret);
    return false;
  }
  Serial.println("Finished InitI2SSpeakerOrMic");
}

void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  M5.dis.drawpix(0, CRGB(128, 128, 0));
  delay(2000);
  Serial.println("Press button to record,released button to playback");
}

void loop() {
  if (M5.Btn.isPressed()) {
    if (not InitI2SSpeakerOrMic(MODE_MIC)) {
      Serial.println("InitI2SSpeakerOrMic failed");
      return;
    }
    data_offset = 0;
    size_t byte_read;
    size_t bytes_written;
    Serial.println("Recording...");
    while (1) {
      auto ret = i2s_read(SPEAKER_I2S_NUMBER, (char *)(microphonedata0 + data_offset), DATA_SIZE, &byte_read, (100 / portTICK_RATE_MS));
      if (ret != ESP_OK) {
        Serial.printf("i2s_read failed: %d\n", ret);
      }
      data_offset += byte_read;
      if (data_offset + DATA_SIZE >= sizeof(microphonedata0)) {
        break;
      }
      M5.update();
      if (M5.Btn.isReleased())
        break;
      // delay(60);
    }
    Serial.println("Recording done");
    if (not InitI2SSpeakerOrMic(MODE_SPK)) {
      Serial.println("InitI2SSpeakerOrMic failed");
      return;
    }
    Serial.println("Playing...");
    M5.dis.drawpix(0, CRGB(0, 128, 128));
    i2s_write(SPEAKER_I2S_NUMBER, microphonedata0, data_offset, &bytes_written, portMAX_DELAY);
    Serial.println("Playing done");
    M5.dis.drawpix(0, CRGB(128, 128, 0));
  }
  M5.update();
}