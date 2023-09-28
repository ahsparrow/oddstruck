#include <M5StickC.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <MQTT.h>

// GPIO pin for 2SS52M magnetoresistive sensor
#define MR_SENSOR_PIN (26)

// GPIO pins for microphone
#define MIC_CLK_PIN (0)
#define MIC_DATA_PIN (34)

// Microphone parameters
#define SAMPLE_RATE (44100)
#define DMA_LENGTH (128)
#define MIC_THRESHOLD (10000)

// Wifi settings
#define SSID "cumulus"
#define PASSWORD "lkjfds11"

WiFiClient net;
MQTTClient mqtt;

// Trigger variables
int triggered = 0;
unsigned long trigger_time;

// Set up I2S for PDM microphone data
void i2sSetup() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = DMA_LENGTH,
  };

  i2s_pin_config_t pin_config;

  pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
  pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num = MIC_CLK_PIN;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = MIC_DATA_PIN;

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

// Connect to Wifi
void wifiSetup() {
  WiFi.begin(SSID, PASSWORD);
  M5.lcd.printf("Connecting WiFi: %s...", SSID);

  for (int i = 0; i < 8; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      M5.lcd.print("\nIP address: ");
      M5.lcd.println(WiFi.localIP());
      break;
    }
    else {
      M5.lcd.print(".");
      delay(500);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    M5.lcd.println("\nFail");
  }

  delay(500);
}

// Connect to MQTT broker
void mqttSetup() {
  M5.lcd.print("Connecting MQTT...");
  mqtt.begin("192.168.1.100", net);

  while (!mqtt.connect("oddstruct", "", "")) {
    M5.lcd.print(".");
    delay(500);
  }
  M5.lcd.println("\nOK");

  delay(500);
}

// Sound detection task
void mic_task(void *args) {
  char buf[DMA_LENGTH * 4] = {0};
  size_t bytesread;
  int16_t *adc;

  unsigned long strike_time;
  long delay;
  long old_delay = 0;
  char str[100];


  while (1) {
    i2s_read(I2S_NUM_0, buf, DMA_LENGTH * 2, &bytesread, portMAX_DELAY);
    adc = (int16_t *)buf;

    if (triggered) {
      for (int i = 0; i < DMA_LENGTH; i++) {
        if (abs(adc[i]) > MIC_THRESHOLD) {
          strike_time = micros() - ((DMA_LENGTH - i) * 1000000L / SAMPLE_RATE);
          delay = (strike_time - trigger_time) / 1000;

          sprintf(str, "%d", delay);
          M5.Lcd.setTextDatum(MC_DATUM);
          M5.Lcd.setTextSize(4);
          M5.Lcd.setTextColor(CYAN);
          M5.Lcd.drawString(str, 80, 40, 2);

          if (mqtt.connected()) {
            sprintf(str, "{\"delay\": %d, \"delta\": %d}", delay, delay - old_delay);
            mqtt.publish("/oddstruck", str);

            old_delay = delay;
          }

          triggered = 0;
          break;
        }
      }
    }
  }
}

// Magnetic sensor task
void sensor_task(void *args) {
   unsigned long start, stop;
   int timeout;

  while (1) {
    // Wait for sensor to trigger
    while (digitalRead(MR_SENSOR_PIN) == 1) {
      vTaskDelay(portTICK_PERIOD_MS);
    }
    start = micros();
    M5.Lcd.fillScreen(RED);

    // Wait for end of trigger (10ms timeout)
    timeout = 0;
    while (timeout < 10) {
      if (digitalRead(MR_SENSOR_PIN) == 0) {
        stop = micros();
        timeout = 0;
      } else {
        timeout += 1;
      }

      vTaskDelay(portTICK_PERIOD_MS);
    }

    trigger_time = (start + stop) / 2;
    triggered = 1;
    M5.Lcd.fillScreen(BLACK);

    // Wait at least a second before triggering again
    vTaskDelay(1000 * portTICK_PERIOD_MS);
  }
}

void setup() {
  M5.begin();

  // Display
  M5.Lcd.setRotation(3);

  // Bell position sensor
  pinMode(MR_SENSOR_PIN, INPUT);

  wifiSetup();
  if (WiFi.status() == WL_CONNECTED) {
    mqttSetup();
  }

  i2sSetup();

  xTaskCreate(mic_task, "mic_task", 2048, NULL, 1, NULL);
  xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 1, NULL);
}

void loop() {
  mqtt.loop();

  vTaskDelay(10 * portTICK_PERIOD_MS);
}