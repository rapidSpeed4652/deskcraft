#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <ESP_I2S.h>
#include "esp_camera.h"
#include <String.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_MOSI D10
#define OLED_CLK D8
#define OLED_RESET D1
#define OLED_DC D2
#define OLED_CS D3

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS)
RoboEyes<Adafruit_SSD1306> roboEyes(display)

#define MIC_CLK_PIN 42
#define MIC_DATA_PIN 41
I2SClass I2S;

#define SPK_DIN_PIN  D5  // GPIO 6
#define SPK_BCLK_PIN D6  // GPIO 43
#define SPK_LRC_PIN  D7  // GPIO 44
I2SClass I2SSpeaker;

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

const size_t AUDIO_CHUNK_SAMPLES = 512;
int16_t audioBuffer[AUDIO_CHUNK_SAMPLES];

bool streamAudioToPC = true;
enum RxState { WAIT_TAG, WAIT_LEN, WAIT_PAYLOAD };
RxState rxState = WAIT_TAG;

char rxTag[4] = {0};
uint8_t rxTagIdx = 0;
uint32_t rxPayloadLen = 0;
uint8_t rxLenBytes[4];
uint8_t rxLenIdx = 0;
uint32_t rxBytesRead = 0;
String cmdAccumulator = "";

void sendPacket(const char* tag, const uint8_t* data, uint32_t len) {
  Serial.write((const uint8_t*)tag, 3);
  Serial.write((const uint8_t*)&len, 4);
  Serial.write(data, len);
}

void captureAndSendImage() {
  // Flush 1 stale buffered frame so the snapshot reflects what's in front of it right now
  camera_fb_t *stale = esp_camera_fb_get();
  if (stale) esp_camera_fb_return(stale);

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  sendPacket("IMG", fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handlePCCommand(String cmd) {
  if (cmd == "SNAP") {
    captureAndSendImage();
  } 
  else if (cmd == "STATE:LISTENING") {
    streamAudioToPC = true;          // Ensure mic stream is active
    roboEyes.setIdleMode(OFF, 0, 0);
    roboEyes.setPosition(DEFAULT);   // Locks gaze straight onto you
    roboEyes.setMood(DEFAULT);
    roboEyes.setCuriosity(ON);
  } 
  else if (cmd == "STATE:THINKING") {
    streamAudioToPC = false;         // Pause mic streaming while VLM processes
    roboEyes.setPosition(NE);        // Looks up-right while thinking
    roboEyes.setHFlicker(ON, 1);     // Subtle processing micro-jitter
  } 
  else if (cmd == "STATE:SPEAKING") {
    streamAudioToPC = false;         // Mute mic so speaker TTS doesn't loop back into Whisper
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.setPosition(DEFAULT);
    roboEyes.setMood(HAPPY);
    roboEyes.anim_laugh();           // Bouncy acknowledgement as speech starts
  } 
  else if (cmd == "STATE:CONFUSED") {
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.anim_confused();
  } 
  else if (cmd == "STATE:IDLE") {
    streamAudioToPC = true;          // Re-enable mic to listen for your next question
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.setMood(DEFAULT);
    roboEyes.setIdleMode(ON, 2, 2);  // Resume casual autonomous looking around
  }
}
void processIncomingSerial() {
  while (Serial.available()) {
    if (rxState == WAIT_TAG) {
      rxTag[rxTagIdx++] = Serial.read();
      if (rxTagIdx == 3) {
        rxTag[3] = '\0';
        rxTagIdx = 0;
        rxLenIdx = 0;
        rxState = WAIT_LEN;
      }
    } 
    else if (rxState == WAIT_LEN) {
      rxLenBytes[rxLenIdx++] = Serial.read();
      if (rxLenIdx == 4) {
        memcpy(&rxPayloadLen, rxLenBytes, 4); // Unpack 32-bit little-endian length
        rxBytesRead = 0;
        cmdAccumulator = "";
        rxState = (rxPayloadLen > 0) ? WAIT_PAYLOAD : WAIT_TAG;
      }
    } 
    else if (rxState == WAIT_PAYLOAD) {
      uint32_t remaining = rxPayloadLen - rxBytesRead;
      size_t avail = Serial.available();
      size_t toRead = min((size_t)remaining, min(avail, SPK_BUF_BYTES));

      size_t actual = Serial.readBytes((char*)spkBuffer, toRead);
      rxBytesRead += actual;

      if (strcmp(rxTag, "SPK") == 0) {
        // Push raw 16kHz 16-bit PCM directly to the MAX98357A I2S DMA buffer
        I2SSpeaker.write(spkBuffer, actual);
      } 
      else if (strcmp(rxTag, "CMD") == 0) {
        for (size_t i = 0; i < actual; i++) {
          char c = (char)spkBuffer[i];
          if (c != '\n' && c != '\r') cmdAccumulator += c;
        }
      }

      if (rxBytesRead >= rxPayloadLen) {
        if (strcmp(rxTag, "CMD") == 0) {
          handlePCCommand(cmdAccumulator);
        }
        rxState = WAIT_TAG;
      }
    }
  }
}

void setup() {
  Serial.begin(921600);

  SPI.begin(OLED_CLK, -1, OLED_MOSI, OLED_CS);
  if(!display.begin(SSD1306_SWITCHCAPVCC))
  {
    Serial.println("SSD1306 SPI allocation failed!");
    for (;;);
  }

  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 60);
  roboEyes.setWidth(36, 36);
  roboEyes.setHeight(36, 36);
  roboEyes.setBorderradius(8, 8);
  roboEyes.setSpacebetween(10);
  roboEyes.setAutoBlinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setCuriosity(ON);

  I2S.setPinsPdmRx(MIC_CLK_PIN, MIC_DATA_PIN);
  I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  I2SSpeaker.setPins(SPK_BCLK_PIN, SPK_LRC_PIN, SPK_DIN_PIN);
  if (!I2SSpeaker.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("MAX98357A Speaker init failed");
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;   config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;   config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;   config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;     // 640x480 JPEG
  config.jpeg_quality = 12;              // 0-63 (lower = sharper image)
  config.fb_count = 2;                  
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_camera_init(&config);
}

void loop() {
  roboEyes.update();

  processIncomingSerial();

  while(Serial.available())
  {
    char c = Serial.read();
    if(c == "\n")
    {
      serialCmd.trim();
      handlePCCommand(serialCmd);
      serialCmd = "";
    }
    else
    {
      serialCmd += c;
    }
  }

  if(streamAudioToPC && I2S.available() >= (AUDIO_CHUNK_SAMPLES * 2))
  {
    size_t bytesRead = I2S.readBytes((char*)audioBuffer, sizeof(audioBuffer));
    if(bytesRead > 0)
    {
      sendPacket("AUD", (u8int_t*)audioBuffer, bytesRead);
    }
  }
}
