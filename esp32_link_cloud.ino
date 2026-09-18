/*
 * esp32_link.ino - ESP32 (ESP-WROOM-32) phrase display node + Blynk cloud
 *
 * UART2 : RX=GPIO17 <- STM32 PC10, TX=GPIO22 -> STM32 PC11, 115200 8N1
 * ILI9341 (VSPI): SCK=18, MOSI=23, MISO=19, CS=5, DC=2, RST=4
 *
 * Frames (\n terminated):  P,NN -> show phrase NN   C -> clear
 * Replies: A,P,NN / A,C ack   E,1 bad id   E,2 bad frame   R boot
 *
 * Cloud (Blynk, core 0 task, non-blocking for local path):
 *   V0 = last phrase (UTF-8 Bangla), V1 = phrase id (0 = cleared)
 *   events: help_request (critical, push), phrase_sent (info)
 *
 * Libraries: Adafruit ILI9341, Adafruit GFX, Blynk (Board: ESP32 Dev Module)
 */

/* ---- Blynk credentials: paste from Blynk Console (must precede includes) ---- */
#define BLYNK_TEMPLATE_ID   "TMPL6zXjUvZad"
#define BLYNK_TEMPLATE_NAME "AAC Node"
#define BLYNK_AUTH_TOKEN    "OYs9NLtlcjpXefOCphwAQg3j-A3eU2zz"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "phrases.h"

/* ---- WiFi (2.4 GHz, WPA2-Personal, e.g. phone hotspot) ---- */
static const char WIFI_SSID[] = "aac";
static const char WIFI_PASS[] = "12345678";

#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define LINK_RX  17
#define LINK_TX  22
#define LINE_MAX 32
#define HELP_ID  3          /* phrase that raises a critical alert */
#define CLOUD_Q_LEN 8

/* UTF-8 text for cloud side; index matches phrase_table */
static const char *phrase_text[PHRASE_COUNT + 1] = {
  "",
  "আমি ঠিক আছি",
  "হ্যাঁ",
  "আমাকে সাহায্য করুন",
};

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

static uint16_t palette[16];
static uint16_t lineBuf[320];
static char     acc[LINE_MAX];
static uint8_t  accLen = 0;
static QueueHandle_t cloudQ;

/* ========================= Display ========================= */
static void buildPalette(void)
{
  for (int n = 0; n < 16; n++) {
    uint8_t v = n * 17;               /* 0 -> black bg, 15 -> white ink */
    palette[n] = ((v >> 3) << 11) | ((v >> 2) << 5) | (v >> 3);
  }
}

static void drawPhrase(uint8_t id)
{
  const PhraseBmp &p = phrase_table[id];
  int16_t x0 = (tft.width()  - p.w) / 2;
  int16_t y0 = (tft.height() - p.h) / 2;
  uint16_t bytesPerRow = (p.w + 1) / 2;

  tft.fillScreen(ILI9341_BLACK);
  for (uint16_t y = 0; y < p.h; y++) {
    const uint8_t *row = p.data + (uint32_t)y * bytesPerRow;
    for (uint16_t x = 0; x < p.w; x++) {
      uint8_t b = row[x >> 1];
      uint8_t n = (x & 1) ? (b & 0x0F) : (b >> 4);
      lineBuf[x] = palette[n];
    }
    tft.drawRGBBitmap(x0, y0 + y, lineBuf, p.w, 1);
  }
}

/* ========================= Cloud (core 0) ========================= */
static void publish(uint8_t id)
{
  if (id == 0) {
    Blynk.virtualWrite(V0, "-");
    Blynk.virtualWrite(V1, 0);
    return;
  }
  Blynk.virtualWrite(V0, phrase_text[id]);
  Blynk.virtualWrite(V1, id);
  Blynk.logEvent(id == HELP_ID ? "help_request" : "phrase_sent", phrase_text[id]);
}

static void cloudTask(void *arg)
{
  (void)arg;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);            /* clear any stale saved credentials */
  delay(200);

  Serial.println("WiFi: scanning...");
  int n = WiFi.scanNetworks();
  Serial.printf("WiFi: %d networks found\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("  '%s'  rssi=%d  ch=%d  enc=%d\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                  WiFi.channel(i), WiFi.encryptionType(i));
  }

  WiFi.setAutoReconnect(true);
  Serial.printf("WiFi: connecting to '%s'\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Blynk.config(BLYNK_AUTH_TOKEN);

  uint8_t id;
  uint32_t lastLog = 0;
  bool announced = false;

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!announced) {
        Serial.print("WiFi: connected, IP = ");
        Serial.println(WiFi.localIP());
        announced = true;
      }
      Blynk.run();
      if (Blynk.connected()) {
        while (xQueueReceive(cloudQ, &id, 0) == pdTRUE) publish(id);
      }
    } else {
      announced = false;
      if (millis() - lastLog > 2000) {
        lastLog = millis();
        Serial.printf("WiFi: status=%d\n", WiFi.status());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void cloudNotify(uint8_t id)
{
  xQueueSend(cloudQ, &id, 0);            /* never blocks local path; drops if full */
}

/* ========================= UART link ========================= */
static void reply(const char *s)
{
  Serial2.print(s);
  Serial2.print('\n');
  Serial.print("TX: ");
  Serial.println(s);
}

static void handleLine(const char *line)
{
  Serial.print("RX: ");
  Serial.println(line);

  if (strcmp(line, "C") == 0) {
    tft.fillScreen(ILI9341_BLACK);
    reply("A,C");
    cloudNotify(0);
    return;
  }

  if (line[0] == 'P' && line[1] == ',') {
    char *end;
    long id = strtol(line + 2, &end, 10);
    if (end != line + 2 && *end == '\0' && id >= 1 && id <= PHRASE_COUNT) {
      drawPhrase((uint8_t)id);
      char ack[16];
      snprintf(ack, sizeof(ack), "A,P,%02ld", id);
      reply(ack);
      cloudNotify((uint8_t)id);          /* after ACK: cloud never delays STM32 */
    } else {
      reply("E,1");
    }
    return;
  }

  reply("E,2");
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);

  buildPalette();
  SPI.begin(18, 19, 23, TFT_CS);
  tft.begin(1000000);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("Link ready");

  cloudQ = xQueueCreate(CLOUD_Q_LEN, sizeof(uint8_t));
  xTaskCreatePinnedToCore(cloudTask, "cloud", 8192, NULL, 1, NULL, 0);

  Serial.println("ESP32 phrase node ready");
  reply("R");
}

void loop()
{
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r' || c == '\n') {
      if (accLen > 0) {
        acc[accLen] = '\0';
        handleLine(acc);
        accLen = 0;
      }
    } else if (accLen < LINE_MAX - 1) {
      acc[accLen++] = c;
          } else {
      accLen = 0;
    }
  }
}