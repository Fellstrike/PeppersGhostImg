/******************************************************************************************************
This sketch runs fine to have two images of the exact same type. 
Currently have one CS hooked up to 21 and the other CS hooked up to 22
This was done w/o messing with the CS sequences aT ALL. Will try to see if I can change something.

Doesn't seem to work with other pins. Though making two tft objects sort of works.
*******************************************************************************************************/

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <SPI.h>              //SPI Library
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "capstoneText.c"

// === WiFi + OTA Setup ===
const char* ssid = "**********";
const char* password = "**********";
WebServer server(80);
WiFiUDP Udp;
const int localPort = 8000; // OSC port

//Define the currently used pins
#define TFT_CS 19
#define TFT_CS2 20
#define TFT_RST 7
#define TFT_DC 6

#define IMG_WIDTH 128
#define IMG_HEIGHT 128

Adafruit_ST7735 tft = Adafruit_ST7735(-1, TFT_DC, TFT_RST);

float p = 3.1415926;

//Fire Stuff
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

#define FIREBALL_RADIUS 50
#define FIREBALL_CENTER_X (SCREEN_WIDTH / 2)
#define FIREBALL_CENTER_Y (SCREEN_HEIGHT / 2)

const uint16_t fireColors[] = {
  0x2000, // dark red
  0x4000, // red
  0xF800, // bright red
  0xFC00, // orange
  0xFFE0, // yellow
  0xFFFF  // white
};

const uint16_t elecColors[] = {
  0x032c, // dark teal
  0x5ddf, // blue
  0xFFE3, // yellow
  0x967f, //light blue
  0xff93, // light yellow
  0xFFFF  // white
};

const int numColors = sizeof(fireColors) / 2;

unsigned long lastFrame = 0;
float pulsePhase = 0.0;

#define MAX_SPARKS 20

struct Spark {
  int x, y;
  int life;
  uint16_t color;
};

Spark sparks[MAX_SPARKS];

GFXcanvas16 canvas1(128, 128);
GFXcanvas16 canvas2(SCREEN_WIDTH, SCREEN_HEIGHT);

void setup() {  
  Serial.begin(115200);

  const char* apSSID = "ESP32-Fireball";
  const char* apPassword = "openfire";  // optional; use NULL for open network

  WiFi.softAP(apSSID, apPassword);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Soft AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP SSID: ");
  Serial.println(WiFi.SSID());

  Udp.begin(localPort);
  server.on("/", []() {
    server.send(200, "text/html", "<h1>ESP32 Eye Online</h1><a href=\"/update\">Update Firmware</a>");
  });
  ElegantOTA.begin(&server);  // OTA Updating
  server.begin();

  tft.initR(INITR_144GREENTAB);
  tft.setSPISpeed(24000000);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_CS2, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_CS2, HIGH);

  // Optional: clear both screens
  canvas1.fillScreen(ST77XX_BLACK);
  canvas2.fillScreen(ST77XX_BLACK);
  drawToDisplay(canvas1, TFT_CS);
  drawToDisplay(canvas2, TFT_CS2);
}

void loop() {
  server.handleClient();  // Handles OTA updates
  ElegantOTA.loop();
  handleOSC();
  unsigned long t = millis() % 7000;

  if (t <= 6000) {
    drawFireballTo(canvas1, fireColors, sizeof(fireColors) / 2, false);
    drawFireballTo(canvas2, elecColors, sizeof(elecColors) / 2, true); // lightning style
  } else {
    drawRGB565Image(eyeImg, canvas1, IMG_WIDTH, IMG_HEIGHT);
    drawRGB565Image(capstoneTextLogo, canvas2, IMG_WIDTH, IMG_HEIGHT);
  }
  drawToDisplay(canvas1, TFT_CS);
  delay(10);
  drawToDisplay(canvas2, TFT_CS2);
  delay(10);
}

uint16_t getWeightedFireColor(float normDist, int colorCount) {
  float bias = pow(normDist, 2.5);  // stronger fade
  int colorCap = colorCount - 2;     // limit white usage
  int baseIndex = floor((1.0 - bias) * colorCap);

  int offset = random(-1, 2);
  int index = constrain(baseIndex + offset, 0, colorCap);
  return index;
}

void updateSparks(GFXcanvas16 &canvas) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    if (sparks[i].life > 0) {
      sparks[i].y -= 1;
      sparks[i].life--;

      if (sparks[i].life > 0) {
        canvas.drawPixel(sparks[i].x, sparks[i].y, sparks[i].color);
      }
    } else if (random(100) < 3) {  // spawn chance
      sparks[i].x = FIREBALL_CENTER_X + random(-FIREBALL_RADIUS / 2, FIREBALL_RADIUS / 2);
      sparks[i].y = FIREBALL_CENTER_Y - FIREBALL_RADIUS / 2;
      sparks[i].life = random(10, 20);
      sparks[i].color = fireColors[random(2, 5)];
    }
  }
}

void drawFireballTo(GFXcanvas16 &canvas, const uint16_t *palette, int numColors, bool lightning) {
  canvas.fillScreen(ST77XX_BLACK);
  float pulse = 1.0 + 0.07 * sin(millis() * 0.004);

  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      float dx = (x - SCREEN_WIDTH / 2) / (50.0 * pulse);
      float dy = (y - SCREEN_HEIGHT / 2) / (50.0 * 1.3);
      float dist = sqrt(dx * dx + dy * dy);

      if (dist <= 1.0) {
        float bias = pow(dist, 2.5);
        int baseIndex = floor((1.0 - bias) * (numColors - 2));
        int offset = random(-1, 2);
        int index = constrain(baseIndex + offset, 0, numColors - 2);
        canvas.drawPixel(x, y, palette[index]);
      } else if (dist <= 1.3) {
        float glow = 1.3 - dist;
        uint8_t g = glow * 60;
        uint16_t glowColor = ((g >> 3) << 11) | ((g >> 2) << 5) | (g >> 3);
        canvas.drawPixel(x, y, glowColor);
      }
    }
  }

  if (lightning) drawLightningArcTo(canvas);
  updateSparks(canvas);
}

void drawLightningArcTo(GFXcanvas16 &canvas) {
  for (int a = 0; a < random(1, 3); a++) {
    float angle1 = random(0, 360) * PI / 180.0;
    float angle2 = angle1 + random(45, 135) * PI / 180.0;

    float r = 45;
    int x1 = SCREEN_WIDTH / 2 + r * cos(angle1);
    int y1 = SCREEN_HEIGHT / 2 + r * sin(angle1) * 1.3;
    int x2 = SCREEN_WIDTH / 2 + r * cos(angle2);
    int y2 = SCREEN_HEIGHT / 2 + r * sin(angle2) * 1.3;

    int segments = 6;
    int prevX = x1, prevY = y1;

    for (int i = 1; i <= segments; i++) {
      float t = (float)i / segments;
      int cx = x1 + (x2 - x1) * t + random(-3, 4);
      int cy = y1 + (y2 - y1) * t + random(-3, 4);
      uint16_t arcColor = elecColors[random(1, 5)];
      canvas.drawLine(prevX, prevY, cx, cy, arcColor);
      prevX = cx;
      prevY = cy;
    }
  }
}

// Function to draw 1-bit bitmap
void drawRGB565Image(const uint16_t *img, GFXcanvas16 &canvas, int w, int h) {
  canvas.fillScreen(ST77XX_BLACK);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint16_t color = pgm_read_word(&img[y * w + x]);  // image is 1D array
      canvas.drawPixel(x, y, color);
    }
  }
}
void drawToDisplay(GFXcanvas16 &canvas, int csPin) {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_CS2, HIGH);
  digitalWrite(csPin, LOW);

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);

  digitalWrite(csPin, HIGH);
}

// === OSC Input ===
void handleOSC() {
  int size;
  if ((size = Udp.parsePacket()) > 0) {
    uint8_t buffer[255];
    size = Udp.read(buffer, 255);

    OSCMessage msg;
    msg.fill(buffer, size); 

    if (!msg.hasError()) {
      //code to switch images here
      }
    else {
      OSCErrorCode error = msg.getError();
      Serial.print("OSC Error: ");
      Serial.println(error);
    }
  }
}
