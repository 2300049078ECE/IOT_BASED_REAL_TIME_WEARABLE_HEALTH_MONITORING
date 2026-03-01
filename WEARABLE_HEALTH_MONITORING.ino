/*
 * ============================================================
 *  WEARABLE HEALTH MONITORING SYSTEM  (v2 — with OLED)
 *  Sensors : MAX30102 (SpO2/HR) | MLX90614 (Temp) | MPU6050 (IMU)
 *  Display : SSD1306 OLED 128×64 (I²C, addr 0x3C)
 *  Cloud   : ThingSpeak IoT
 *  Board   : ESP32
 * ============================================================
 *
 *  OLED Screens (auto-rotate every 3 s):
 *    Screen 0 → Heart Rate + SpO₂
 *    Screen 1 → Body Temp + Ambient Temp
 *    Screen 2 → Accelerometer X/Y/Z
 *    Screen 3 → Fall status + WiFi status
 *
 *  ThingSpeak Channel Fields:
 *    Field 1 → Heart Rate (BPM)
 *    Field 2 → SpO2 (%)
 *    Field 3 → Body Temperature (°C)
 *    Field 4 → Ambient Temperature (°C)
 *    Field 5 → Accel X (m/s²)
 *    Field 6 → Accel Y (m/s²)
 *    Field 7 → Accel Z (m/s²)
 *    Field 8 → Fall Detection (0/1)
 *
 *  Libraries required (install via Library Manager):
 *    - SparkFun MAX3010x Pulse and Proximity Sensor Library
 *    - Adafruit MLX90614 Library
 *    - Adafruit MPU6050 (+ Adafruit Unified Sensor)
 *    - Adafruit SSD1306
 *    - Adafruit GFX Library
 *    - ThingSpeak by MathWorks
 *    - WiFi (built-in ESP32)
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include "ThingSpeak.h"

// MAX30102
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// MLX90614
#include <Adafruit_MLX90614.h>

// MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// OLED SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─────────────────────────────────────────────
//  CONFIGURATION — Edit these values
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "Testwifi";
const char* WIFI_PASSWORD = "12345678";

unsigned long THINGSPEAK_CHANNEL_ID = 2703273;  // your channel ID
const char*   THINGSPEAK_WRITE_KEY  = "I3D6SUTJI5ZP0ALX";

// Upload interval (ThingSpeak free tier min = 15 s)
const unsigned long UPLOAD_INTERVAL_MS = 20000;

// OLED screen rotation interval (ms)
const unsigned long SCREEN_INTERVAL_MS = 3000;

// Fall detection thresholds (m/s²)
const float FALL_THRESHOLD = 20.0;
const float REST_THRESHOLD = 3.0;

// ─────────────────────────────────────────────
//  OLED Setup  (128×64, I²C address 0x3C)
// ─────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1       // no reset pin; share ESP32 reset
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─────────────────────────────────────────────
//  Objects
// ─────────────────────────────────────────────
WiFiClient        wifiClient;
MAX30105          particleSensor;
Adafruit_MLX90614 mlx;
Adafruit_MPU6050  mpu;

// ─────────────────────────────────────────────
//  MAX30102 SpO2 buffers
// ─────────────────────────────────────────────
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t  spo2Value;
int8_t   spo2Valid;
int32_t  heartRate;
int8_t   hrValid;

// ─────────────────────────────────────────────
//  Heart-rate smoothing
// ─────────────────────────────────────────────
const byte RATE_ARRAY_SIZE = 4;
byte  rates[RATE_ARRAY_SIZE];
byte  rateSpot = 0;
long  lastBeat = 0;
float beatsPerMinute;
int   beatAvg;

// ─────────────────────────────────────────────
//  Sensor readings (globals for OLED access)
// ─────────────────────────────────────────────
float bodyTemp    = 0.0;
float ambientTemp = 0.0;
float ax = 0.0, ay = 0.0, az = 0.0;
float aMag = 0.0;

// ─────────────────────────────────────────────
//  Fall detection state
// ─────────────────────────────────────────────
bool fallDetected = false;
unsigned long fallTime = 0;
const unsigned long FALL_RESET_MS = 5000;

// ─────────────────────────────────────────────
//  Timers
// ─────────────────────────────────────────────
unsigned long lastUpload = 0;
unsigned long lastScreen = 0;
byte currentScreen = 0;
#define TOTAL_SCREENS 4

// ═══════════════════════════════════════════════════════════
//  OLED HELPER — draw a thin header bar
// ═══════════════════════════════════════════════════════════
void drawHeader(const char* title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);
}

// ═══════════════════════════════════════════════════════════
//  OLED HELPER — draw page dots at bottom
// ═══════════════════════════════════════════════════════════
void drawPageDots() {
  int spacing = 10;
  int startX  = (SCREEN_WIDTH - (TOTAL_SCREENS * spacing)) / 2;
  for (byte i = 0; i < TOTAL_SCREENS; i++) {
    int x = startX + i * spacing + 3;
    if (i == currentScreen)
      display.fillCircle(x, 61, 2, SSD1306_WHITE);
    else
      display.drawCircle(x, 61, 2, SSD1306_WHITE);
  }
}

// ═══════════════════════════════════════════════════════════
//  OLED SCREEN 0 — Heart Rate + SpO₂
// ═══════════════════════════════════════════════════════════
void screenHeartRate() {
  int hrShow  = hrValid   ? (int)heartRate : beatAvg;
  int spo2Show = spo2Valid ? (int)spo2Value : 0;

  display.clearDisplay();
  drawHeader("  HEART & OXYGEN");

  // Heart Rate
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("HR:");
  display.setTextSize(2);
  display.setCursor(28, 12);
  display.print(hrShow);
  display.setTextSize(1);
  display.setCursor(68, 18);
  display.print("BPM");

  // Divider
  display.drawLine(0, 34, SCREEN_WIDTH - 1, 34, SSD1306_WHITE);

  // SpO2
  display.setTextSize(1);
  display.setCursor(0, 37);
  display.print("SpO2:");
  display.setTextSize(2);
  display.setCursor(40, 36);
  display.print(spo2Show);
  display.setTextSize(1);
  display.setCursor(80, 42);
  display.print("%");

  // Warning if SpO2 low
  if (spo2Show > 0 && spo2Show < 95) {
    display.setCursor(95, 37);
    display.print("LOW!");
  }

  drawPageDots();
  display.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED SCREEN 1 — Temperature
// ═══════════════════════════════════════════════════════════
void screenTemperature() {
  display.clearDisplay();
  drawHeader("  TEMPERATURE");

  // Body temp
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("Body:");
  display.setTextSize(2);
  display.setCursor(38, 12);
  display.print(bodyTemp, 1);
  display.setTextSize(1);
  display.setCursor(100, 18);
  display.print("\xF8""C");   // °C

  // Fever indicator
  if (bodyTemp > 38.0) {
    display.setCursor(0, 27);
    display.print(">> FEVER ALERT <<");
  }

  display.drawLine(0, 34, SCREEN_WIDTH - 1, 34, SSD1306_WHITE);

  // Ambient temp
  display.setTextSize(1);
  display.setCursor(0, 37);
  display.print("Ambient:");
  display.setTextSize(2);
  display.setCursor(60, 36);
  display.print(ambientTemp, 1);
  display.setTextSize(1);
  display.setCursor(110, 42);
  display.print("\xF8""C");

  drawPageDots();
  display.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED SCREEN 2 — Accelerometer
// ═══════════════════════════════════════════════════════════
void screenAccel() {
  display.clearDisplay();
  drawHeader("  ACCELEROMETER");

  display.setTextSize(1);

  display.setCursor(0, 14);
  display.print("X:");
  display.setCursor(16, 14);
  display.print(ax, 2);
  display.setCursor(80, 14);
  display.print("m/s2");

  display.setCursor(0, 26);
  display.print("Y:");
  display.setCursor(16, 26);
  display.print(ay, 2);
  display.setCursor(80, 26);
  display.print("m/s2");

  display.setCursor(0, 38);
  display.print("Z:");
  display.setCursor(16, 38);
  display.print(az, 2);
  display.setCursor(80, 38);
  display.print("m/s2");

  display.drawLine(0, 50, SCREEN_WIDTH - 1, 50, SSD1306_WHITE);
  display.setCursor(0, 53);
  display.print("|Mag|: ");
  display.print(aMag, 2);

  drawPageDots();
  display.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED SCREEN 3 — Status (Fall + WiFi + Upload)
// ═══════════════════════════════════════════════════════════
void screenStatus() {
  display.clearDisplay();
  drawHeader("  SYSTEM STATUS");

  display.setTextSize(1);

  // Fall status
  display.setCursor(0, 14);
  display.print("Fall: ");
  if (fallDetected) {
    display.setTextSize(2);
    display.setCursor(40, 12);
    display.print("! FALL !");
    display.setTextSize(1);
  } else {
    display.print("Normal");
  }

  // WiFi status
  display.setCursor(0, 30);
  display.print("WiFi: ");
  display.print(WiFi.status() == WL_CONNECTED ? "Connected" : "OFFLINE");

  // Upload countdown
  unsigned long nextUpload = UPLOAD_INTERVAL_MS - (millis() - lastUpload);
  if (lastUpload == 0) nextUpload = 0;
  display.setCursor(0, 42);
  display.print("Upload in: ");
  display.print(nextUpload / 1000);
  display.print("s");

  // Signal strength
  display.setCursor(0, 54);
  display.print("RSSI: ");
  display.print(WiFi.RSSI());
  display.print(" dBm");

  drawPageDots();
  display.display();
}

// ═══════════════════════════════════════════════════════════
//  OLED SPLASH SCREEN
// ═══════════════════════════════════════════════════════════
void splashScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 5);
  display.println("Health Monitor v2");
  display.drawLine(0, 16, SCREEN_WIDTH - 1, 16, SSD1306_WHITE);
  display.setCursor(20, 22);
  display.println("Initializing...");
  display.setCursor(0, 38);
  display.println(" MAX30102  MLX90614");
  display.setCursor(0, 50);
  display.println(" MPU6050   ESP32");
  display.display();
  delay(2000);
}

// ═══════════════════════════════════════════════════════════
//  OLED STATUS LINE (during init)
// ═══════════════════════════════════════════════════════════
void oledStatus(const char* msg) {
  display.fillRect(0, 38, SCREEN_WIDTH, 26, SSD1306_BLACK);
  display.setCursor(0, 42);
  display.print(msg);
  display.display();
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22

  Serial.println("\n=== Wearable Health Monitor v2 ===");

  // ── OLED ──────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("ERROR: SSD1306 OLED not found.");
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  splashScreen();
  Serial.println("SSD1306 OLED OK");

  // ── WiFi ──────────────────────────────────
  oledStatus("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500); Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    oledStatus("WiFi OK!");
  } else {
    Serial.println("\nWiFi FAILED — continuing offline");
    oledStatus("WiFi FAILED");
  }
  ThingSpeak.begin(wifiClient);
  delay(800);

  // ── MAX30102 ───────────────────────────────
  oledStatus("Init MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102 not found.");
    oledStatus("MAX30102 ERROR!");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  Serial.println("MAX30102 OK");
  oledStatus("MAX30102 OK");
  delay(500);

  // ── MLX90614 ──────────────────────────────
  oledStatus("Init MLX90614...");
  if (!mlx.begin()) {
    Serial.println("ERROR: MLX90614 not found.");
    oledStatus("MLX90614 ERROR!");
    while (1);
  }
  Serial.println("MLX90614 OK");
  oledStatus("MLX90614 OK");
  delay(500);

  // ── MPU6050 ───────────────────────────────
  oledStatus("Init MPU6050...");
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found.");
    oledStatus("MPU6050 ERROR!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 OK");
  oledStatus("MPU6050 OK");
  delay(500);

  // ── Pre-fill SpO2 buffer ───────────────────
  oledStatus("Filling SpO2 buf..");
  Serial.println("Filling SpO2 buffer (100 samples)...");
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
    // Progress bar on OLED
    if (i % 10 == 0) {
      display.fillRect(0, 54, map(i, 0, 100, 0, SCREEN_WIDTH), 8, SSD1306_WHITE);
      display.display();
    }
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid, &heartRate, &hrValid
  );
  oledStatus("All systems READY!");
  delay(1000);
  Serial.println("Ready!\n");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {

  // ── 1. Update SpO2 / HR (rolling window) ──────────────────
  for (byte i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }
  for (byte i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid, &heartRate, &hrValid
  );

  // Beat-to-beat HR
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60.0 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_ARRAY_SIZE;
      beatAvg = 0;
      for (byte x = 0; x < RATE_ARRAY_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_ARRAY_SIZE;
    }
  }

  // ── 2. Read MLX90614 ──────────────────────────────────────
  bodyTemp    = mlx.readObjectTempC();
  ambientTemp = mlx.readAmbientTempC();

  // ── 3. Read MPU6050 ──────────────────────────────────────
  sensors_event_t accel_evt, gyro_evt, temp_evt;
  mpu.getEvent(&accel_evt, &gyro_evt, &temp_evt);
  ax   = accel_evt.acceleration.x;
  ay   = accel_evt.acceleration.y;
  az   = accel_evt.acceleration.z;
  aMag = sqrt(ax * ax + ay * ay + az * az);

  // ── 4. Fall detection ─────────────────────────────────────
  static bool phase1 = false;
  if (!fallDetected) {
    if (!phase1 && aMag > FALL_THRESHOLD)  phase1 = true;
    if ( phase1  && aMag < REST_THRESHOLD) {
      fallDetected = true;
      fallTime = millis();
      Serial.println("⚠️  FALL DETECTED!");
      phase1 = false;
      currentScreen = 3;   // jump to status screen on fall
    }
  }
  if (fallDetected && (millis() - fallTime > FALL_RESET_MS)) {
    fallDetected = false;
  }

  // ── 5. OLED update (rotate screens) ──────────────────────
  if (millis() - lastScreen >= SCREEN_INTERVAL_MS) {
    lastScreen = millis();
    if (!fallDetected) {   // don't auto-rotate away during fall alert
      currentScreen = (currentScreen + 1) % TOTAL_SCREENS;
    }
  }
  // Always render the current screen
  switch (currentScreen) {
    case 0: screenHeartRate();  break;
    case 1: screenTemperature();break;
    case 2: screenAccel();      break;
    case 3: screenStatus();     break;
  }

  // ── 6. Serial debug ───────────────────────────────────────
  int hrShow   = hrValid   ? (int)heartRate : beatAvg;
  int spo2Show = spo2Valid ? (int)spo2Value : -1;
  Serial.printf("[HR] %d BPM (avg %d)  |  [SpO2] %d%%  |  [Temp] Body:%.1f°C  Amb:%.1f°C\n",
    hrShow, beatAvg, spo2Show, bodyTemp, ambientTemp);
  Serial.printf("[IMU] ax:%.2f ay:%.2f az:%.2f  |mag:%.2f  |Fall:%s\n",
    ax, ay, az, aMag, fallDetected ? "YES" : "no");

  // ── 7. Upload to ThingSpeak ───────────────────────────────
  if (millis() - lastUpload >= UPLOAD_INTERVAL_MS) {
    lastUpload = millis();

    int hrToSend  = hrValid   ? (int)heartRate : beatAvg;
    int spo2Send  = spo2Valid ? (int)spo2Value : 0;

    ThingSpeak.setField(1, hrToSend);
    ThingSpeak.setField(2, spo2Send);
    ThingSpeak.setField(3, bodyTemp);
    ThingSpeak.setField(4, ambientTemp);
    ThingSpeak.setField(5, ax);
    ThingSpeak.setField(6, ay);
    ThingSpeak.setField(7, az);
    ThingSpeak.setField(8, (int)(fallDetected ? 1 : 0));

    String status = "OK";
    if (fallDetected)               status = "FALL ALERT";
    else if (spo2Send < 95 && spo2Send > 0) status = "LOW SpO2";
    else if (hrToSend > 100)        status = "HIGH HR";
    else if (bodyTemp > 38.0)       status = "FEVER";
    ThingSpeak.setStatus(status);

    int httpCode = ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_KEY);
    if (httpCode == 200) {
      Serial.println("✅ ThingSpeak upload OK");
    } else {
      Serial.printf("❌ ThingSpeak error: %d\n", httpCode);
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting WiFi...");
        WiFi.reconnect();
        delay(3000);
      }
    }
    Serial.println("──────────────────────────────────");
  }

  delay(10);
}
