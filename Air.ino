#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ===============================
// PIN MAPPING
// ===============================

const int PMS_RX_PIN = 16;   // PMS5003 TXD -> ESP32 GPIO16
const int PMS_TX_PIN = 17;   // Optional, PMS5003 RXD -> GPIO17 or leave open

const int MQ135_PIN = 34;    // MQ135 AO divider output -> GPIO34

const int DHT_PIN = 4;
#define DHT_TYPE DHT22

const int OLED_SDA = 21;
const int OLED_SCL = 22;

const int BUZZER_PIN = 25;

const int BUTTON_PIN = 32;

const int GREEN_LED_PIN = 13;
const int YELLOW_LED_PIN = 14;
const int RED_LED_PIN = 27;

// ===============================
// OLED SETUP
// ===============================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===============================
// SENSOR OBJECTS
// ===============================

DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial pmsSerial(2);

// ===============================
// STATUS CONSTANTS
// ===============================

const int GENERAL_MODE = 0;
const int SENSITIVE_MODE = 1;

const int STATUS_GOOD = 0;
const int STATUS_WARNING = 1;
const int STATUS_POOR = 2;

int currentMode = GENERAL_MODE;
int currentStatus = STATUS_GOOD;

// Your buzzer was likely active-low.
// LOW = buzzer ON, HIGH = buzzer OFF.
const bool BUZZER_ACTIVE_HIGH = false;

// ===============================
// PMS5003 DATA
// ===============================

struct PMSData {
  uint16_t framelen;
  uint16_t pm10_standard;
  uint16_t pm25_standard;
  uint16_t pm100_standard;
  uint16_t pm10_env;    // PM1.0
  uint16_t pm25_env;    // PM2.5
  uint16_t pm100_env;   // PM10
  uint16_t particles_03um;
  uint16_t particles_05um;
  uint16_t particles_10um;
  uint16_t particles_25um;
  uint16_t particles_50um;
  uint16_t particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

PMSData pmsData;
bool hasPMSData = false;

// ===============================
// LIVE SENSOR VALUES
// ===============================

float temperatureC = NAN;
float humidity = NAN;

int mqRaw = 0;
int mqBaseline = 0;
bool mqBaselineReady = false;

float mqAdcVoltage = 0.0;
float mqSensorVoltage = 0.0;

// MQ135 voltage divider:
// MQ135 AO --- 10k --- ADC node --- 20k --- GND
// ESP32 sees 2/3 of MQ135 AO.
// Original MQ135 AO estimate = ADC voltage * 1.5
const float DIVIDER_MULTIPLIER = 1.5;

// ===============================
// TIMING
// ===============================

unsigned long lastDHTRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastButtonCheck = 0;

const unsigned long DHT_INTERVAL_MS = 2000;
const unsigned long OLED_INTERVAL_MS = 1000;
const unsigned long SERIAL_INTERVAL_MS = 2000;
const unsigned long BUTTON_DEBOUNCE_MS = 250;

unsigned long mqBaselineStart = 0;
long mqBaselineSum = 0;
int mqBaselineCount = 0;
const unsigned long MQ_BASELINE_TIME_MS = 15000;

// ===============================
// HELPER FUNCTIONS
// ===============================

void setBuzzer(bool on) {
  if (BUZZER_ACTIVE_HIGH) {
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
  }
}

const char* modeText() {
  if (currentMode == GENERAL_MODE) return "GENERAL";
  return "SENSITIVE";
}

const char* statusText() {
  if (currentStatus == STATUS_GOOD) return "GOOD";
  if (currentStatus == STATUS_WARNING) return "WARNING";
  return "POOR";
}

// ===============================
// PMS5003 READER
// ===============================

bool readPMSFrame() {
  if (pmsSerial.available() < 32) {
    return false;
  }

  while (pmsSerial.available() >= 2) {
    if (pmsSerial.peek() == 0x42) {
      int first = pmsSerial.read();
      int second = pmsSerial.read();

      if (first == 0x42 && second == 0x4D) {
        uint8_t buffer[30];
        int bytesRead = pmsSerial.readBytes(buffer, 30);

        if (bytesRead != 30) {
          return false;
        }

        uint16_t sum = 0x42 + 0x4D;
        for (int i = 0; i < 28; i++) {
          sum += buffer[i];
        }

        auto read16 = [&](int idx) -> uint16_t {
          return (uint16_t(buffer[idx]) << 8) | buffer[idx + 1];
        };

        pmsData.framelen        = read16(0);
        pmsData.pm10_standard   = read16(2);
        pmsData.pm25_standard   = read16(4);
        pmsData.pm100_standard  = read16(6);
        pmsData.pm10_env        = read16(8);
        pmsData.pm25_env        = read16(10);
        pmsData.pm100_env       = read16(12);
        pmsData.particles_03um  = read16(14);
        pmsData.particles_05um  = read16(16);
        pmsData.particles_10um  = read16(18);
        pmsData.particles_25um  = read16(20);
        pmsData.particles_50um  = read16(22);
        pmsData.particles_100um = read16(24);
        pmsData.unused          = read16(26);
        pmsData.checksum        = read16(28);

        if (sum != pmsData.checksum) {
          Serial.println("PMS5003 checksum error");
          return false;
        }

        return true;
      }
    } else {
      pmsSerial.read();
    }
  }

  return false;
}

// ===============================
// MQ135
// ===============================

void readMQ135() {
  long sum = 0;
  const int samples = 20;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(MQ135_PIN);
    delay(2);
  }

  mqRaw = sum / samples;

  mqAdcVoltage = (mqRaw / 4095.0) * 3.3;
  mqSensorVoltage = mqAdcVoltage * DIVIDER_MULTIPLIER;
}

void updateMQBaseline() {
  if (mqBaselineReady) return;

  readMQ135();

  mqBaselineSum += mqRaw;
  mqBaselineCount++;

  if (millis() - mqBaselineStart >= MQ_BASELINE_TIME_MS) {
    if (mqBaselineCount > 0) {
      mqBaseline = mqBaselineSum / mqBaselineCount;
    } else {
      mqBaseline = mqRaw;
    }

    mqBaselineReady = true;

    Serial.print("MQ135 baseline set to raw ADC: ");
    Serial.println(mqBaseline);
  }
}

// ===============================
// DHT22
// ===============================

void readDHT22() {
  if (millis() - lastDHTRead < DHT_INTERVAL_MS) return;
  lastDHTRead = millis();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h)) humidity = h;
  if (!isnan(t)) temperatureC = t;
}

// ===============================
// BUTTON
// ===============================

void handleButton() {
  static int lastButtonState = HIGH;

  if (millis() - lastButtonCheck < BUTTON_DEBOUNCE_MS) return;
  lastButtonCheck = millis();

  int buttonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    if (currentMode == GENERAL_MODE) {
      currentMode = SENSITIVE_MODE;
    } else {
      currentMode = GENERAL_MODE;
    }

    Serial.print("Mode changed to: ");
    Serial.println(modeText());
  }

  lastButtonState = buttonState;
}

// ===============================
// AIR QUALITY LOGIC
// ===============================

int classifyAirQuality() {
  int pm25 = hasPMSData ? pmsData.pm25_env : 0;
  int mqDelta = mqBaselineReady ? (mqRaw - mqBaseline) : 0;

  int pm25Warning;
  int pm25Poor;
  int mqWarningDelta;
  int mqPoorDelta;

  if (currentMode == GENERAL_MODE) {
    pm25Warning = 35;
    pm25Poor = 55;

    mqWarningDelta = 150;
    mqPoorDelta = 350;
  } else {
    pm25Warning = 12;
    pm25Poor = 35;

    mqWarningDelta = 100;
    mqPoorDelta = 250;
  }

  bool poorPM = pm25 >= pm25Poor;
  bool warningPM = pm25 >= pm25Warning;

  bool poorMQ = mqDelta >= mqPoorDelta;
  bool warningMQ = mqDelta >= mqWarningDelta;

  if (poorPM || poorMQ) {
    return STATUS_POOR;
  }

  if (warningPM || warningMQ) {
    return STATUS_WARNING;
  }

  return STATUS_GOOD;
}

// ===============================
// OUTPUTS
// ===============================

void updateLEDs() {
  digitalWrite(GREEN_LED_PIN, currentStatus == STATUS_GOOD ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, currentStatus == STATUS_WARNING ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, currentStatus == STATUS_POOR ? HIGH : LOW);
}

void updateBuzzer() {
  if (!mqBaselineReady) {
    setBuzzer(false);
    return;
  }

  unsigned long now = millis();

  if (currentStatus == STATUS_GOOD) {
    setBuzzer(false);
  } 
  else if (currentStatus == STATUS_WARNING) {
    // short chirp every 3 seconds
    unsigned long cycle = now % 3000;
    setBuzzer(cycle < 150);
  } 
  else if (currentStatus == STATUS_POOR) {
    // repeated longer beep
    unsigned long cycle = now % 1000;
    setBuzzer(cycle < 500);
  }
}

void updateOLED() {
  if (millis() - lastOLEDUpdate < OLED_INTERVAL_MS) return;
  lastOLEDUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (!mqBaselineReady) {
    display.println("Air Quality");
    display.println("Calibrating MQ135");

    unsigned long elapsed = millis() - mqBaselineStart;
    unsigned long remaining = 0;

    if (elapsed < MQ_BASELINE_TIME_MS) {
      remaining = (MQ_BASELINE_TIME_MS - elapsed) / 1000;
    }

    display.print("Wait: ");
    display.print(remaining);
    display.println(" s");
    display.println("Keep in normal air");
    display.display();
    return;
  }

  display.print("Mode: ");
  display.println(modeText());

  display.print("PM2.5: ");
  if (hasPMSData) {
    display.println(pmsData.pm25_env);
  } else {
    display.println("--");
  }

  display.print("Gas: ");
  display.println(mqRaw);

  display.print("T: ");
  if (!isnan(temperatureC)) {
    display.print(temperatureC, 1);
    display.print("C ");
  } else {
    display.print("--C ");
  }

  display.print("H: ");
  if (!isnan(humidity)) {
    display.print(humidity, 0);
    display.println("%");
  } else {
    display.println("--%");
  }

  display.print("Status: ");
  display.println(statusText());

  display.display();
}

void printSerialStatus() {
  if (millis() - lastSerialPrint < SERIAL_INTERVAL_MS) return;
  lastSerialPrint = millis();

  Serial.println("----- AIR QUALITY MONITOR -----");

  Serial.print("Mode: ");
  Serial.println(modeText());

  Serial.print("Status: ");
  Serial.println(statusText());

  if (hasPMSData) {
    Serial.print("PM1.0: ");
    Serial.print(pmsData.pm10_env);
    Serial.println(" ug/m3");

    Serial.print("PM2.5: ");
    Serial.print(pmsData.pm25_env);
    Serial.println(" ug/m3");

    Serial.print("PM10: ");
    Serial.print(pmsData.pm100_env);
    Serial.println(" ug/m3");
  } else {
    Serial.println("PMS5003: no valid data yet");
  }

  Serial.print("MQ135 raw ADC: ");
  Serial.println(mqRaw);

  Serial.print("MQ135 baseline: ");
  Serial.println(mqBaseline);

  Serial.print("MQ135 delta: ");
  Serial.println(mqRaw - mqBaseline);

  Serial.print("MQ135 ADC voltage: ");
  Serial.print(mqAdcVoltage, 3);
  Serial.println(" V");

  Serial.print("MQ135 estimated AO voltage: ");
  Serial.print(mqSensorVoltage, 3);
  Serial.println(" V");

  Serial.print("Temp: ");
  Serial.print(temperatureC);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();
}

// ===============================
// SETUP
// ===============================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting Portable Air Quality Monitor...");

  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);

  dht.begin();

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  pmsSerial.setTimeout(100);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found. Check wiring or I2C address.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Air Quality");
    display.println("Monitor Starting");
    display.display();
  }

  mqBaselineStart = millis();
  mqBaselineSum = 0;
  mqBaselineCount = 0;
  mqBaselineReady = false;

  Serial.println("Keep the sensor in normal air for MQ135 baseline calibration.");
}

// ===============================
// LOOP
// ===============================

void loop() {
  handleButton();

  if (readPMSFrame()) {
    hasPMSData = true;
  }

  readDHT22();
  readMQ135();

  if (!mqBaselineReady) {
    updateMQBaseline();
    currentStatus = STATUS_GOOD;
  } else {
    currentStatus = classifyAirQuality();
  }

  updateLEDs();
  updateBuzzer();
  updateOLED();
  printSerialStatus();

  delay(50);
}
