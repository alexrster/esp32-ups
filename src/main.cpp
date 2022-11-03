#include <Arduino.h>
#include <esp_task_wdt.h>
#include <wifi-mqtt.h>
#include <ArduinoOTA.h>

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"

#define INT_LED                       15

#define BATTERY_VOLTAGE_UPDATE_MS     5000
#define BATTERY_VOLTAGE_READ_MS       500

#define AC_DETECTOR_PIN               4
#define AC_DETECTOR_TIMEOUT_MS        (1000 / 50 / 2) * 2     // allow to miss 2 zero crosses @50Hz
//                                     ms     Hz   crosses
#define MODE_CHANGE_MIN_DELAY_MS      3000

#define RELAY_INV_PIN                 16
#define RELAY_CHRG_PIN                17
#define BATTERY_VOLTAGE_PIN           2
#define H35_SCL                       35
#define H35_SDA                       33
#define H36_PIN1                      13
#define H36_PIN2                      14

typedef enum : uint8_t {
  LINE = 0,
  BATTERY = 1
} ups_mode_t;

unsigned long 
  now = 0,
  last = 0,
  lastAcValue = 0,
  lastBatteryVoltageReadMs = 0,
  lastBatteryVoltageUpdateMs = 0,
  lastChargerStatsUpdateMs = 0,
  lastChargerUpdateMs = 0,
  lastModeChangeMs = 0,
  zc = 0;

bool
  wifiConnected = false,
  batteryChargerActive = true;

uint16_t
  batteryLevel = 0,
  maxBatteryLevel = 0,
  minBatteryLevel = 10000;

ups_mode_t 
  current_mode = LINE;

void IRAM_ATTR acDetectorISR() {
  zc++;
  lastAcValue = now;
}

void ac_setup() {
  pinMode(AC_DETECTOR_PIN, INPUT);
  attachInterrupt(AC_DETECTOR_PIN, acDetectorISR, RISING);
}

void setup() {
  pinMode(RELAY_INV_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(RELAY_CHRG_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(INT_LED, OUTPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  digitalWrite(RELAY_INV_PIN, HIGH);
  digitalWrite(RELAY_CHRG_PIN, HIGH);

  ac_setup();
  log_i("AC DETECTOR SETUP COMPLETE!");

  wifi_setup();
  log_i("WIFI SETUP COMPLETE!");

  ArduinoOTA.begin();
  log_i("ARDUINO-OTA SETUP COMPLETE!");

  log_i("SETUP COMPLETE!");
}

void onLineOff() {
  log_i("ELECTRICITY CUT OFF! Switching to Battery mode!");
  current_mode = BATTERY;
  lastModeChangeMs = now;
  batteryChargerActive = false;

  digitalWrite(INT_LED, HIGH);
  digitalWrite(RELAY_CHRG_PIN, HIGH); // Switch CHARGER OFF
  digitalWrite(RELAY_INV_PIN, LOW); // Switch INVERTER ON

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/active", "0");
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/mode", "battery");
}

void onLineOn() {
  log_i("ELECTRICITY RESTORED! Switching to Line mode!");
  current_mode = LINE;
  lastModeChangeMs = now;

  digitalWrite(INT_LED, LOW);
  digitalWrite(RELAY_CHRG_PIN, LOW); // Switch CHARGER ON
  digitalWrite(RELAY_INV_PIN, HIGH); // Switch INVERTER OFF

  lastAcValue = now;
  delay(200);
  lastAcValue = now;

  pubSubClient.publish(MQTT_TOPIC_PREFIX "/mode", "line");

  minBatteryLevel = 10000;
  maxBatteryLevel = 0;
  batteryChargerActive = true;
  lastChargerUpdateMs = now;
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/active", "1");
}

void battery_charger_loop() {
  if (current_mode == BATTERY) return;

  if (now - lastChargerStatsUpdateMs > 1000) {
    lastChargerStatsUpdateMs = now;

    maxBatteryLevel = batteryLevel < maxBatteryLevel ? maxBatteryLevel : batteryLevel;
    minBatteryLevel = batteryLevel > minBatteryLevel ? minBatteryLevel : batteryLevel;
  }

  if (now - lastChargerUpdateMs > 90000) {
    lastChargerUpdateMs = now;

    if (maxBatteryLevel < 7350 && !batteryChargerActive) {
      batteryChargerActive = true;
      digitalWrite(RELAY_CHRG_PIN, LOW); // Switch CHARGER ON
    }
    else if (maxBatteryLevel > 7660 && maxBatteryLevel - minBatteryLevel < 250 && batteryChargerActive) {
      batteryChargerActive = false;
      digitalWrite(RELAY_CHRG_PIN, HIGH); // Switch CHARGER OFF
    }

    pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/min_raw", String(minBatteryLevel).c_str());
    pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/max_raw", String(maxBatteryLevel).c_str());
    pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/active", batteryChargerActive ? "1" : "0");

    maxBatteryLevel = 0;
    minBatteryLevel = 10000;
  }
}

void ac_loop() {
  // ELECTRICITY CUT OFF
  if (now - lastAcValue > AC_DETECTOR_TIMEOUT_MS && zc == 0) {
    if (current_mode == LINE && now - lastModeChangeMs > MODE_CHANGE_MIN_DELAY_MS) {
      onLineOff();
    }
  } 

  // ELECTRICITY RESUMED
  else {
    zc = 0;
    if (current_mode == BATTERY && now - lastModeChangeMs > MODE_CHANGE_MIN_DELAY_MS) {
      onLineOn();
    }
  }
}

void battery_voltage_loop() {
  if (now - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_READ_MS) {
    lastBatteryVoltageReadMs = now;
    batteryLevel = (batteryLevel + analogRead(BATTERY_VOLTAGE_PIN)) / 2;
  }

  if (now - lastBatteryVoltageUpdateMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageUpdateMs = now;

    pubSubClient.publish(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str());
    pubSubClient.publish(MQTT_TOPIC_PREFIX "/mode", current_mode == LINE ? "line" : "battery");
  }
}

void on_wifi_reconnect() {
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/mode", current_mode == LINE ? "line" : "battery");
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/charger/active", batteryChargerActive ? "1" : "0");
  pubSubClient.publish(MQTT_TOPIC_PREFIX "/wifi/connected_ms", String(now).c_str());
}

void loop() {
  esp_task_wdt_reset();
  now = millis();

  ac_loop();
  battery_voltage_loop();
  battery_charger_loop();

  bool wifiPrevConnected = wifiConnected;
  wifiConnected = wifi_loop(now);
  if (wifiConnected) {
    if (!wifiPrevConnected) {
      on_wifi_reconnect();
    }

    if (current_mode == LINE) {
      ArduinoOTA.handle();
    }
  }

  last = now;
  delay(1);
}