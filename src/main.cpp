#include <Arduino.h>
#include <esp_task_wdt.h>
#include <wifi-mqtt.h>
#include <ArduinoOTA.h>

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"

#define INT_LED                       15

#define BATTERY_VOLTAGE_UPDATE_MS     5000

#define AC_DETECTOR_PIN               4
#define AC_DETECTOR_TIMEOUT_MS        (1000 / 50 / 2) * 2 + 3    // allow to miss 2 zero crosses @50Hz, +3ms
//                                     ms     Hz   crosses

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
  lastAcValue = 0,
  lastBatteryVoltageReadMs = 0,
  zc = 0;

uint16_t
  batteryLevel = 0;

ups_mode_t 
  current_mode = LINE;

void IRAM_ATTR acDetectorISR() {
  zc++;
  lastAcValue = now;
}

void on_ota_begin() {
  if (current_mode != LINE) {

  }
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

  // digitalWrite(INT_LED, HIGH);
  // digitalWrite(RELAY_INV_PIN, LOW);
  // digitalWrite(RELAY_CHRG_PIN, LOW);
  // delay(1000);
  // digitalWrite(INT_LED, LOW);
  // digitalWrite(RELAY_INV_PIN, HIGH);
  // digitalWrite(RELAY_CHRG_PIN, HIGH);
  // delay(1000);
  // digitalWrite(INT_LED, HIGH);
  // digitalWrite(RELAY_INV_PIN, LOW);
  // digitalWrite(RELAY_CHRG_PIN, LOW);
  // delay(500);
  // digitalWrite(INT_LED, LOW);
}

void ac_loop() {
  if (now - lastAcValue > AC_DETECTOR_TIMEOUT_MS && zc == 0) {
    if (current_mode == LINE) {
      // ELECTRICITY CUT OFF CASE!!!
      log_i("ELECTRICITY CUT OFF! Switching to Battery mode!");
      current_mode = BATTERY;

      digitalWrite(INT_LED, HIGH);
      digitalWrite(RELAY_CHRG_PIN, HIGH); // Switch CHARGER OFF
      digitalWrite(RELAY_INV_PIN, LOW); // Switch INVERTER ON

      pubSubClient.publish(MQTT_CLIENT_ID "/mode", "battery");
    }

    // auto value = zc;
    // pubSubClient.publish(MQTT_CLIENT_ID "/ac/count", String(value).c_str());
    // pubSubClient.publish(MQTT_CLIENT_ID "/ac/millis", String(now - lastAcPublish).c_str());
  } else {
    zc = 0;
    if (current_mode == BATTERY) {
      log_i("ELECTRICITY RESTORED! Switching to Line mode!");
      current_mode = LINE;

      digitalWrite(INT_LED, LOW);
      // digitalWrite(RELAY_CHRG_PIN, HIGH); // Switch CHARGER OFF
      digitalWrite(RELAY_INV_PIN, HIGH); // Switch INVERTER OFF

      pubSubClient.publish(MQTT_CLIENT_ID "/mode", "line");
    }
  }
}

void battery_voltage_loop() {
  if (now - lastBatteryVoltageReadMs > BATTERY_VOLTAGE_UPDATE_MS) {
    lastBatteryVoltageReadMs = now;

    batteryLevel = analogRead(BATTERY_VOLTAGE_PIN);
    pubSubClient.publish(MQTT_CLIENT_ID "/battery/raw", String(batteryLevel).c_str());
  }
}

void loop() {
  esp_task_wdt_reset();

  now = millis();
  ac_loop();
  battery_voltage_loop();
  if (wifi_loop(now)) {
    if (current_mode == LINE) {
      ArduinoOTA.handle();
    }
  }

  delay(1);
}