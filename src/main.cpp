#include <Arduino.h>
#include <esp_task_wdt.h>

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"
#define WIFI_RECONNECT_MILLIS         10000
#define WIFI_WATCHDOG_MILLIS          60000

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME                 "esp32-ups-01"
#endif

#define MQTT_SERVER_NAME              "10.9.9.96"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define INT_LED                       15

#define AC_DETECTOR_PIN               4
#define AC_DETECTOR_TIMEOUT_MS        (1000 / 50 / 2) * 2 + 3    // allow to miss 2 zero crosses @50Hz, +3ms
//                                     ms     Hz   crosses

#define RELAY_INV_PIN                 16
#define RELAY_CHRG_PIN                17

typedef enum : uint8_t {
  LINE = 0,
  BATTERY = 1
} ups_mode_t;

unsigned long 
  now = 0,
  lastAcValue = 0,
  zc = 0;

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

  digitalWrite(RELAY_INV_PIN, HIGH);
  digitalWrite(RELAY_CHRG_PIN, HIGH);

  ac_setup();
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
    }
  }
}

void loop() {
  esp_task_wdt_reset();

  now = millis();
  ac_loop();

  delay(1);
}
