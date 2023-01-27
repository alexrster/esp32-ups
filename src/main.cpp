#include <Arduino.h>
#include <esp_task_wdt.h>
#include <wifi-mqtt.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BATTERY_VOLTAGE_UPDATE_MS     5000
#define BATTERY_VOLTAGE_READ_MS       500

#define AC_DETECTOR_PIN               4
#define AC_DETECTOR_TIMEOUT_MS        (1000 / 50 / 2) * 2 +        5   // allow to miss 2 zero crosses @50Hz, +5ms
// #define AC_DETECTOR_TIMEOUT_MS     (1000 / 50 / 2) * 3 +        5   // allow to miss 3 zero crosses @50Hz, +5ms
// #define AC_DETECTOR_TIMEOUT_MS     (1000 / 50 / 2) * 4 +        5   // allow to miss 4 zero crosses @50Hz, +5ms
//                                    |ms    |Hz       |crosses   |ms
#define MODE_CHANGE_TO_BATTERY_MS     3000
#define MODE_CHANGE_TO_LINE_MS        10000

#define RELAY_INV_PIN                 16
#define RELAY_CHRG_PIN                17
#define BATTERY_VOLTAGE_PIN           2
#define H35_SCL                       35
#define H35_SDA                       33
#define H36_PIN1                      13
#define H36_PIN2                      14
#define DS18B20_PIN                   15

// #define INT_LED_PIN                   15 // IT CONFLICTS WITH TEMPERATURE (DS18B20) SENSORS -- DO NOT USE IT!!!

#define TSENSORS_READ_MS              10000
#define TSENSORS_COUNT                3

const uint8_t tSensor_ids[TSENSORS_COUNT][8] = {
  { 0x28, 0x00, 0x5C, 0x34, 0x26, 0x4C, 0x0C, 0xDC },
  { 0x28, 0x00, 0x92, 0x6C, 0x26, 0x4C, 0x0C, 0x98 },
  { 0x28, 0x00, 0x91, 0x6F, 0x26, 0x4C, 0x16, 0xBD }
};

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
  lastTSensorsReadMs = 0,
  lastAcPeriodPublishMs = 10000,
  lastModeBatteryMs = 0,
  lastModeLineMs = 0,
  zc = 0;

bool
  wifiConnected = false,
  batteryChargerActive = true;

uint16_t
  batteryLevel = 0,
  maxBatteryLevel = 0,
  minBatteryLevel = 10000,
  lastMaxBatteryLevel = 0;

ups_mode_t 
  current_mode = LINE;

OneWire oneWireBus(DS18B20_PIN);
DallasTemperature tSensors(&oneWireBus);

float 
  tSensor_vals[TSENSORS_COUNT];

void IRAM_ATTR acDetectorISR() {
  zc++;
  lastAcValue = millis();
}

void set_battery_charger(bool state) {
  lastChargerUpdateMs = now;
  batteryChargerActive = state;
  digitalWrite(RELAY_CHRG_PIN, state ? HIGH : LOW); // HIGH = ON
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/active", state ? "1" : "0");
}

void onLineOff() {
  log_i("ELECTRICITY CUT OFF! Switching to Battery mode!");
  current_mode = BATTERY;
  lastModeChangeMs = millis();

  digitalWrite(RELAY_INV_PIN, LOW); // Switch INVERTER ON

  set_battery_charger(false);
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/mode", "battery");
}

void onLineOn() {
  log_i("ELECTRICITY RESTORED! Switching to Line mode!");
  current_mode = LINE;
  lastModeChangeMs = millis();

  digitalWrite(RELAY_INV_PIN, HIGH); // Switch INVERTER OFF

  lastAcValue = millis();
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/mode", "line");

  minBatteryLevel = 10000;
  maxBatteryLevel = 0;
  set_battery_charger(true);
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

    if (maxBatteryLevel < 7150 && !batteryChargerActive) {
      set_battery_charger(true);
    }
    else if (maxBatteryLevel >= 7220 && batteryChargerActive) {
      if (std::abs((int)(lastMaxBatteryLevel - maxBatteryLevel)) < 10 && maxBatteryLevel - minBatteryLevel < 100) {
        lastMaxBatteryLevel = 0;
        set_battery_charger(false);
      }
      else {
        lastMaxBatteryLevel = maxBatteryLevel;
      }
    }

    pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/min_raw", String(minBatteryLevel).c_str());
    pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/max_raw", String(maxBatteryLevel).c_str());
    pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/active", batteryChargerActive ? "1" : "0");

    maxBatteryLevel = 0;
    minBatteryLevel = 10000;
  }
}

void ac_loop() {
  auto n = millis();
  // ELECTRICITY CUT OFF
  if (n - lastAcValue > AC_DETECTOR_TIMEOUT_MS && zc == 0) {
    if (current_mode == BATTERY) {
      lastModeBatteryMs = n;
    }

    if (current_mode == LINE && n - lastModeChangeMs > MODE_CHANGE_TO_BATTERY_MS) {
      onLineOff();
    }
  } 

  // ELECTRICITY RESUMED
  else {
    zc = 0;
    if (current_mode == LINE) {
      lastModeLineMs = n;
    }

    if (current_mode == BATTERY && n - lastModeChangeMs > MODE_CHANGE_TO_LINE_MS && n - lastModeBatteryMs > MODE_CHANGE_TO_LINE_MS) {
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

    pubsub_queue_message(MQTT_TOPIC_PREFIX "/battery/raw", String(batteryLevel).c_str());
    pubsub_queue_message(MQTT_TOPIC_PREFIX "/mode", current_mode == LINE ? "line" : "battery");
    pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/active", batteryChargerActive ? "1" : "0");
  }
}

void tSensors_publish() {
  for (uint8_t i=0; i<TSENSORS_COUNT; i++) {
    String topic_name(MQTT_TOPIC_PREFIX "/temperature/");
    topic_name.concat(i+1);

    if (tSensors.isConnected(tSensor_ids[i])) {
      pubsub_queue_message(topic_name.c_str(), String(tSensor_vals[i]).c_str());

      topic_name.concat("/status");
      pubsub_queue_message(topic_name.c_str(), "online");
    }
    else {
      topic_name.concat("/status");
      pubsub_queue_message(topic_name.c_str(), "offline");
    }
  }
}

void tSensors_loop() {
  if (now - lastTSensorsReadMs > TSENSORS_READ_MS) {
    lastTSensorsReadMs = now;

    auto cnt = tSensors.getDeviceCount();
    pubsub_queue_message(MQTT_TOPIC_PREFIX "/temperature/sensors_count", String(cnt).c_str());

    tSensors.requestTemperatures();
    for (uint8_t i=0; i<TSENSORS_COUNT; i++) {
      auto val = tSensors.getTempC(tSensor_ids[i]);
      if (tSensor_vals[i] == 0) tSensor_vals[i] = val;
      else tSensor_vals[i] = (tSensor_vals[i] + val) / 2;
    }

    tSensors_publish();
  }
}

void on_wifi_reconnect() {
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/mode", current_mode == LINE ? "line" : "battery");
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/charger/active", batteryChargerActive ? "1" : "0");
  pubsub_queue_message(MQTT_TOPIC_PREFIX "/wifi/connected_ms", String(now).c_str());

  tSensors_publish();
}

void loop() {
  esp_task_wdt_reset();
  now = millis();

  ac_loop();
  tSensors_loop();
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

void on_mqtt_restart(uint8_t *payload, unsigned int length) {
  if (length == 0) return;

  if (payload[0] == '1' || payload[0] == 't' || (length > 1 && payload[1] == 'n')) ESP.restart();
}

void on_mqtt_charger_set(uint8_t *payload, unsigned int length) {
  if (length == 0) return;

  if (payload[0] == '1' || payload[0] == 't' || (length > 1 && payload[1] == 'n')) set_battery_charger(true);
  else if (payload[0] == '0' || payload[0] == 'f' || (length > 1 && payload[1] == 'f')) set_battery_charger(false);
}

void on_mqtt_mode_set(uint8_t *payload, unsigned int length) {
  if (length == 0) return;

  if (payload[0] == 'l') onLineOn();
  else if (payload[0] == 'b') onLineOff();
}

void ac_setup() {
  pinMode(AC_DETECTOR_PIN, INPUT);
  attachInterrupt(AC_DETECTOR_PIN, acDetectorISR, RISING);
}

void tSensors_setup() {
  tSensors.begin();

  for (uint8_t i=0; i<TSENSORS_COUNT; i++)
    tSensor_vals[i] = 0;
 }

void setup() {
  pinMode(RELAY_INV_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(RELAY_CHRG_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  digitalWrite(RELAY_INV_PIN, HIGH);   // INVERTER OFF
  digitalWrite(RELAY_CHRG_PIN, HIGH);  // CHARGER ON

  ac_setup();
  log_i("AC DETECTOR SETUP COMPLETE!");

  tSensors_setup();
  log_i("TEMPERATURE SENSORS SETUP COMPLETE!");

  pubsub_subscribe(MQTT_TOPIC_PREFIX "/charger/active/set", on_mqtt_charger_set);
  pubsub_subscribe(MQTT_TOPIC_PREFIX "/mode/set", on_mqtt_mode_set);
  pubsub_subscribe(MQTT_TOPIC_PREFIX "/restart", on_mqtt_restart);
  log_i("PUBSUB SUBSDCRIPTIONS CONFIGURED!");

  wifi_setup();
  log_i("WIFI SETUP COMPLETE!");

  ArduinoOTA.begin();
  log_i("ARDUINO-OTA SETUP COMPLETE!");

  log_i("SETUP COMPLETE!");
}
