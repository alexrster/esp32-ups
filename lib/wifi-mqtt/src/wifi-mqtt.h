#ifndef __WIFI_MQTT
#define __WIFI_MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <queue>

#define WIFI_HOSTNAME                 "esp32-ups-01"
#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"
#define WIFI_RECONNECT_MILLIS         10000
#define WIFI_WATCHDOG_MILLIS          60000

#define MQTT_SERVER_NAME              "ns2.in.qx.zone"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define MQTT_TOPIC_PREFIX             "dev/" MQTT_CLIENT_ID
#define MQTT_STATUS_TOPIC             MQTT_TOPIC_PREFIX "/status"
#define MQTT_VERSION_TOPIC            MQTT_TOPIC_PREFIX "/version"
#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"
#define MQTT_QUEUE_MAX_SIZE           100

typedef std::function<void(uint8_t*, unsigned int)> message_handler_t;

extern WiFiClient wifiClient;
extern PubSubClient pubSubClient;

void wifi_setup();
bool wifi_loop(unsigned long now);
bool pubsub_queue_message(const char* topic, const char* payload, boolean retained = false);
bool pubsub_queue_publish();
void pubsub_subscribe(const char* topic, message_handler_t handler);

#endif