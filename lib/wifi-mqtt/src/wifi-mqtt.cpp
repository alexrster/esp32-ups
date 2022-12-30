#include "wifi-mqtt.h"

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

unsigned long 
  lastWifiReconnect = 0,
  lastPubSubReconnectAttempt = 0,
  lastWifiOnline = 0;

bool reconnectPubSub(unsigned long now) {
  if (now - lastPubSubReconnectAttempt > MQTT_RECONNECT_MILLIS) {
    lastPubSubReconnectAttempt = now;

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, true)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, MQTT_STATUS_ONLINE_MSG, true);

#ifdef VERSION
      pubSubClient.publish(MQTT_VERSION_TOPIC, VERSION, true);
#endif
    }
    
    return pubSubClient.connected();
  }

  return false;
}

bool mqtt_loop(unsigned long now) {
  if (!pubSubClient.connected() && !reconnectPubSub(now)) {
    return false;
  }

  return pubSubClient.loop();
}

bool wifi_loop(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) {
#ifdef WIFI_WATCHDOG_ENABLED
    if (now - lastWifiOnline > WIFI_WATCHDOG_MILLIS) esp_restart();
    else 
#endif
    if (now - lastWifiReconnect > WIFI_RECONNECT_MILLIS) {
      lastWifiReconnect = now;

      if (WiFi.reconnect()) {
        lastWifiOnline = now;
        return true;
      }
    }

    return false;
  }
  
  lastWifiReconnect = now;
  lastWifiOnline = now;
  
  return mqtt_loop(now);
}

void wifi_setup() {
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(wifi_ps_type_t::WIFI_PS_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);
}
