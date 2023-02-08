#include "wifi-mqtt.h"
#include <queue>
#include <list>

struct message_t {
  String topic;
  String payload;
  bool retained;

  message_t(const char* topic, const char* payload, boolean retained) 
    : topic(topic), payload(payload), retained(retained)
  { }
};

struct topic_subscription_t {
  String topic;
  uint8_t qos;
  message_handler_t handler;

  topic_subscription_t(const char* topic, uint8_t qos, message_handler_t handler)
    : topic(topic), qos(qos), handler(handler)
  { }

  topic_subscription_t(const char* topic, message_handler_t handler)
    : topic(topic), qos(0), handler(handler)
  { }
};

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

std::queue<message_t> messageQueue;
std::list<topic_subscription_t> topicSubscriptions;

unsigned long 
  lastWifiReconnect = 0,
  lastPubSubReconnectAttempt = 0,
  lastWifiOnline = 0;

bool reconnectPubSub(unsigned long now) {
  if (now - lastPubSubReconnectAttempt > MQTT_RECONNECT_MILLIS) {
    lastPubSubReconnectAttempt = now;

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, true)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, MQTT_STATUS_ONLINE_MSG, true);
      pubSubClient.publish(MQTT_IPADDRESS_TOPIC, WiFi.localIP().toString().c_str(), true);

#ifdef VERSION
      pubSubClient.publish(MQTT_VERSION_TOPIC, VERSION, true);
#endif

      for (auto s : topicSubscriptions) {
        pubSubClient.subscribe(s.topic.c_str(), s.qos);
      }
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
  
  return pubsub_queue_publish() && mqtt_loop(now);
}

void mqtt_on_message(char* topic, uint8_t* payload, unsigned int length) {
  for (auto& s : topicSubscriptions) {
    if (s.topic.equals(topic)) {
      s.handler(payload, length);
    }
  }
}

void wifi_setup() {
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(wifi_ps_type_t::WIFI_PS_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);
  pubSubClient.setCallback(mqtt_on_message);
}

bool pubsub_queue_message(const char* topic, const char* payload, boolean retained) {
  if (messageQueue.size() >= MQTT_QUEUE_MAX_SIZE) return false;

  messageQueue.push(message_t(topic, payload, retained));
  return true;
}

bool pubsub_queue_publish() {
  bool result = true;
  while (!messageQueue.empty()) {
    auto m = messageQueue.front();
    messageQueue.pop();

    result &= pubSubClient.publish(m.topic.c_str(), m.payload.c_str(), m.retained);
  }

  return result;
}

void pubsub_subscribe(const char* topic, uint8_t qos, message_handler_t handler) {
  topicSubscriptions.push_back(topic_subscription_t(topic, qos, handler));
}

void pubsub_subscribe(const char* topic, message_handler_t handler) {
  topicSubscriptions.push_back(topic_subscription_t(topic, handler));
}
