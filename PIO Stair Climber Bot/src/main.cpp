#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>

// this program is set up for the rear ESP to subscribe to the front ESP and camera

const char* ssid = "Tufts_Robot";
const char* password = "";
const char* mqtt_broker = "10.243.82.33";
const int mqtt_port = 1883;
const char* topic_front_sub = "status/front";
const char* topic_cam_sub = "status/camera";
const char* topic_pub = "status/rear";
const char* mqtt_client_id = "rear_esp";

WiFiClient espClient;
PubSubClient client(espClient);

const char* default_status = "rear esp message test";

// logging levels for funsies
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3

int current_log_level = LOG_LEVEL_DEBUG;

#define LOG_ERROR(fmt, ...)   if (current_log_level >= LOG_LEVEL_ERROR) { Serial.printf("[ERROR]: " fmt "\n", ##__VA_ARGS__); }
#define LOG_INFO(fmt, ...)    if (current_log_level >= LOG_LEVEL_INFO)  { Serial.printf("[INFO]: " fmt "\n", ##__VA_ARGS__); }
#define LOG_DEBUG(fmt, ...)   if (current_log_level >= LOG_LEVEL_DEBUG) { Serial.printf("[DEBUG]: " fmt "\n", ##__VA_ARGS__); }

void connectWiFi() {
    float time = 0;
    LOG_INFO("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        time += 1;
        LOG_DEBUG("Time elapsed: %.1f seconds", time);
  }
    LOG_INFO("WiFi connected");
}

void connectMQTT() {
    LOG_INFO("Attempting Connection to MQTT Server...");
    while (!client.connected()) {
    
        if (client.connect(mqtt_client_id)) {
            client.subscribe(topic_front_sub);  // Subscribe to the topic
            delay(0.5);
            client.subscribe(topic_cam_sub);
        } 
        else {
            LOG_ERROR("MQTT connection failed, retrying in 5 seconds...");
            delay(5000);
        }
    }

    LOG_INFO("MQTT Client connected as %s", mqtt_client_id);
    LOG_INFO("Subscribed to topic: %s", topic_front_sub);
    LOG_INFO("Subscribed to topic: %s", topic_cam_sub);
}

void update_status(const char* name, const char* status) {
  StaticJsonDocument<256> responseDoc;
  responseDoc["name"] = name;
  responseDoc["status"] = status;

  char responseBuffer[256];
  serializeJson(responseDoc, responseBuffer);

  client.publish(topic_pub, responseBuffer);
  LOG_DEBUG("Published status update: %s", responseBuffer);
}

void on_message(char* topic, byte* payload, unsigned int length) {
  LOG_INFO("Message received on topic: %s", topic);
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  LOG_DEBUG("Full message: %s", message.c_str());

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    LOG_ERROR("JSON parsing failed: %s", error.c_str());
    return;
  }

  if (!doc.containsKey("name") || !doc.containsKey("status")) {
    LOG_ERROR("Invalid JSON format: Missing fields");
    return;
  }

  const char* name = doc["name"];
  const char* status = doc["status"];

  LOG_INFO("Name: %s", name);
  LOG_INFO("Status: %s", status);

  update_status("rear_esp", default_status);
}

void setup() {
  Serial.begin(115200);
  LOG_INFO("SETUP: Initializing system");
  connectWiFi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(on_message);
  connectMQTT();
  LOG_INFO("SETUP COMPLETE. STARTING MAIN LOOP");
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();  // Non-blocking MQTT handling
}
