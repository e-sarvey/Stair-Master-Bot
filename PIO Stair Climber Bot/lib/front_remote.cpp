#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <Adafruit_VL53L1X.h>
#include <ArduinoJson.h>
// Hardware Parameters and Pin Setup
// DC motors
#define MOTOR1_IN1  26  // Front-left motor
#define MOTOR1_IN2  25
#define MOTOR2_IN1  33  // Front-right motor
#define MOTOR2_IN2  32
// Stepper motor
#define STEP_PIN  17
#define DIR_PIN   16
#define EN_PIN 4
#define MICROSTEPS 16
#define STEPS_PER_REV 200
// LiDAR
#define XSHUT_PIN 15
#define IRQ_PIN 2
#define TARGET_DISTANCE_MM 3
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
// Stepper motor instance
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
// Wi-Fi and MQTT parameters
const char* ssid = "Tufts_Robot";
const char* password = "";
const char* mqtt_broker = "10.243.82.33";
const int mqtt_port = 1883;
const char* mqtt_client_id = "front_esp_client";
const char* motors_topic = "hardware/motors/front";
const char* stepper_topic = "hardware/stepper/front"; // testing only
WiFiClient espClient;
PubSubClient client(espClient);
// Logging wrapper for funsises
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3
int current_log_level = LOG_LEVEL_DEBUG; // set the logging level here
#define LOG_ERROR(fmt, ...)   if (current_log_level >= LOG_LEVEL_ERROR) { Serial.printf("[ERROR]: " fmt "\n", ##__VA_ARGS__); }
#define LOG_INFO(fmt, ...)    if (current_log_level >= LOG_LEVEL_INFO)  { Serial.printf("[INFO]: " fmt "\n", ##__VA_ARGS__); }
#define LOG_DEBUG(fmt, ...)   if (current_log_level >= LOG_LEVEL_DEBUG) { Serial.printf("[DEBUG]: " fmt "\n", ##__VA_ARGS__); }

// Function to initialize Wi-Fi
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
// Function to initialize MQTT
void connectMQTT() {
    LOG_INFO("Attempting Connection to MQTT Server...");
    while (!client.connected()) {
        if (client.connect(mqtt_client_id)) {
            client.subscribe(motors_topic);  // Subscribe to the topic
            delay(0.5);
            client.subscribe(stepper_topic);
        }
        else {
            LOG_ERROR("MQTT connection failed, retrying in 5 seconds...");
            delay(5000);
        }
    }
    LOG_INFO("MQTT Client connected as %s", mqtt_client_id);
    LOG_INFO("Subscribed to topic: %s", motors_topic);
    LOG_INFO("Subscribed to topic: %s", stepper_topic);
}
// Function to update status
// void update_status(const char* name, const char* status) {
//     StaticJsonDocument<256> responseDoc;
//     responseDoc["name"] = name;
//     responseDoc["status"] = status;
//     char responseBuffer[256];
//     serializeJson(responseDoc, responseBuffer);
//     client.publish(topic_pub, responseBuffer);
//     LOG_DEBUG("Published status update: %s", responseBuffer);
// }
// void on_message(char* topic, byte* payload, unsigned int length) {
//     LOG_INFO("Message received on topic: %s", topic);
//     String message;
//     for (unsigned int i = 0; i < length; i++) {
//         message += (char)payload[i];
//     }
//     LOG_DEBUG("Full message: %s", message.c_str());
//     StaticJsonDocument<256> doc;
//     DeserializationError error = deserializeJson(doc, message);
//     if (error) {
//         LOG_ERROR("JSON parsing failed: %s", error.c_str());
//         return;
//     }
//     if (!doc.containsKey("name") || !doc.containsKey("status")) {
//         LOG_ERROR("Invalid JSON format: Missing fields");
//         return;
//     }
//     // Store the latest received values globally
//     latest_name = doc["name"].as<String>();
//     latest_status = doc["status"].as<String>();
//     LOG_INFO("Latest Name: %s", latest_name.c_str());
//     LOG_INFO("Latest Status: %s", latest_status.c_str());
// }
// Function to control the DC motors (turn on and off based on JSON payload)
// void controlDCMotors(String payload) {
//     StaticJsonDocument<200> doc;
//     DeserializationError error = deserializeJson(doc, payload);
//     if (error) {
//         Serial.println("Failed to parse JSON");
//         return;
//     }
//     int motor1_speed = doc["motor1"].as<int>();
//     int motor2_speed = doc["motor2"].as<int>();
//     analogWrite(MOTOR1_IN1, max(0, motor1_speed));
//     analogWrite(MOTOR1_IN2, max(0, -motor1_speed));
//     analogWrite(MOTOR2_IN1, max(0, motor2_speed));
//     analogWrite(MOTOR2_IN2, max(0, -motor2_speed));
// }
// Function to stop motors
void stopMotors() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
}
// Function to turn motors on
void motorOn(int motor1_in1, int motor1_in2, int motor2_in1, int motor2_in2) {
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in1, HIGH);
    digitalWrite(motor2_in2, LOW);
}
void moveStepper(String payload) {
  // Parse the payload (assumed to be a numeric string representing steps)
  int steps = payload.toInt();  // Convert the string to an integer
  if (steps != 0) {
    stepper.move(steps * MICROSTEPS);  // Move stepper by the number of steps (considering microstepping)
    // Run the stepper until the target position is reached
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    LOG_INFO("Stepper moved %d steps", steps);
  } else {
    LOG_ERROR("Invalid step count in payload: %s", payload.c_str());
  }
}
// Function to handle incoming MQTT messages
void callback(char* topic, byte* message, unsigned int length) {
    String payload;
    for (unsigned int i = 0; i < length; i++) {
        payload += (char)message[i];
    }
    Serial.print("Message received on topic: ");
    Serial.println(topic);
    Serial.print("Payload: ");
    Serial.println(payload);
    if (String(topic) == motors_topic) {
        // Turn motors on or off based on command
        if (payload == "on") {
            motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2);  // Motor on
        } else if (payload == "off") {
            stopMotors();  // Motor off
        }
    } else if (String(topic) == stepper_topic) {
        // Stepper control: move based on payload steps
        moveStepper(payload);
    } //else if (String(topic) == "hardware/lidar") {
        // controlLiDAR(payload);
    //}
}
// Main setup function
void setup() {
    Serial.begin(115200);
    unsigned long serial_start_time = millis();
    while (!Serial) {
        delay(100);  // Small delay to wait
        if (millis() - serial_start_time > 2000) {  // Timeout: 2 seconds
            LOG_ERROR("Serial not connected. Restarting setup...");
            ESP.restart();  // Restart ESP32 to attempt Serial connection again
        }
    }
    LOG_INFO("SETUP: Initializing system");
    Wire.begin();
    Wire.setClock(400000);
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable the motor driver
    stepper.setMaxSpeed(1000);  // Set maximum speed (steps per second)
    stepper.setAcceleration(500);  // Set acceleration (steps per second^2)
    stopMotors();
    // WIFI SETUP
    connectWiFi();
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
}
// Main loop
void loop() {
    if (!client.connected()) {
        connectMQTT();
    }
    client.loop();
}