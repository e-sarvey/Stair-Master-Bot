#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Adafruit_VL53L1X.h>

/*
this program is set up for the FRONT ESP to subscribe to the rear ESP. 
This program handles the following:
    - Controls counterweight stepper
    - Control rear motor speeds
    - Measures Lidar for distance to step
    - Communicates status with rear ESP
*/ 

// Hardware Parameters and Pin Setup
    // dc motors
    #define MOTOR1_IN1  26  // Front-left motor
    #define MOTOR1_IN2  25
    #define MOTOR2_IN1  33  // Front-right motor
    #define MOTOR2_IN2  32

    // stepper
    #define STEP_PIN  17  
    #define DIR_PIN   16  
    #define EN_PIN 4
    #define MICROSTEPS 16  // Microstepping factor (1/16)
    #define STEPS_PER_REV 200  // Steps per revolution for a standard stepper motor
    #define MASS_FOR_STEPS 2000
    #define MASS_BACK_STEPS 2000
// Create Stepper Motor Instance
    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Create LiDAR instance
    #define XSHUT_PIN 15
    #define IRQ_PIN 2
    #define TARGET_DISTANCE_MM 3 // Target lidar distance in millimeters
    Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
   
// Wifi and MQTT parameters
    const char* ssid = "Tufts_Robot";
    const char* password = "";
    const char* mqtt_broker = "10.243.82.33";
    const int mqtt_port = 1883;
    // these will be different between front and rear
    const char* topic_rear_sub = "status/rear";
    const char* topic_pub = "status/front";
    const char* mqtt_client_id = "front_esp";
    const char* default_status = "front esp message test"; // testing only

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

String latest_name = "";
String latest_status = "";

// ------------ // WIFI/MQTT FUNCTIONS // ------------ //
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
            client.subscribe(topic_rear_sub);  // Subscribe to the topic
        } 
        else {
            LOG_ERROR("MQTT connection failed, retrying in 5 seconds...");
            delay(5000);
        }
    }

    LOG_INFO("MQTT Client connected as %s", mqtt_client_id);
    LOG_INFO("Subscribed to topic: %s", topic_rear_sub);
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

    // Store the latest received values globally
    latest_name = doc["name"].as<String>();
    latest_status = doc["status"].as<String>();

    LOG_INFO("Latest Name: %s", latest_name.c_str());
    LOG_INFO("Latest Status: %s", latest_status.c_str());
}


bool await_message(const char* expected_name, const char* expected_status) {
    LOG_INFO("Awaiting message from: %s with status: %s", expected_name, expected_status);

    unsigned long start_time = millis();
    unsigned long elapsed_time = 0;  // Track elapsed time

    while (true) {
        client.loop();  // Process incoming MQTT messages

        // Compare the global variables with expected values
        if (latest_status == expected_status && latest_name == expected_name) {
            LOG_INFO("Correct message received: Name=%s, Status=%s", latest_name.c_str(), latest_status.c_str());
            latest_name = "";   // Reset global variables after use
            latest_status = "";
            return true;
        }

        // Update and print elapsed time every second
        unsigned long current_time = millis();
        if (current_time - start_time >= (elapsed_time + 1000)) {
            elapsed_time = (current_time - start_time);
            LOG_DEBUG("Waiting... Elapsed time: %lu seconds", elapsed_time / 1000);
        }

        delay(10);  // Small delay to prevent CPU hogging
    }
}
// ------------ // OTHER FUNCTIONS // ------------ //
int readLidar() {
    int16_t distance;

    LOG_INFO("LIDAR TAKING MEASUREMENT");

    // Check if data is ready
    if (vl53.dataReady()) {
        // Retrieve the distance measurement
        distance = vl53.distance();
        if (distance == -1) {
            LOG_ERROR("LIDAR measurement failed: Status %d", vl53.vl_status);
            return 0;
        }

        LOG_INFO("Distance (mm): %d", distance);

        // Clear the interrupt to prepare for the next reading
        vl53.clearInterrupt();
        return distance;
    } else {
        LOG_DEBUG("LIDAR data not ready yet");
        return -1;  // Indicate no new data available
    }
}

// ------------ // MOTOR/STEPPER FUNCTIONS // ------------ //
void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
}

void motorOn(int motor1_in1, int motor1_in2, int motor2_in1, int motor2_in2) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
}

void moveForward(int move_time) {
  motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2); // Front motors
  delay(move_time); // Run for defined time
  stopMotors();
}

void moveStepper(int steps) {
  stepper.move(steps * MICROSTEPS);

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

// ------------ // MAIN FUNCTIONS // ------------ //
void move_one_stair() {
    // move motors forward until lidar threshold met
    motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2); // Turn on front motors
    while (readLidar() > TARGET_DISTANCE_MM) {
        // Keep moving forward until the LiDAR distance is 3mm or less
        delay(10); 
    }
    stopMotors(); // Stop the front motors when the target distance is reached
    update_status("front","stair_reached");
    // rear moves to step and lifts
    await_message("rear", "front_placed");
    moveStepper(MASS_BACK_STEPS);
    update_status("front","mass_moved");

    await_message("rear", "rear_lifted");

    motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2);
        while (readLidar() > TARGET_DISTANCE_MM*2) {
        // Keep moving forward until the LiDAR distance is 3mm or less
        delay(10); 
    }
    moveStepper(MASS_FOR_STEPS);

    
    

}

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

    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable the motor driver

    stepper.setMaxSpeed(1000);  // Set maximum speed (steps per second)
    stepper.setAcceleration(500);  // Set acceleration (steps per second^2)

    stopMotors();

    // LIDAR Initialization
    Wire.begin();
    if (!vl53.begin(0x29, &Wire)) {  // Default I2C address for VL53L1X
        LOG_ERROR("Failed to initialize VL53L1X. Status: %d", vl53.vl_status);
        while (1) delay(10);
    }
    LOG_INFO("VL53L1X initialized successfully!");

    LOG_INFO("Sensor ID: 0x%X", vl53.sensorID());

    // Start ranging
    if (!vl53.startRanging()) {
        LOG_ERROR("Couldn't start ranging: Status %d", vl53.vl_status);
        while (1) delay(10);
    }
    LOG_INFO("Ranging started");

    // Set the timing budget (Valid values: 15, 20, 33, 50, 100, 200, 500 ms)
    vl53.setTimingBudget(50);
    LOG_INFO("Timing budget set to: %d ms", vl53.getTimingBudget());

    // WIFI SETUP
    connectWiFi();
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(on_message);
    connectMQTT();
    LOG_INFO("SETUP COMPLETE. AWAITING ACTIVATION MESSAGE");
    await_message("activation", "go");

    for (int i = 0; i < 8; i++) {
        move_one_stair();
        delay(500);
    }
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();  // Non-blocking MQTT handling
}
