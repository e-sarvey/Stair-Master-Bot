#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Wire.h>

/*
this program is set up for the rear ESP to subscribe to the front ESP and camera. 
This program handles the following:
    - Recieves step info from camera (or camera via central PC)
    - Controls z-axis stepper
    - Control rear motor speeds
    - Measures IMU for fall detection
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

    #define MPU 0x68  // Accelerometer address

// Create Stepper Motor Instance
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// steps and timing for hardcoding
    #define FRONT_LIFT_STEPS 3000
    #define BACK_LIFT_STEPS -3000
    #define FRONT_LOWER_STEPS 500
    #define BACK_LOWER_STEPS -500
    #define MASS_FOR_STEPS 2000
    #define MASS_BACK_STEPS 2000

    #define FORWARD_TIME 500
    #define DELAY_TIME 100

// Wifi and MQTT parameters
    const char* ssid = "Tufts_Robot";
    const char* password = "";
    const char* mqtt_broker = "10.243.82.33";
    const int mqtt_port = 1883;
    // these will be different between front and rear
    const char* topic_front_sub = "status/front";
    const char* topic_cam_sub = "status/camera";
    const char* topic_pub = "status/rear";
    const char* mqtt_client_id = "rear_esp";
    const char* default_status = "rear esp message test"; // testing only

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

    // Store the latest received values globally
    latest_name = doc["name"].as<String>();
    latest_status = doc["status"].as<String>();

    LOG_INFO("Latest Name: %s", latest_name.c_str());
    LOG_INFO("Latest Status: %s", latest_status.c_str());
}


bool await_message(const char* expected_name, const char* expected_status) {
    LOG_INFO("Awaiting message from: %s with status: %s", expected_name, expected_status);
    unsigned long start_time = millis();
    const unsigned long timeout = 10000;  // Timeout in milliseconds (10 seconds)

    while (millis() - start_time < timeout) {
        client.loop();  // Process incoming MQTT messages

        // Compare the global variables with expected values
        if (latest_status == expected_status && latest_name == expected_name) {
            LOG_INFO("Correct message received: Name=%s, Status=%s", latest_name.c_str(), latest_status.c_str());
            latest_name = "";   // Reset global variables after use
            latest_status = "";
            return true;
        }
        delay(10);  // Small delay to prevent CPU hogging
    }

    LOG_ERROR("Timeout waiting for message from: %s", expected_name);
    return false;  // Timeout occurred
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

void moveStepper1(int steps) {
  stepper.move(steps * MICROSTEPS);

  while (stepper.distanceToGo() != 0 && !detectTilt()) {
    stepper.run();
  }
}
// ------------ // OTHER FUNCTIONS // ------------ //
void setup_accel() {
    // Initialize MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);
  delay(100);

  // Configure accelerometer sensitivity (optional)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  // Accelerometer config register
  Wire.write(0x00);  // Set to ±2g
  Wire.endTransmission(true);

  Serial.println("MPU6050 initialized");
}

bool detectTilt(){
    int16_t ax, ay, az;
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);
    ax = (Wire.read() << 8 | Wire.read());
    ay = (Wire.read() << 8 | Wire.read());
    az = (Wire.read() << 8 | Wire.read());

     // Calculate acceleration magnitude
    float vertical_comp = az / 16384.0;

    // Display acceleration
    //Serial.print("Acceleration Magnitude: ");
    //Serial.println(accelMagnitude);

    // Tap detection with threshold
    if (vertical_comp < 0.5) {  // Adjust threshold as necessary
        Serial.println("Tilted!");
        return true;
    }
    else{
        return false;
    }
}

// ------------ // MAIN FUNCTIONS // ------------ //
void move_one_stair(){
    //WAIT FOR FRONT TO REACH STAIR

    // Lift front
    moveStepper1(FRONT_LIFT_STEPS);

    // Move back motors forward
    moveForward(FORWARD_TIME);

    //TELL FRONT DONE LIFTING AND MOVING FORWARD

    // Lower front
    moveStepper1(FRONT_LOWER_STEPS);

    //WAIT FOR FRONT TO FINISH MOVING MASS

    // Lift back
    moveStepper1(BACK_LIFT_STEPS);

    //TELL FRONT DONE LIFTING

}

void setup() {
    Serial.begin(115200);
    LOG_INFO("SETUP: Initializing system");
    Wire.begin();
    Wire.setClock(400000);

    setup_accel();

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
    client.setCallback(on_message);
    connectMQTT();
    LOG_INFO("SETUP COMPLETE. STARTING MAIN LOOP");

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
