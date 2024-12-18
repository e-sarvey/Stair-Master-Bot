#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Adafruit_VL53L1X.h>

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

// Logging wrapper for funsises
    #define LOG_LEVEL_NONE 0
    #define LOG_LEVEL_ERROR 1
    #define LOG_LEVEL_INFO 2
    #define LOG_LEVEL_DEBUG 3

    int current_log_level = LOG_LEVEL_DEBUG; // set the logging level here

    #define LOG_ERROR(fmt, ...)   if (current_log_level >= LOG_LEVEL_ERROR) { Serial.printf("[ERROR]: " fmt "\n", ##__VA_ARGS__); }
    #define LOG_INFO(fmt, ...)    if (current_log_level >= LOG_LEVEL_INFO)  { Serial.printf("[INFO]: " fmt "\n", ##__VA_ARGS__); }
    #define LOG_DEBUG(fmt, ...)   if (current_log_level >= LOG_LEVEL_DEBUG) { Serial.printf("[DEBUG]: " fmt "\n", ##__VA_ARGS__); }


void moveStepper(int steps) {
    stepper.move(steps * MICROSTEPS);

    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}

void readLidar() {
    int16_t distance;

    LOG_INFO("LIDAR TAKING MEASUREMENT");

    // Check if data is ready
    if (vl53.dataReady()) {
        // Retrieve the distance measurement
        distance = vl53.distance();
        if (distance == -1) {
            LOG_ERROR("LIDAR measurement failed: Status %d", vl53.vl_status);
        }

        LOG_INFO("Distance (mm): %d", distance);

        // Clear the interrupt to prepare for the next reading
        vl53.clearInterrupt();
    } else {
        LOG_DEBUG("LIDAR data not ready yet");
    }
}

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
    readLidar();
    moveForward(1000);
    moveStepper(100);
}

void loop() {

}