#include <AccelStepper.h>
#include "Adafruit_VL53L0X.h"

// Pin Definitions
// H-Bridge 1
#define MOTOR1_IN1  26  // Front-left motor
#define MOTOR1_IN2  25
#define MOTOR2_IN1  33  // Front-right motor
#define MOTOR2_IN2  32

// Initialize steppers
#define STEPPER1_STEP  17  // Step pin for stepper 1
#define STEPPER1_DIR   16  // Direction pin for stepper 1
#define ENABLE_PIN 4
#define MICROSTEPS 16  // Microstepping factor (1/16)
#define STEPS_PER_REV 200  // Steps per revolution for a standard stepper motor
#define TARGET_DISTANCE_MM 3 // Target distance in millimeters

// Create Stepper Motor Instances
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP, STEPPER1_DIR);

// Create LiDAR instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Steps
#define FRONT_LIFT_STEPS 3000
#define BACK_LIFT_STEPS -3000
#define FRONT_LOWER_STEPS -500
#define BACK_LOWER_STEPS 500
#define MASS_FOR_STEPS 2000
#define MASS_BACK_STEPS 2000

// Timing
#define FORWARD_TIME 500
#define DELAY_TIME 100

// Helper Functions
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
  stepper1.move(steps * MICROSTEPS);

  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
}

// Function to calculate time to take a specific number of steps
unsigned long calculateTimeForSteps(int numSteps) {
  // Calculate the number of steps per second
  float stepsPerSecond = 1000.0;

  // Calculate the time required for the number of steps in milliseconds
  unsigned long timeMs = (numSteps / stepsPerSecond) * 1000;

  return timeMs;
}

int readLidar() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
    return measure.RangeMilliMeter;
  } else {
    Serial.println(" out of range ");
    return 0;
  }
}

// FRONT ESP
void moveOneStair() {
  // Step 1: Move front motors forward
  motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2); // Turn on front motors
  while (readLidar() > TARGET_DISTANCE_MM) {
    // Keep moving forward until the LiDAR distance is 3mm or less
    delay(10); // Small delay to avoid overwhelming the sensor
  }
  stopMotors(); // Stop the front motors when the target distance is reached

  // Move mass backward
  moveStepper1(MASS_BACK_STEPS);
  delay(calculateTimeForSteps(FRONT_LIFT_STEPS - MASS_BACK_STEPS)); // Pause for front to be lifted
  delay(FORWARD_TIME); // Pause for back wheels forward
  delay(calculateTimeForSteps(FRONT_LOWER_STEPS)); // Pause for front to be lowered

  // Move mass forward
  moveStepper1(MASS_FOR_STEPS);
  delay(calculateTimeForSteps(BACK_LIFT_STEPS - MASS_FOR_STEPS)); // Pause for back to be lifted
}

void setup() {
  Serial.begin(115200);

  // Initialize all motor pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the motor driver

  // Configure the stepper motor
  stepper1.setMaxSpeed(1000);  // Set maximum speed (steps per second)
  stepper1.setAcceleration(500);  // Set acceleration (steps per second^2)

  // Stop all motors initially
  stopMotors();

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

  for (int i = 0; i < 8; i++) {
    moveOneStair();
    delay(500);
  }
}

void loop() {
  // Empty loop
}

