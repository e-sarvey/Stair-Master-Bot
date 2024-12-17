#include <AccelStepper.h>

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

#define MPU 0x68  // Accelerometer

// Create Stepper Motor Instances
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP, STEPPER1_DIR);

// Steps
#define FRONT_LIFT_STEPS 3000
#define BACK_LIFT_STEPS -3000
#define FRONT_LOWER_STEPS 500
#define BACK_LOWER_STEPS -500
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

  while (stepper1.distanceToGo() != 0 && !detectTilt()) {
    stepper1.run();
  }
}

void setupAccel() {
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
  ax = (Wire.read() << 8 | Wire.read());
  ay = (Wire.read() << 8 | Wire.read());
  az = (Wire.read() << 8 | Wire.read());

  // Calculate acceleration magnitude
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az) / 16384.0;

  // Display acceleration
  //Serial.print("Acceleration Magnitude: ");
  //Serial.println(accelMagnitude);

  // Tap detection with threshold
  if (accelMagnitude > 5) {  // Adjust threshold as necessary
    Serial.println("Tilted!");
    return True
  }
  else{
    return False
  }
}

// BACK ESP
void moveOneStair() {

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
  Wire.begin();
  Wire.setClock(400000);  // I2C speed

  // Initialize accelerometer
  setupAccel();
  
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

  //WAIT FOR MESSAGE FROM CAMERA WITH HEIGHT
  //STEPS TO RAISE = HEIGHT(MM)*100*1.05
  
  for (int i = 0; i < 8; i++) {
    moveOneStair();
    delay(500);
  }
}

void loop() {
  // Empty loop
}
