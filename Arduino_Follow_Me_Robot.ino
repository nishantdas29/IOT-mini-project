/*
 * Hand-Following Robot
 * 
 * This robot uses ultrasonic and IR sensors to detect and follow a hand.
 * When it detects a hand at an appropriate distance, it automatically follows it.
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - L293D Motor Driver Shield
 * - Ultrasonic Sensor (HC-SR04)
 * - 2x IR Sensors
 * - 4x TT Gear Motors
 * - SG90 Servo Motor (for scanning)
 * 
 * Connections:
 * - Ultrasonic: Trigger->A1, Echo->A0
 * - IR Sensors: Right->A2, Left->A3
 * - Motors connected through Motor Shield
 * - Servo: Signal->D10
 */

#include <NewPing.h>      // For ultrasonic sensor
#include <Servo.h>        // For servo motor
#include <AFMotor.h>      // For Adafruit/generic L293D motor shield

// Pin Definitions
#define TRIGGER_PIN  A1   // Ultrasonic sensor trigger pin
#define ECHO_PIN     A0   // Ultrasonic sensor echo pin
#define RIGHT_IR     A2   // Right IR sensor pin
#define LEFT_IR      A3   // Left IR sensor pin
#define SERVO_PIN    10   // Servo motor pin

// Constants
#define MAX_DISTANCE     100  // Maximum ultrasonic sensing distance (cm)
#define MIN_DISTANCE     10   // Minimum follow distance (too close, stop)
#define OPTIMAL_DISTANCE 20   // Target distance to maintain from hand
#define MAX_FOLLOW       50   // Maximum distance to start following
#define MOTOR_SPEED      255  // Base motor speed (0-255)
#define TURN_SPEED       255  // Speed when turning

// Setup objects for sensors and motors
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo scanServo;

// Create motor objects - using all 4 channels
AF_DCMotor frontRight(1, MOTOR12_1KHZ); // Front right motor on M1
AF_DCMotor frontLeft(2, MOTOR12_1KHZ);  // Front left motor on M2
AF_DCMotor backRight(3, MOTOR34_1KHZ);  // Back right motor on M3
AF_DCMotor backLeft(4, MOTOR34_1KHZ);   // Back left motor on M4

// Variables
unsigned int distance = 0;    // Distance from ultrasonic
bool rightDetected = false;   // IR sensor detection flags
bool leftDetected = false;
unsigned long lastScanTime = 0;  // For periodic scanning when idle

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Hand-Following Robot Initializing...");
  
  // Initialize IR sensors as inputs
  pinMode(RIGHT_IR, INPUT);
  pinMode(LEFT_IR, INPUT);
  
  // Initialize servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(90); // Center position
  
  // Initialize motors (start with stopped)
  frontRight.setSpeed(0);
  frontLeft.setSpeed(0);
  backRight.setSpeed(0);
  backLeft.setSpeed(0);
  
  stopMotors();
  
  // Initial scan to show the robot is ready
  scanForHand();
  
  Serial.println("Robot Ready!");
  delay(1000);
}

void loop() {
  // Read sensors
  readSensors();
  
  // Main decision logic for following hand
  if (distance > 0) { // If distance is 0, sensor detected nothing
    if (distance < MIN_DISTANCE) {
      // Hand too close - stop
      Serial.println("Hand too close - stopping");
      stopMotors();
    }
    else if (distance <= MAX_FOLLOW) {
      // Hand in follow range - determine direction
      if (!rightDetected && leftDetected) {
        // Hand detected on left side
        Serial.println("Hand on left - turning left");
        turnLeft();
      }
      else if (rightDetected && !leftDetected) {
        // Hand detected on right side
        Serial.println("Hand on right - turning right");
        turnRight();
      }
      else {
        // Hand centered or both IR sensors triggered
        // Adjust speed based on distance
        if (distance < OPTIMAL_DISTANCE) {
          // Slightly closer than optimal - slow down
          Serial.println("Hand centered but close - moving slowly");
          moveForward(MOTOR_SPEED - 50);
        }
        else if (distance > OPTIMAL_DISTANCE + 10) {
          // Further than optimal - speed up
          Serial.println("Hand centered but far - moving faster");
          moveForward(MOTOR_SPEED + 30);
        }
        else {
          // At optimal distance - maintain speed
          Serial.println("Hand centered at optimal distance - following");
          moveForward(MOTOR_SPEED);
        }
      }
    }
    else {
      // Hand too far away - stop and scan
      Serial.println("Hand too far - stopping");
      stopMotors();
      
      // Periodically scan when idle
      if (millis() - lastScanTime > 3000) { // Every 3 seconds
        scanForHand();
        lastScanTime = millis();
      }
    }
  }
  else {
    // No hand detected - stop and scan
    Serial.println("No hand detected - stopping");
    stopMotors();
    
    // Periodically scan when idle
    if (millis() - lastScanTime > 3000) { // Every 3 seconds
      scanForHand();
      lastScanTime = millis();
    }
  }
  
  // Small delay for stability
  delay(50);
}

void readSensors() {
  // Read ultrasonic sensor (in cm)
  distance = sonar.ping_cm();
  
  // Read IR sensors (typically LOW when object detected, HIGH when nothing detected)
  // But this might be reversed for some sensors - adjust as needed
  rightDetected = (digitalRead(RIGHT_IR) == LOW);
  leftDetected = (digitalRead(LEFT_IR) == LOW);
  
  // Debug output
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("cm | Right IR: ");
  Serial.print(rightDetected ? "Detected" : "None");
  Serial.print(" | Left IR: ");
  Serial.println(leftDetected ? "Detected" : "None");
}

void moveForward(int speed) {
  frontRight.setSpeed(speed);
  frontLeft.setSpeed(speed);
  backRight.setSpeed(speed);
  backLeft.setSpeed(speed);
  
  frontRight.run(FORWARD);
  frontLeft.run(FORWARD);
  backRight.run(FORWARD);
  backLeft.run(FORWARD);
}

void turnRight() {
  frontRight.setSpeed(TURN_SPEED);
  frontLeft.setSpeed(TURN_SPEED);
  backRight.setSpeed(TURN_SPEED);
  backLeft.setSpeed(TURN_SPEED);
  
  frontRight.run(BACKWARD);
  frontLeft.run(FORWARD);
  backRight.run(BACKWARD);
  backLeft.run(FORWARD);
}

void turnLeft() {
  frontRight.setSpeed(TURN_SPEED);
  frontLeft.setSpeed(TURN_SPEED);
  backRight.setSpeed(TURN_SPEED);
  backLeft.setSpeed(TURN_SPEED);
  
  frontRight.run(FORWARD);
  frontLeft.run(BACKWARD);
  backRight.run(FORWARD);
  backLeft.run(BACKWARD);
}

void stopMotors() {
  frontRight.run(RELEASE);
  frontLeft.run(RELEASE);
  backRight.run(RELEASE);
  backLeft.run(RELEASE);
}

void scanForHand() {
  Serial.println("Scanning for hand...");
  
  // Scan from center to right
  for (int pos = 90; pos <= 180; pos += 5) {
    scanServo.write(pos);
    delay(30);
    
    // Check for hand during scan
    if (sonar.ping_cm() > 0 && sonar.ping_cm() < MAX_FOLLOW) {
      Serial.println("Hand detected during scan!");
      scanServo.write(90); // Return to center
      return;
    }
  }
  
  // Scan from right to left
  for (int pos = 180; pos >= 0; pos -= 5) {
    scanServo.write(pos);
    delay(30);
    
    // Check for hand during scan
    if (sonar.ping_cm() > 0 && sonar.ping_cm() < MAX_FOLLOW) {
      Serial.println("Hand detected during scan!");
      scanServo.write(90); // Return to center
      return;
    }
  }
  
  // Return to center
  scanServo.write(90);
  lastScanTime = millis(); // Update scan time
}