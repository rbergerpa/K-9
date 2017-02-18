#include <Servo.h>
#include <Wire.h>

// Uncomment for Adafruit Motor Shield
#include <Adafruit_MotorShield.h>
# include "AFmotor.h"  

const int START_SONAR_PIN = 6;
const int LEFT_SONAR_PIN = 2;
const int CENTER_SONAR_PIN = 3;
const int RIGHT_SONAR_PIN = 4;

const int MIN_SIDE_DISTANCE = 36;  // inches
const int MIN_CENTER_DISTANCE = 14;
const int SERVO_CENTER = 1450; // microseconds

enum {Left, Right};

void setup() {
    initSerial();
    
    Serial.println("Initializing");
    initMotors();

    Serial.println("Ready");
}

void printDistances(int left, int center, int right) {
  Serial.print("Left: ");
  Serial.print(left);

  Serial.print("  Center: ");
  Serial.print(center);

  Serial.print("  Right: ");
  Serial.print(right);
  Serial.println();
}

int userSteering = 0;
int userSpeed = 0;

void loop() {
  int servo_ms;
  
  pollSerial();

  setMotorSpeed(Left, userSpeed + userSteering);
  setMotorSpeed(Right, userSpeed - userSteering);
}

String inString = "";

void initSerial() {
    inString.reserve(100);

    Serial.begin(115200);
}

void setSpeed(int speed) {
  Serial.print("Setting speed to ");
  Serial.println(speed);
  userSpeed = speed;
}

void setSteering(int steering) {
  Serial.print("Setting steering to ");
  Serial.println(steering);
  userSteering = steering;
}

void parseCommand(String &str) {  
  if (str.startsWith("speed=")) {
    setSpeed(str.substring(strlen("speed=")).toInt());
  } else   if (str.startsWith("steering=")) {
    setSteering(str.substring(strlen("steering=")).toInt());
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      parseCommand(inString);
      inString = "";
    } else {
      inString += ch;
    }
  }
}



