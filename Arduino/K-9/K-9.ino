// Software for K-9
// Accepts desired speed and steering via ROS messages
// Overrides commanded speed and steeering to avoid obstacles, based on sonar data

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <PciManager.h>
#include <Sonar.h>

// Uncomment for H-Bridge motors
//#include "HBmotor.h"

// Uncomment for Adafruit Motor Shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
# include "AFmotorsWrapper.h"
  
enum {Left, Right};

const int START_SONAR_PIN = 9;
const int LEFT_SONAR_PIN = 10;
const int CENTER_SONAR_PIN = 11;
const int RIGHT_SONAR_PIN = 12;

Sonar leftSonar(LEFT_SONAR_PIN);
Sonar centerSonar(CENTER_SONAR_PIN);
Sonar rightSonar(RIGHT_SONAR_PIN);

const int MIN_SIDE_DISTANCE = 16;  // inches
const int MIN_CENTER_DISTANCE = 10;
const float MAX_STEERING = 0.8; // During obstacle avoidance
const float STEERING_GAIN = 0.05;

ros::NodeHandle  nh;

float userSpeed;
float userSteering;

// Called when a message is received on the "motors" topic
void MotorsCallback( const std_msgs::Float32MultiArray& msg) {
  char str[30];

   userSpeed = msg.data[0];
   userSteering = msg.data[1];
 
  sprintf(str, "userSpeed %d userSteering %d", ((int) (userSpeed*100.0)), ((int) (userSteering*100.0)));
  nh.loginfo(str);
}

ros::Subscriber<std_msgs::Float32MultiArray> motorsSub("/motors", &MotorsCallback );

void avoidObstacles() {
  char str[30];
  float speed = userSpeed;
  float steering = userSteering;

  int left = leftSonar.inches();
  int center = centerSonar.inches();
  int right = rightSonar.inches();
  sprintf(str, "left %d center %d right %d", left, center, right);
  nh.loginfo(str);

  if (center < MIN_CENTER_DISTANCE) {
    turnAround(left, center, right);
    return;
  }

  if (left < MIN_SIDE_DISTANCE || right < MIN_SIDE_DISTANCE) {
    steering = STEERING_GAIN * (right - left);
    if (steering < -MAX_STEERING) {
      steering = -MAX_STEERING;
    } else if (steering > MAX_STEERING) {
      steering = MAX_STEERING;
    }
 }

  outputSpeedAndSteering(speed, steering);
} // avoidObstacles

void turnAround(float left, float center, float right) {
  nh.loginfo("turnAround");
  float steering = 0.0;

  outputSpeedAndSteering(0.0, 0.0);
  delay(250);
  
  if (right > left) {
    steering =  MAX_STEERING;
  } else {
    steering = - MAX_STEERING;
  }

  outputSpeedAndSteering(0.0, steering);
  delay(1000);

  outputSpeedAndSteering(userSpeed, 0.0);
} // turnAround

void outputSpeedAndSteering(float speed, float steering) {
  char str[50];
  
  printf(str, "outputSpeedAndSteering speed %d steering %d", ((int) (speed*100.0)), ((int) (steering*100.0)));
  nh.loginfo(str);
 
  setMotorSpeed(Left, speed + steering);
  setMotorSpeed(Right, speed - steering);
} // outputSpeedAndSteering

void setup() {
  Serial.begin(57600);
  initMotors();
  startSonar(START_SONAR_PIN);
      
  nh.initNode();
  nh.loginfo("Starting");
  nh.subscribe(motorsSub);
} // setup

void loop() {
  avoidObstacles();

  nh.spinOnce();
  delay(100);
} // loop
