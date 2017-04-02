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

const int START_SONAR_PIN = 4;
const int LEFT_SONAR_PIN = 10;
const int CENTER_SONAR_PIN = 11;
const int RIGHT_SONAR_PIN = 12;

Sonar leftSonar(LEFT_SONAR_PIN);
Sonar centerSonar(CENTER_SONAR_PIN);
Sonar rightSonar(RIGHT_SONAR_PIN);

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
  nh.logdebug(str);
  
  sprintf(str, "speed %d steering %d", ((int) (speed*100.0)), ((int) (steering*100.0)));
  nh.logdebug(str);

  outputSpeedAndSteering(speed, steering);
}

void outputSpeedAndSteering(float speed, float steering) {
  setMotorSpeed(Left, speed + steering);
  setMotorSpeed(Right, speed - steering);
}

void setup() {
  Serial.begin(57600);
  initMotors();
      
  nh.initNode();
  nh.loginfo("Starting");
  nh.subscribe(motorsSub);
}

void loop() {
  avoidObstacles();

  nh.spinOnce();
  delay(100);

}
