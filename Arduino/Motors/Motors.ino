#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Uncomment for H-Bridge motors
//#include "HBmotor.h"

// Uncomment for Adafruit Motor Shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
# include "AFmotorsWrapper.h"
  
enum {Left, Right};

ros::NodeHandle  nh;

void MotorsCb( const std_msgs::Float32MultiArray& msg) {
  char str[30];
  float speed = msg.data[0];
  float steering = msg.data[1];
 sprintf(str, "speed %d steering %d", ((int) (speed*100.0)), ((int) (steering*100.0)));
  nh.loginfo(str);

  setMotorSpeed(Left, speed + steering);
  setMotorSpeed(Right, speed - steering);
}

ros::Subscriber<std_msgs::Float32MultiArray> motorsSub("/motors", &MotorsCb );

void setup() {
  Serial.begin(57600);
  initMotors();
      
  nh.initNode();
  nh.loginfo("Starting");
  nh.subscribe(motorsSub);
}

void loop() {
  nh.spinOnce();
}
