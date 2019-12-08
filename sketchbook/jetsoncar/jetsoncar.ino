/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a FRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h> 

ros::NodeHandle  nodeHandle;
// These are general bounds for the steering servo and the
// FRAXXAS Electronic Speed Controller (ESC)
const int minSteering = 30 ;
const int maxSteering = 150 ;
const int minThrottle = 0 ;
const int maxThrottle = 150 ;

Servo FL;
Servo FR;
Servo RL;
Servo RR;
#define PIN_FL 9
#define PIN_FR 10
#define PIN_RL 8
#define PIN_RR 11
#define SPEED_0 1500

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the FRAXXAS works like a Servo

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// PWM VALUES SHOULD ALWAYS STAY BETWEEN 1000 AND 2000
int convertTwist2Talon (double twist, bool inverse) {
  int ret;
  if (inverse) {
    ret = twist * -200;
  } else {
    ret = twist * 200;
  }
  return ret + 1500;
}


void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  int steeringAngle = fmap(twistMsg.angular.z, 0.0, 1.0, minSteering, maxSteering) ;
  // The following could be useful for debugging
  // str_msg.data= steeringAngle ;
  // chatter.publish(&str_msg);
  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) { 
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }
  //steeringServo.write(steeringAngle) ;

  int vel = convertTwist2Talon(twistMsg.linear.x, false);
  int invel = convertTwist2Talon(twistMsg.linear.x, true);
  int diff;
 
  if (abs((vel-1500) > 100)) {
    diff = twistMsg.angular.z * 150;
  } else {
    diff = twistMsg.angular.z * 200;
  }

  vel += diff;
  invel += diff;
  
  FL.write(invel);
  FR.write(vel);
  RL.write(invel);
  RR.write(vel);
  str_msg.data = diff;
  chatter.publish(&str_msg);
  
 
  // The following could be useful for debugging
  // str_msg.data= escCommand ;
  // chatter.publish(&str_msg);
  
  //electronicSpeedController.write(escCommand) ;
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
 
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/jetsoncar_teleop_joystick/cmd_vel", &driveCallback) ;

void setup(){
  Serial.begin(57600);
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  FL.attach(PIN_FL);
  FR.attach(PIN_FR);
  RL.attach(PIN_RL);
  RR.attach(PIN_RR);
  FL.write(SPEED_0);
  FR.write(SPEED_0);
  RL.write(SPEED_0);
  RR.write(SPEED_0);
  delay(1000) ;
  
}

void loop(){
  nodeHandle.spinOnce();
  delay(1);
}
