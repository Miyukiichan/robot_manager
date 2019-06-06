#include "robot_manager/Teleop.h"

/* The Teleop class periodically sends velocity messages to a given topic and allows the user
 * to change the velocities sent
 */

Teleop::Teleop(FileReader reader,ros::NodeHandle * nh) {
  struct teleop_info info = reader.getTeleopInfo();
  topic_name = info.topic;
  degradation = info.degradation;
  turning_increment = info.turning_increment;
  linear_increment = info.linear_increment;
  refresh_timeout = info.refresh_timeout;
  degradation_timeout = info.degradation_timeout;
  pub = nh->advertise<geometry_msgs::Twist>(topic_name,10);
  refresh_begin = ros::Time::now();
  degradation_begin = ros::Time::now();
  speed = 0;
  rotation = 0;
}

/* Sends a velocity message after the given delay has elapsed.
 * Also degrades the rotational velocity after its own delay so the robot automatically
 * straightens over time*/
void Teleop::refresh() {
  if (ros::Time::now() - degradation_begin > ros::Duration(degradation_timeout)) { //Rotational degradation repeater
    degradation_begin = ros::Time::now();
    /*Decreases/Increases rotation until it reaches 0*/
    if (rotation > 0)
      rotation = (rotation - degradation > 0) ? rotation-=degradation : rotation = 0;
    else if (rotation < 0)
      rotation = (rotation + degradation < 0) ? rotation+=degradation : rotation = 0;
    sendMessage();
  }
  if (ros::Time::now() - refresh_begin > ros::Duration(refresh_timeout)) { //General message repeater
    refresh_begin = ros::Time::now();
    sendMessage();
  }
}

/*If direction is true (1) then the rotation will be increased by the turning increment (clockwise).
 * Otherwise if it is 0 then it will decrease.
 * Pi was used as the maximum turning value, although that may not always be appropriate
 */
void Teleop::turn(int direction) {
  if (direction) {//right
    rotation = (rotation + turning_increment < M_PI) ? rotation + turning_increment : M_PI;
  }
  else { //left
    rotation = (rotation - turning_increment > -M_PI) ? rotation - turning_increment : -M_PI;
  }
}

/*If direction is true (1) then the linear speed will increase by the linear increment.
 * Otherwise it will decrease.
 * There is no maximum/minimum value*/
void Teleop::move(int direction) {
  speed = direction ? speed+=linear_increment : speed-=linear_increment;
}

/*Changes the linear and rotational velocities to 0 and sends a message with these values
 * 10 times. This is to ensure that the message is received for safety reasons*/
void Teleop::stop() {
  for (int i = 0; i < 10; i++) {
    speed = 0;
    rotation = 0;
    sendMessage();
  }
}

/*Sends geometry Twist messages using the current linear and angular velocities*/
void Teleop::sendMessage() {
    geometry_msgs::Twist msg;
    msg.linear.x = speed;
    msg.angular.z = rotation;
    pub.publish(msg);
}
