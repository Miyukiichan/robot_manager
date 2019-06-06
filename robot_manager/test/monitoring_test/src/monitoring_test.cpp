#include <ros/ros.h>
#include <stdio.h>
#include "monitoring_test/SafetyStatus.h"

int main() {
  int argc = 0;
  char **argv = NULL;
  ros::init(argc,argv,"monitoring_test");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<monitoring_test::SafetyStatus>("/idris/safety_status",10);
  monitoring_test::SafetyStatus msg;
  int status = 0;
  while(ros::ok()) {
    char buff[255];
    read(1,buff,255);
    if (status < 2)
      status++;
    else
      status = 0;
    msg.obstacle = status;
    pub.publish(msg);
  }
}
