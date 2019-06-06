#ifndef TELEOP
#define TELEOP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "robot_manager/FileReader.h"

class Teleop {
  private:
    std::string topic_name;
    double degradation;
    double turning_increment;
    double linear_increment;
    double speed;
    double rotation;
    double refresh_timeout;
    double degradation_timeout;
    ros::Publisher pub;
    ros::Time refresh_begin;
    ros::Time degradation_begin;
  public:
    Teleop(FileReader,ros::NodeHandle*);
    void refresh();
    void turn(int);
    void move(int);
    void stop();
    void sendMessage();
};

#endif
