#include "robot_manager/Monitor.h"

/* A simple class that subscribes to topics defined in the configuration
 * file and returns structures that are processed by the printing function
 * in Window
 */

Monitor::Monitor() {
  enable = 0;
}

/* Init is used instead of a constructor as windows that inherit from the
 * Window class also have monitor objects. Having to deliberately 
 * initialise the subscribers removes unnecessary duplications*/
void Monitor::init(ros::NodeHandle * nh, FileReader * reader) {
  struct monitoring data = reader->getMonitoringInfo();    
  telemSub = nh->subscribe(data.telemTopic, 10, &Monitor::telemCallback,this);
  safetySub = nh->subscribe(data.safetyTopic, 10, &Monitor::safetyCallback,this);
  /*Structure data set to 0 by default so the window has something to print  
   * if there are no messages*/
  telemetry = {0};
  safety = {0};
  enable = 1;
}

/* Callback functions that transfer message data to structures for each 
 * message type when the respective message is received
 * */
void Monitor::telemCallback(const robot_manager::Telemetry &msg) {
  telemetry.speed = msg.speed;
  telemetry.steering = msg.steering;
  telemetry.linear_displacement = msg.linear_displacement;
}

void Monitor::safetyCallback(const robot_manager::SafetyStatus &msg) {
  safety.safeSpeed = msg.safeSpeed; 
  safety.obstacle = msg.obstacle; 
  safety.obstacleDistance = msg.obstacleDistance; 
  safety.obstacleBearing = msg.obstacleBearing; 
  safety.avoidanceMode = msg.avoidanceMode; 
  safety.laserStatus = msg.laserStatus; 
}

/*Getter functions*/
struct telem_data Monitor::getTelem() {
  return telemetry;
}

struct safety_data Monitor::getSafety() {
  return safety;
}

/*Check for messages once so the main loop can continue regardless*/
int Monitor::refresh () {
  if (enable)
    ros::spinOnce();
}
