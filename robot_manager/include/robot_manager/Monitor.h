#ifndef MONITOR
#define MONITOR

#include <ros/ros.h>
#include "robot_manager/FileReader.h"
#include "robot_manager/SafetyStatus.h"
#include "robot_manager/Telemetry.h"

/*These structs contain data recieved from ros messages that are used in the monitoring tab
 * They are initialized to 0 in case there are no messages
 */

struct telem_data {
  double speed;
  double steering;
  double linear_displacement;
};

struct safety_data {
  int safeSpeed;
  int obstacle;
  double obstacleDistance;
  double obstacleBearing;
  int avoidanceMode;
  int laserStatus;
};

class Monitor {
  private:
    ros::Subscriber telemSub;
    ros::Subscriber safetySub;
    struct telem_data telemetry;
    struct safety_data safety;
    int enable;
    void telemCallback(const robot_manager::Telemetry &);
    void safetyCallback(const robot_manager::SafetyStatus &);
  public:
    Monitor();
    void init(ros::NodeHandle*,FileReader*);
    struct telem_data getTelem(); 
    struct safety_data getSafety(); 
    int refresh();
};

#endif
