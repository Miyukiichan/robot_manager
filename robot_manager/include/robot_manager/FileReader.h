#ifndef FILEREADER
#define FILEREADER

#include <yaml-cpp/yaml.h>
#include "robot_manager/Process.h"

/*These structs contain teleoperation and monitoring parameters that are passed to the 
 * Teleop and Monitor classes
 */

struct teleop_info {
  std::string topic;
  double degradation;
  double turning_increment;
  double linear_increment;
  double refresh_timeout;
  double degradation_timeout;
};

struct monitoring {
  std::string telemTopic;
  std::string safetyTopic;
};

class FileReader {
  private:
    const char * name;
    std::vector<Process> processes;
    struct teleop_info info;
    struct monitoring mon;
  public:
    FileReader(const char*,struct termios*,sigset_t*,sigset_t*);
    void readConfig(struct termios*,sigset_t*,sigset_t*);
    std::vector<Process> getProcesses();
    struct teleop_info getTeleopInfo();
    struct monitoring getMonitoringInfo();
};

#endif
