#include "robot_manager/FileReader.h"

/* A class for reading a YAML file and constructing parameters to pass to 
 * other object constructors
 */

/*Constructor that immediately calls the readConfig function*/
FileReader::FileReader(const char * n, struct termios * original, sigset_t * mask, sigset_t * oldmask) {
  name = n;
  readConfig(original,mask,oldmask);
}

/* Reads each root node in the YAML file if it exists and constructs a set of instance variables
 * that are retrieved later. Accepts original terminal attributes and signal mask of the parent
 * as well as the altered mask to be applied to the children
 */
void FileReader::readConfig(struct termios * original, sigset_t * mask, sigset_t * oldmask) {
  YAML::Node config = YAML::LoadFile(name);
  for (auto it : config["command"]) {
    const std::string n = it["name"].as<std::string>();
    const std::string e = it["exec"].as<std::string>();
    std::string a = ""; //Parameter is empty by default
    if (it["param"])
      a = it["param"].as<std::string>();
    Process p(n,e,a,original,mask,oldmask);
    processes.push_back(p);
  }

  /*Reading teleoperation data*/
  if (config["teleop"]) {
    info.topic = config["teleop"]["topic"].as<std::string>();
    info.degradation = config["teleop"]["degradation"].as<double>();
    info.turning_increment = config["teleop"]["turning_increment"].as<double>();
    info.linear_increment = config["teleop"]["linear_increment"].as<double>();
    info.refresh_timeout = config["teleop"]["refresh_timeout"].as<double>();
    info.degradation_timeout = config["teleop"]["degradation_timeout"].as<double>();
  }

  /*Reading monitoring data*/
  if (config["monitoring"]) {
    mon.telemTopic = config["monitoring"]["telemetry_topic"].as<std::string>();
    mon.safetyTopic = config["monitoring"]["safety_topic"].as<std::string>();
  }
}

/*Getter functions*/
std::vector<Process> FileReader::getProcesses() {
  return processes;
}

struct teleop_info FileReader::getTeleopInfo() {
  return info;
}

struct monitoring FileReader::getMonitoringInfo() {
  return mon;
}
