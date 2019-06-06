#include "robot_manager/TabBar.h"

/* The tab bar inherits from Menu and behaves in a similar way. 
 * One key difference is that it also manages the processes that 
 * are currently open
 */

TabBar::TabBar(int h, int w, int y, int x) : Menu(h,w,y,x) {
  tab = 1; //Changes printing orientation to horizontal
  tab_size = 10;
  /*Adding extra tab items that are not processes*/
  extra.push_back("Teleop");
  extra.push_back("Monitoring");
  Menu::setNames(extra);
  Menu::print();
}

/* This function gets the next line in the output stream for each process.
 * It also returns the buffer associated with the currently selected tab
 * as long as it is part of the process list. Otherwise, it returns an
 * empty buffer
 */
std::vector<std::string> TabBar::refresh(int size) {
  reap(); //Checks terminating processes
  if (!processes.empty()) {
    for (int i = 0; i < processes.size(); i++) {
      processes[i].refreshBuffer(size);
    }
    if (highlight > extra.size())
      return processes[highlight - 1 - extra.size()].getBuffer();
  }
  std::vector<std::string> s;
  return s;
}

/* Starts the given process "p" and adds it to the process list as well as the list of names
 * The opened process is then immediately selected
 */
void TabBar::open(Process p) {
  p.start();
  processes.push_back(p);
  Menu::setNames(extra);
  Menu::jump(names.size());
  Menu::print();
}

/* Kills the currently selected process, adds it to the death queue and removes it from the 
 * process list
 */
int TabBar::close() {
  if (!processes.empty() && highlight > extra.size()) {
    processes[highlight - 1 - extra.size()].kill_process();
    dead.push_back(processes[highlight - 1 - extra.size()]);
    processes.erase(processes.begin() + highlight - 1 - extra.size());
    names.erase(names.begin() + highlight - 1);
    if (highlight > names.size()) //Moves to the previous item if removing from the end
      Menu::move(-1);
    else
      Menu::print();
  }
  else if (processes.empty())
    return 1;
  return 0;
}

/*Returns true if the teleoperation tab is selected*/
int TabBar::isTeleop() {
  return highlight == 1;
}

/*Returns true if the monitoring tab is selected*/
int TabBar::isMonitor() {
  return highlight == 2;
}

/*Returns the index of the pressed tab. If no tabs were pressed, 0 is returned*/
int TabBar::getPressed(MEVENT * event) {
  if (wmouse_trafo(win,&event->y,&event->x,0) && names.size() * tab_size >= event->x) {
    return event->x/(tab_size + 1) + 1;
  }
return 0;
}

/*Removes successfully terminated processes from the death queue*/
void TabBar::reap() {
  for (int i = 0; i < dead.size(); i++) {
    if (dead[i].checkDeath())
      dead.erase(dead.begin() + i);
  }
}
