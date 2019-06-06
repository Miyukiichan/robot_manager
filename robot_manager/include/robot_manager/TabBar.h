#ifndef TABBAR
#define TABBAR

#include "robot_manager/Menu.h"

class TabBar: public Menu {
  private:
    std::vector<std::string> extra;
    std::vector<Process> dead;
    void reap();
  public:
    TabBar(int,int,int,int);
    std::vector<std::string> refresh(int);
    void open(Process);
    int close();
    int isTeleop();
    int isMonitor();
    int getPressed(MEVENT*);
};

#endif
