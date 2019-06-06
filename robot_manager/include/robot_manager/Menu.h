#ifndef MENU
#define MENU

#include "robot_manager/Process.h"
#include "robot_manager/Window.h"

class Menu: public Window {
  protected:
    std::vector<Process> processes;
    std::vector<std::string> names;
  public:
    Menu(int,int,int,int);
    void print();
    void setProcesses(std::vector<Process>);
    void setNames(std::vector<std::string>);
    void move(int);
    void jump(int);
    Process open();
    int getPressed(MEVENT*);
};

#endif
