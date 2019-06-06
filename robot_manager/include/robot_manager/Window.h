#ifndef WIN
#define WIN

#include "robot_manager/Button.h"
#include "robot_manager/FileReader.h"
#include "robot_manager/Monitor.h"

class Window {
  protected:
    int height, width, startY, startX;
    WINDOW *win;
    int highlight;
    int tab;
    int tab_size;
    int scrollPosition;
    int teleop;
    int monitor;
    std::vector<Button> buttons;
    int bufferSize;
    int scrollMode;
    Monitor mon;
    void drawScrollbar();
    void drawTeleop();
    void drawMonitor();
    void drawColourBox(std::string,int,int);
    void initTeleop(int,int);
  public:
    Window(int,int,int,int,FileReader* = NULL,ros::NodeHandle* = NULL);
    void print(std::vector<std::string>);
    void setTeleop(int);
    void setMonitor(int);
    void clear();
    void scr(int);
    void scrollJump(int);
    int getPressed(MEVENT*);
};

#endif
