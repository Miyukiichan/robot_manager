#include "robot_manager/FileReader.h"
#include "robot_manager/Window.h"
#include "robot_manager/Menu.h"
#include "robot_manager/TabBar.h"
#include "robot_manager/Teleop.h"

/* Initialises program as a ROS node*/
void rosInit() {
  int argc = 0;
  char **argv = NULL;
	ros::init(argc,argv,"my_teleop");
}

/* Adjusts the signal mask to be sent to each pseudo-terminals*/
void setMask(sigset_t * m, sigset_t * o) {
  sigemptyset(m);
  sigaddset(m, SIGCHLD);
  sigaddset(m, SIGINT);
  sigaddset(m, SIGTERM);
  sigaddset(m, SIGWINCH);
  sigaddset(m, SIGRTMIN);
  sigaddset(m, SIGRTMIN + 1);
}

/* Tries to find the config file in .config
 * If it fails, the sample file name is returned instead
 */
std::string getFileName() {
  char * home = getenv("HOME");
  std::string filename = home + std::string("/.config/robot_manager/config.yaml");
  if (access(filename.c_str(),R_OK) == -1)
    filename = home + std::string("/catkin_ws/src/robot_manager/config.yaml");
  return filename;
}

/* Initialising ncurses settings and environment*/
void cursesInit() {
  initscr();
  mousemask(BUTTON1_CLICKED,NULL);
  notimeout(stdscr,1);
  curs_set(0);
  cbreak();
  noecho();
  start_color();
}

/* The main function initialises and displays all of the window objects and handles user input
 * The input loop also refreshes the teleoperation object as well as refreshes the child process 
 * output buffer
 */
int main() {
  cursesInit();
  rosInit();
  ros::NodeHandle nh;
  struct termios original;
  tcgetattr(0,&original);
  sigset_t mask;
  sigset_t oldmask;
  setMask(&mask,&oldmask);
  sigprocmask(SIG_BLOCK, &mask, &oldmask);
  FileReader reader(getFileName().c_str(),&original,&mask,&oldmask);

  /*Calculating the dimensions of all of the windows using the hight of the tab bar and the
   width of the side bar*/
  int max_y, max_x;
  getmaxyx(stdscr,max_y,max_x);
  const int tab_bar_height = 3;
  const int side_bar_width = 25;
  const int tab_bar_width = max_x - side_bar_width;
  const int menu_height = max_y - tab_bar_height;

  /*The title window in the top left and the close button are not window objects as they do not
   need to print multiple lines. They are instead just normal ncurses windows*/
  WINDOW * title = newwin(tab_bar_height,side_bar_width,0,0);
  mvwprintw(title,1,1,"Commands");
  box(title,0,0);
  wrefresh(title);
  WINDOW * close = newwin(tab_bar_height,5,0,max_x - 5);
  mvwprintw(close,1,2,"X");
  box(close,0,0);
  wrefresh(close);
  nodelay(close,1);
  keypad(close,1);

  /*Object initialisation*/
  Window main(menu_height,tab_bar_width,tab_bar_height,side_bar_width,&reader,&nh);
  Menu menu(menu_height,side_bar_width,tab_bar_height,0);
  menu.setProcesses(reader.getProcesses());
  menu.print();
  TabBar bar(tab_bar_height,tab_bar_width - 5,0,side_bar_width);
  Teleop teleop(reader,&nh);

  /*Main Loop*/
  int finished = 0;
  while (!finished) {
    MEVENT event;

    /*Normal output mode is the default but switches to teleop/monitoring mode
     depending on the current tab*/
    main.setTeleop(0);
    main.setMonitor(0);
    if (bar.isTeleop()) {
      main.setTeleop(1);
      teleop.refresh();
    }
    else if (bar.isMonitor()) {
      main.setMonitor(1);
    }
    main.print(bar.refresh(tab_bar_width - 1));
    wmove(close,0,0); //Ensures that all input is registered from the close window for consistency

    /*Interpret user input*/
    switch(wgetch(close)) {
      case 'p':
        menu.scr(-1);
        menu.print();
        break;
      case 'o':
        menu.scr(1);
        menu.print();
        break;
      case 'j':
        menu.move(1);
        break;
      case 'k':
        menu.move(-1);
        break;

      /*Switch between tabs and send stop signal if moving away from the 
       teleoperation tab*/
      case 'h':
        if (bar.isTeleop()) {
          teleop.stop();
        }
        main.clear();
        bar.move(-1);
        break;
      case 'l':
        if (bar.isTeleop()) {
          teleop.stop();
        }
        main.clear();
        bar.move(1);
        break;

      /*Control the robot only if viewing the teleoperation tab*/
      case KEY_UP:
        if (bar.isTeleop())
          teleop.move(1);
        break;
      case KEY_LEFT:
        if (bar.isTeleop())
          teleop.turn(1);
        break;
      case KEY_DOWN:
        if (bar.isTeleop())
          teleop.move(0);
        break;
      case KEY_RIGHT:
        if (bar.isTeleop())
          teleop.turn(0);
        break;

      /*Stopping occurs regardless of what tab is being viewed (for safety).
       It also switches to the teleoperation tab for quick control in case
       of emergencies*/
      case ' ':
        teleop.stop();
        bar.jump(1);
        break;
      case 10:
        {
        main.clear();
        bar.open(menu.open());
        break;
        }
      case 'x':
        main.clear();
        bar.clear();
        bar.close();
        break;
      case 'm':
        main.scr(-1);
        break;
      case 'n':
        main.scr(1);
        break;
      case 'q':
        finished = 1;
        break;

      /*Checking for mouse input*/
      case KEY_MOUSE:
        if (getmouse(&event) == OK) {
          if (wmouse_trafo(close,&event.y,&event.x,0)) {
            main.clear();
            bar.clear();
            finished = bar.close();
          }

          /*Select a menu item and open it in a tab*/
          int menuIndex = menu.getPressed(&event);
          if (menuIndex) {
            main.clear();
            menu.jump(menuIndex);
            bar.open(menu.open());
          }

          /*Select a tab and toggle teleop mode accordingly*/
          menuIndex = bar.getPressed(&event);
          if (menuIndex)
            bar.jump(menuIndex);
            if (menuIndex > 1)
              teleop.stop();

          /*Touch screen teleoperation*/
          if (bar.isTeleop()) {
            menuIndex = main.getPressed(&event);
            switch(menuIndex) {
              case 0:
                teleop.move(1);
                break;
              case 1:
                teleop.turn(0);
                break;
              case 2:
                teleop.move(0);
                break;
              case 3:
                teleop.turn(1);
                break;
              case 4:
                teleop.stop();
                break;
            }
          }
        }
        break; 
    }
    usleep(5000); //Small delay keeps CPU usage below 100%
  }
  teleop.stop();
  endwin();
  return 0;
}
