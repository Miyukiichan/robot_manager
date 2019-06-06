#include "robot_manager/Menu.h"

/* This class inherits from Window and implements menu selection functionality.
 * It also contains a list of processes that can be opened (returned)
 */

/* Sets teleoperation and monitoring mode to 0 after calling the parent constructor
 * The highlight index starts at 1 as the beginning of the list by default
 * */
Menu::Menu(int h, int w, int y, int x) : Window(h,w,y,x,NULL,NULL) {
  highlight = 1;
  teleop = 0;
  monitor = 0;
}

/*Runs the printing function on the list of process names*/
void Menu::print() {
  Window::print(names);
}

/*Clears any existing names and sets the process list*/
void Menu::setProcesses(std::vector<Process> p) {
  processes = p;
  std::vector<std::string> none;
  setNames(none);
}

/* Sets the list of names based on any exra items (ie. teleop and monitor)
 * as well as each process name*/
void Menu::setNames(std::vector<std::string> extra) {
  names.clear();
  for (auto it : extra)
    names.push_back(it);
  for (auto it : processes)
    names.push_back(it.getName());
}

/* Moves the index up or down the list by "index" amount and wraps around if 
 * the upper/lower limits are exceeded
 */
void Menu::move(int shift) {
  int size = names.size();
  if (shift < 0) {
    highlight = (highlight <= 1) ? size : highlight + shift;
  }
  else if (shift > 0) {
    highlight = (highlight >= size) ? 1 : highlight + shift;
  }
  print();
}

/* Jumps to a given menu item if it is inside the bounds of the list
 * otherwise it is set to the closest extreme value
 */
void Menu::jump(int location) {
  if (location < 1)
    highlight = 1;
  else if (location > names.size())
    highlight = names.size();
  else
    highlight = location;
  print();
}

/* Returns the process associated with the currently selected menu item.
 * The extra items are dealt with in TabBar*/
Process Menu::open() {
  return processes[highlight - 1];
}

/*Returns the index of the pressed menu item if it exists. Otherwise 0 is returned*/
int Menu::getPressed(MEVENT * event) {
  int maxBuffer = tab ? height - 2 : width - 2;
  if (wmouse_trafo(win,&event->y,&event->x,0) && names.size() >= event->y) {
    int index = event->y;
    /*Takes scrolling offset into account if the menu is too big*/
    if (names.size() > maxBuffer) {
      index+=((names.size() - maxBuffer) - scrollPosition);
    }
    return index;
  }
  return 0;
}
