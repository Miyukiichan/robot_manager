#include "robot_manager/Button.h"

/* This class represents a sub window that acts as a button inside the main 
 * output window. Subwindows are used to make drawing borders easier as well
 * as easily check mouse events
 */

/*Constructer that defines the dimensions and creates the sub window*/
Button::Button(WINDOW * original, int h, int w, int x, int y, std::string t) {
  height = h;
  width = w;
  startX = x;
  startY = y;
  text = t;
  subwin = derwin(original,height,width,startX,startY);
  print();
}

/*Prints the label in the center of the button*/
void Button::print() {
  mvwprintw(subwin,height/2,width/2-text.size()/2,text.c_str());
  box(subwin,0,0);
}

/* Determins if the button was pressed
 * Reterns 1 if the mouse event occurs inside the sub window, otherwise 0 is 
 * returned
 * */
int Button::pressed(MEVENT * event) {
  return wmouse_trafo(subwin,&event->y,&event->x,0);
}
