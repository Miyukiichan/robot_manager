#include "robot_manager/Window.h"

/* This class manages windows and prints buffers. It also manages scrolling as well as displaying
 * teleoperation buttons and monitoring elements
 */

Window::Window(int h, int w, int y, int x, FileReader * reader, ros::NodeHandle * nh) {
  height = h;
  width = w;
  startY = y;
  startX = x;
  delwin(win);
  win = newwin(height,width,startY,startX);
  highlight = -1; //None of the indexes are highlighted
  tab = 0; //Vertical output by default
  tab_size = 1;
  teleop = 1; //Spawns teleop first by default
  monitor = 0;
  bufferSize = 0; //Initially empty
  scrollMode = 0; //Indicates if the user is currently scrolling
  initTeleop(10,5); //Draws the teleoperation buttons with the provided dimensions
  nodelay(win,1);
  keypad(win,1);
  if (reader != NULL) //File reader only needed for monitoring so other types of windows do not create monitor subscribers
    mon.init(nh,reader);
}

/* The main printing function that prints a vector of lines if in normal output mode.
 * Otherwise it calls one of the other relevant printing functions
 */
void Window::print(std::vector<std::string> list) {
  mon.refresh(); //Check for new messages if monitoring is enabled
  werase(win);
  if (teleop)
    drawTeleop();
  else if (monitor)
    drawMonitor();
  else {
    int axis = tab ? width - 2 : height - 2;
    bufferSize = list.size(); //The amount of lines in the buffer
    if (!scrollMode) //If the user is not scrolling and is viewing the output as it appears
      /* Scrolling is disabled if the buffer size is smaller than the height of the output window
       * Otherwise the printing loop starts at bufferSize - axis(height of the window) and ends at bufferSize
       * This effectively shows the most recent line and as many previous lines as the screen can fit
       */
      scrollPosition = bufferSize < axis ? 0 : bufferSize - axis;
    /*Printing loop starts from scroll position and stops before it reaches the end of the buffer or the end of the window*/
    for (int i = scrollPosition; i < scrollPosition + bufferSize && i < scrollPosition + axis; i++) {
      int index = i - scrollPosition + 1; //The current line in the window being printed to
      if (index == highlight) //If this is a line being selected in a menu
      /* Reverse the colour pair (background becomes white, foreground becomes black)
       * Indicates current selection by highlighting
       */
        wattron(win,A_REVERSE);
      std::string line = list[i];
      /*Increments horizontally or vertically*/
      if (tab)
        mvwprintw(win,1,index + ((index - 1) * tab_size),line.c_str());
      else
        mvwprintw(win,index,1,line.c_str());
      wattroff(win,A_REVERSE);
    }
    drawScrollbar();
  }
  box(win,0,0);
  wrefresh(win);
}

/* Resets the scrolling mode to 0 by default.
 * Used when switching tabs so that the output can be viewed properly
 */
void Window::clear() {
  scrollMode = 0;
}

/*Changes the scroll position by "amount" and toggles the scrolling mode accordingly*/
void Window::scr(int amount) {
  int axis = tab ? width - 2 : height - 2;
  if (amount > 0) { //Going Down
    /*Increases the scroll position if it does not exceed (bufferSize - the height of the window*/
    scrollPosition = scrollPosition + amount < bufferSize - axis ? scrollPosition + amount : bufferSize - axis;
    scrollMode = scrollPosition != bufferSize - axis; //If user scrolls to the bottom, scrolling mode is disabled
  }
  else if (amount < 0) { //Going Up
    /*Same as the previous block but in reverse*/
    scrollPosition = scrollPosition + amount > 0 ? scrollPosition + amount : 0;
    scrollMode = 1;
    }
  if (bufferSize <= axis) { // No Scrolling as it is impossible
    scrollPosition = 0;
    scrollMode = 0;
  }
}

/* Used for potential mouse controlled scrolling (not implemented)
 * The position on the scrollbar is multiplied by the amount of lines in the buffer per the amount on the screen
 * to calculate a new scroll position
 * This dynamic scrolling resolution lets the user scroll through all of the output but 
 * is impossible to view every line if the buffer is too big
 */
void Window::scrollJump(int position) {
  int multiplier = (bufferSize / height);
  scrollPosition = position * multiplier;
  /*Taking care of overshooting due to rounding errors*/
  if (bufferSize <= height || scrollPosition < multiplier)
    scrollPosition = 0;
  else if (scrollPosition > height || (height - scrollPosition) < multiplier) 
    scrollPosition = height;
}

/* Draws the scrollbar on the edge of the window.
 * The size of the bar is dependant on the size of the buffer in relation to the size of the window*/
void Window::drawScrollbar() {
  int maxBuffer = tab ? width - 2 : height - 2;
  if(maxBuffer + 1 < bufferSize) {
    int barSize = 1; //Default smallest size
    if((bufferSize - maxBuffer) < maxBuffer) //size between 1 and maxBuffer - 1
      barSize = maxBuffer - (bufferSize - maxBuffer);
    wattron(win,A_REVERSE);
    for (int i = 0; i <= barSize; i++) {
      if (tab)
        mvwprintw(win,height - 1,scrollPosition - i," ");
      else
        mvwprintw(win,scrollPosition - i,width - 1," ");
    }
    wattroff(win,A_REVERSE);
  }
}

/*Setting teleop mode*/
void Window::setTeleop(int t) {
  teleop = t;
}

/*Setting monitoring mode*/
void Window::setMonitor(int m) {
  monitor = m;
}

/* This function defines the sizes and labels for the teleoperation buttons before constructing
 * them and adding them to the list of buttons. It does this in order of:
 * top,right,down,left,centre
 */
void Window::initTeleop(int max_x, int max_y) {
  std::vector<std::string> options = {"Forwards","Right","Backwards","Left","Stop"};
  /*Determining which label is the biggest*/
  int biggest = 0;
  for (auto it : options) {
    biggest = it.size() > biggest ? it.size() : biggest;
  }
  /*Setting minimum dimensions for the buttons to display properly*/
  if (max_y < 3)
    max_y = 3;
  /*Needs to be an odd number to have the label on the centre line*/
  else if ((max_y % 2) == 0)
    max_y++;
  /*Has to be able to fit the biggest label with single spaces each side*/
  if ((max_x - biggest) < 0)
    max_x = biggest + 2;
  /*Even number to centre the biggest label. Results in odd numbered labels being off-centre
   * This cannot be avoided using character by character printing libraries
   */
  else if (((max_x - biggest) % 2) == 1)
    max_x++;
  /*Each button is positioned appropriately relative to every other button and the edge of the window*/
  for (int i = 0; i < options.size(); i++) {
    int x,new_max_x,y,new_max_y; 
    switch(i) {
      case 0:
        x = max_x + 2;
        y = 1;
        break;
      case 1:
        x = 2 * max_x + 3;
        y = max_y + 2;
        break;
      case 2:
        x = max_x + 2;
        y = 2 * max_y + 3;
        break;
      case 3:
        x = 1;
        y = max_y + 2;
        break;
      case 4:
        x = max_x + 2;
        y = max_y + 2;
    }
    Button button(win,max_y,max_x,y,x,options[i]);
    buttons.push_back(button);
  }
}

/*Calls the draw function for each button to be printed during teleop mode*/
void Window::drawTeleop() {
  for (auto it : buttons) {
    it.print();
  }
}

/*Returns the number associated with the pressed button. If no buttons are pressed -1 is returned*/
int Window::getPressed(MEVENT * event) {
  for (int i = 0; i < buttons.size(); i++) {
    if (buttons[i].pressed(event))
      return i;
  }
  return -1;
}

/*Function for drawing hard coded monitoring window. It gets the message data from the monitor
 * object and displays each element with a preceding label
 */
void Window::drawMonitor() {
  struct telem_data telem = mon.getTelem();
  struct safety_data safety = mon.getSafety();
  /*Creating the strings to be printed*/
  std::string speed = "Current speed: " + std::to_string(telem.speed);
  std::string steering = "Steering: " + std::to_string(telem.steering);
  std::string displacement = "Linear displacement: " + std::to_string(telem.linear_displacement);
  std::string safeSpeed = "Safe speed: " + std::to_string(safety.safeSpeed);
  std::string obstacle = "Obstacle status: ";
  std::string laserStatus = "Laser status: ";
  /*Printing the strings*/
  mvwprintw(win,1,1,"TELEMETRY");
  mvwprintw(win,2,1,speed.c_str());
  mvwprintw(win,3,1,steering.c_str());
  mvwprintw(win,4,1,displacement.c_str());
  mvwprintw(win,6,1,"SAFETY");
  mvwprintw(win,7,1,safeSpeed.c_str());
  /*Printing a colour-coded box instead of the raw value from the message*/
  drawColourBox(obstacle,8,safety.obstacle);
  drawColourBox(laserStatus,9,safety.laserStatus);
}

/*Prints a label and a colour-coded box based on the value of the provided message data*/
void Window::drawColourBox(std::string str, int line, int value) {
  value++; //Colour pairs cannot be 0
  /* Green, yellow and red colour pairs that set the foreground and background colours and 
   * assign them to numbers
   */
  init_pair(1,COLOR_GREEN,COLOR_GREEN);
  init_pair(2,COLOR_YELLOW,COLOR_YELLOW);
  init_pair(3,COLOR_RED,COLOR_RED);
  mvwprintw(win,line,1,str.c_str()); //Print the label before the box
  value = value < 4 ? value : 1; //Default value is 1 if outside the expected range
  /*Printing the box (two space characters) with the correct colour*/
  wattron(win,COLOR_PAIR(value));
  mvwprintw(win,line,str.size() + 1,"  ");
  wattroff(win,COLOR_PAIR(value));
}
