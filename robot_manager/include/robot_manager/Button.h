#include <string>
#include <curses.h>

class Button {
  private:
    WINDOW * subwin;
    int startX,startY,width,height;
    std::string text;
  public:
    Button(WINDOW*,int,int,int,int,std::string);
    void print();
    int pressed(MEVENT*);
};
