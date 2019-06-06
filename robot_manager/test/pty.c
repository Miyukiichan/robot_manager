#include <pty.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <curses.h>
#include <termios.h>

int main() {
  initscr();
  int pty;
  char name[255];
  char buff[255];
  struct termios original;
  tcgetattr(1,&original);
  if (forkpty(&pty,name,&original,0)) {
    //Parent process
    FILE * f = fdopen(pty,"r");
    for (int i = 0; i < 20; i++) {
      memset(buff,0,255);
      if(fgets(buff,255,f)) {
        mvprintw(i,0,buff);
        refresh();
        FILE * out = fopen("myout.txt","a");
        fputs(buff,out);
        fclose(out);
      }
      sleep(1);
    }
  }
  else {
    execl("/bin/ls","ls","-l",(char*)0);
    exit(0);
  }
  endwin();
  return 0;
}
