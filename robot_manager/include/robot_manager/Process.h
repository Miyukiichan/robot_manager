#ifndef PROCESS
#define PROCESS

#include <string>
#include <vector>
#include <regex>
#include <boost/algorithm/string.hpp>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <pty.h>
#include <termios.h>
#include <sys/types.h>

class Process {
  private:
    std::string name;
    std::string executable;
    std::string arguments;
    pid_t pid;
    std::vector<std::string> buffer;
    FILE * stream;
    int pty;
    struct termios * original;
    sigset_t * mask;
    sigset_t * oldmask;
  public:
    Process(std::string,std::string,std::string,struct termios *,sigset_t *,sigset_t *);
    void start();
    void kill_process();
    void refreshBuffer(int);
    std::string getName();
    std::vector<std::string> getBuffer();
    int checkDeath();
};

#endif
