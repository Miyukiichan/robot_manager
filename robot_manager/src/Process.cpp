#include "robot_manager/Process.h"

/* This is a fairly complex class that represents processes that can be run
 * from within the application
 */

Process::Process(std::string n,std::string e,std::string a,struct termios * o,sigset_t * m, sigset_t * old) {
  name = n;
  executable = e;
  arguments = a;
  original = o;
  mask = m;
  oldmask = old;
}

/* This function starts a process after forking a new pseudo-terminal and
 * correctly formatting the executable and parameter strings to be passed 
 * to execv
 */
void Process::start() {
  sigprocmask(SIG_SETMASK,mask,oldmask); //Setting the new mask with certain signals disabled
  pid = forkpty(&pty,0,original,0); //Combines forking and creating new pty, passes original termios settings to be used
  if (pid == 0) { //Child process
    /*Replacing the full path with just the executable file name*/
    std::string command(executable);
    std::vector<std::string> vect;
    boost::split(vect,command,boost::is_any_of("/"));
    command = vect[vect.size() - 1];
    vect.clear();
    /*Adding additional arguments*/
    if (arguments != "")
      boost::split(vect,arguments,boost::is_any_of(" "));
    int size = vect.size() + 2; //Arguments + executable + null terminator
    char * ar[size];
    for (int i = 0; i < size; i++) {
      char temp[255];
      if (i == size - 1) {
        ar[i] = NULL; //Setting null terminator
      }
      else {
        if (i == 0) { //Adding the executable name
          stpcpy(temp,command.c_str());
        }
        else {
          stpcpy(temp,vect[i - 1].c_str()); //Adding each parameter
        }
        ar[i] = (char *) malloc(strlen(temp));
        stpcpy(ar[i],temp);
      }
    }
    execv(executable.c_str(), ar);
    for (int i = 0; i < size - 1; i++)
      free(ar[i]);
    exit(0);
  }
  else {
    /*Disables blocking and standard input for the child pty*/
    int flags = fcntl(pty,F_GETFL);
    fcntl(pty, F_SETSIG, SIGRTMIN + 1);
    fcntl(pty, F_SETFL, O_ASYNC | O_NONBLOCK | flags);
    fcntl(pty, F_SETOWN, pid);
    stream = fdopen(pty,"r"); //Opens child pty file descriptor as a file stream
  }
}

/*Sends an interrupt signal to the child process and stops receiving output*/
void Process::kill_process() {
  fclose(stream);
  close(pty);
  kill(pid,SIGINT);
}

/* Reads a line of text from the output stream and adds it to the buffer.
 * A line is read until a newline character or until "length".
 * This results in line wrapping according to the size of the output window.
 * ASCII escape codes are removed from each line using regex
 */
void Process::refreshBuffer(int length) {
  char buff[length];
  if (fgets(buff,length,stream)) {
    std::string s(buff);
    std::regex r("[^/]\\[+\\]?\\?*[0-9;]*[a-zA-z]");
    s = std::regex_replace(s,r,"");
    buffer.push_back(s);
  }
}

/*Returns 1 if the process has terminated. Otherwise 0 is returned*/
int Process::checkDeath() {
  int status;
  return waitpid(pid,&status,WNOHANG) == pid;
}

/*Getter functions*/
std::string Process::getName() {
  return name;
}

std::vector<std::string> Process::getBuffer() {
  return buffer;
}
