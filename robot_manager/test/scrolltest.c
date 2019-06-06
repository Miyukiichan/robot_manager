#include <stdio.h>
#include <stdlib.h>

int main(int argc, char * argv[]) {
  if (argc < 2)
    printf("No args\n");  
  else {
    for(int i = 0; i < atoi(argv[1]); i++)
      printf("%d\n",i);
  }
  return 0;
}
