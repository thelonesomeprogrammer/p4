// main.c that takes a string and prints it to the console
#include <stdio.h>

extern int pidinit();
extern int pidupdate();

extern int leadinit();
extern int leadupdate();

int main(int argc, char *argv[]) {
  if (argc < 1) {
    return 1;
  }

  printf("%i\n", 60);
  return 0;
}
