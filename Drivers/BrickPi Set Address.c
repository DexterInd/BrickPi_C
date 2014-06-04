/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: Sep. 13, 2013
*  Last updated: Sep. 14, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a program for testing the RPi BrickPi drivers.
*/

#define DEBUG

#include <stdio.h>
#include <math.h>
#include <time.h>

#include "tick.h"

#include "BrickPi.h"

//#include <unistd.h>  
//#include <errno.h>  
//#include <stdio.h>  
//#include <stdlib.h>  
#include <linux/i2c-dev.h>  
//#include <sys/ioctl.h>  
#include <fcntl.h>

// gcc -o program "/home/root/DB_MT/Raspberry Pi/C/BrickPi Set Address.c" -lrt -lm
// ./program

// gcc -o program "/home/pi/dbrpi/C/BrickPi Set Address.c" -lrt -lm -L/usr/local/lib -lwiringPi
// sudo ./program

int result;

int main() {
  ClearTick();

  result = BrickPiSetupAddress(0, 2);
  printf("BrickPiSetupAddress: %d\n", result);
  return 0;
}