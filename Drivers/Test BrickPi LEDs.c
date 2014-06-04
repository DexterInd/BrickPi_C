/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: Sep. 24, 2013
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

// gcc -o program "/home/root/DB_MT/Raspberry Pi/C/Test BrickPi LEDs.c" -lrt -lm
// ./program

// nano "/media/BEAGLEBONE_/Matt's Technology/Raspberry Pi/C/Test BrickPi LEDs.c"
// gcc -o program "/media/BEAGLEBONE_/Matt's Technology/Raspberry Pi/C/Test BrickPi LEDs.c" -lrt -lm
// ./program

// gcc -o program "/home/pi/dbrpi/C/Test BrickPi LEDs.c" -lrt -lm -L/usr/local/lib -lwiringPi
// sudo ./program

int result;

int main() {
  ClearTick();

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;
  
  BrickPi.Timeout = 100;                       // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;
  
  int l;
  int i;
  while(1){
    for(l = 0; l < 5; l++){
      for(i = 0; i < 5; i++){
        BrickPiSetLed(LED_1, LED_ON);
        delay(25);
        BrickPiSetLed(LED_1, LED_OFF);
        delay(25);
      }
      for(i = 0; i < 5; i++){
        BrickPiSetLed(LED_2, LED_ON);
        delay(25);
        BrickPiSetLed(LED_2, LED_OFF);
        delay(25);
      }
    }
    
    for(l = 0; l < 10; l++){
      BrickPi.LED[LED_1] = LED_OFF;
      BrickPi.LED[LED_2] = LED_ON;
      BrickPiUpdateLEDs();
      delay(100);
      BrickPi.LED[LED_1] = LED_ON;
      BrickPi.LED[LED_2] = LED_OFF;
      BrickPiUpdateLEDs();
      delay(100);
    }
    
/*    for(l = 0; l < 2; l++){
      BrickPi.LED[LED_2] = LED_ON;    
      for (i = 0; i < 1024; i++)
      {
        BrickPi.LED[LED_1] = i;
        BrickPiUpdateLEDs();
        delay(1);
      }
      BrickPi.LED[LED_2] = LED_OFF;
      for (i = 1023; i >= 0; i--)
      {
        BrickPi.LED[LED_1] = i;
        BrickPiUpdateLEDs();
        delay(1);
      }
    }*/
  }
  return 0;
}