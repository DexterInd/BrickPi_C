/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: Aug. 8, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a program for testing the RPi BrickPi drivers.
*/

#include <stdio.h>
#include <math.h>
#include <time.h>

#include "tick.h"

//#include <wiringPi.h>

#include "BrickPi.h"

//#include <unistd.h>  
//#include <errno.h>  
//#include <stdio.h>  
//#include <stdlib.h>  
#include <linux/i2c-dev.h>  
//#include <sys/ioctl.h>  
#include <fcntl.h>

// gcc -o program "/home/root/DB_MT/Raspberry Pi/C/Test BrickPi Motor PID position.c" -lrt -lm
// ./program

// gcc -o program "Test BrickPi Motor PID position.c" -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

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

  BrickPi.MotorEnable[PORT_B] = TYPE_MOTOR_POSITION;
  BrickPi.MotorEnable[PORT_C] = TYPE_MOTOR_POSITION;
  BrickPi.MotorEnable[PORT_D] = TYPE_MOTOR_POSITION;
  
  result = BrickPiSetupSensors();
  printf("BrickPiSetupSensors: %d\n", result); 
  if(!result){    
    while(1){
      result = BrickPiUpdateValues();
      if(!result){
        BrickPi.MotorTarget[PORT_B] = BrickPi.Encoder[PORT_A];
        BrickPi.MotorTarget[PORT_C] = BrickPi.Encoder[PORT_B];
        BrickPi.MotorTarget[PORT_D] = BrickPi.Encoder[PORT_C];
      }
      usleep(10000);
    }
  }
  return 0;
}