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

// gcc -o program "/home/root/DB_MT/Raspberry Pi/C/Test BrickPi Sensors.c" -lrt -lm
// ./program

// gcc -o program "Test BrickPi Sensors.c" -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

int result;

int main() {
  ClearTick();

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;
  
  BrickPi.Timeout = 500;                       // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;

  BrickPi.SensorType[PORT_1] = TYPE_SENSOR_LIGHT_ON;
  BrickPi.SensorType[PORT_2] = TYPE_SENSOR_TOUCH;  
  BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT;
  BrickPi.SensorType[PORT_4] = TYPE_SENSOR_COLOR_FULL;
  
  result = BrickPiSetupSensors();
  printf("BrickPiSetupSensors: %d\n", result); 
  if(!result){    
    while(1){
      result = BrickPiUpdateValues();
      if(!result){
//        printf("Results:\n  Light: %4.1d\n  Touch: %.1d\n  US   : %3.1d\n  Color: %3.1d\n    Blank: %4.1d\n    Red  : %4.1d\n    Green: %4.1d\n    Blue : %4.1d\n", BrickPi.Sensor[PORT_1], BrickPi.Sensor[PORT_2], BrickPi.Sensor[PORT_3], BrickPi.Sensor[PORT_4], BrickPi.SensorArray[PORT_4][INDEX_BLANK], BrickPi.SensorArray[PORT_4][INDEX_RED], BrickPi.SensorArray[PORT_4][INDEX_GREEN], BrickPi.SensorArray[PORT_4][INDEX_BLUE]);
        printf("Light: %4.1d  Touch: %.1d  US: %3.1d  Color: %.1d  Blank: %4.1d  Red: %4.1d  Green: %4.1d  Blue: %4.1d\n", BrickPi.Sensor[PORT_1], BrickPi.Sensor[PORT_2], BrickPi.Sensor[PORT_3], BrickPi.Sensor[PORT_4], BrickPi.SensorArray[PORT_4][INDEX_BLANK], BrickPi.SensorArray[PORT_4][INDEX_RED], BrickPi.SensorArray[PORT_4][INDEX_GREEN], BrickPi.SensorArray[PORT_4][INDEX_BLUE]);
      }
//      usleep(240000);
      usleep(10000);
    }
  }
  return 0;
}