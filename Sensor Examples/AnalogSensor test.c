/*
*  Jaikrishna
*  t.s.jaikrishna<at>gmail.com
*  Initial date: June 20, 2013
*  Based on Matthew Richardson's Example for testing BrickPi
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a program for testing the RPi BrickPi driver with Analog Sensors on all Ports
*/

#include <stdio.h>
#include <math.h>
#include <time.h>

#include "tick.h"

#include <wiringPi.h>

#include "BrickPi.h"

//#include <unistd.h>  
//#include <errno.h>  
//#include <stdio.h>  
//#include <stdlib.h>  
#include <linux/i2c-dev.h>  
//#include <sys/ioctl.h>  
#include <fcntl.h>

// gcc -o program "Test BrickPi lib.c" -lrt -lm -L/usr/local/lib -lwiringPi
// gcc -o program "Test BrickPi lib.c" -lrt
// ./program

int result;
#undef DEBUG


int main() {
  ClearTick();

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;
 
  BrickPi.SensorType[PORT_1] = TYPE_SENSOR_RAW;
  BrickPi.SensorType[PORT_2] = TYPE_SENSOR_RAW;
  BrickPi.SensorType[PORT_3] = TYPE_SENSOR_RAW;
  BrickPi.SensorType[PORT_4] = TYPE_SENSOR_RAW;

  result = BrickPiSetupSensors();
  printf("BrickPiSetupSensors: %d\n", result); 
  if(!result){
    
    usleep(10000);
    
    while(1){
      result = BrickPiUpdateValues();

      if(!result){
      	
         printf("Results: %3.1d %3.1d %3.1d %3.1d \n", BrickPi.Sensor[PORT_1], BrickPi.Sensor[PORT_2],BrickPi.Sensor[PORT_3], BrickPi.Sensor[PORT_4] );
       }
      usleep(10000);
    }
  }
  return 0;
}