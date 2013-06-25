/*
*  Jaikrishna
*  t.s.jaikrishna<at>gmail.com
*  Initial date: June 21, 2013
*  Based on Matthew Richardson's example on testing BrickPi drivers and Xander Soldaat's Example on NXT for RobotC
*  You may use this code as you wish, provided you give credit where it's due.
*  
*  This is a program for testing the RPi BrickPi drivers and I2C communication on the BrickPi with a dCompass on HMC5883L 
*/

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <math.h>
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


int           X, Y, Z;
float angle;

#define PI 3.14159265359
#define I2C_PORT  PORT_1                             // I2C port for the dCompass
#define I2C_SPEED 0                                  // delay for as little time as possible. Usually about 100k baud

#define I2C_DEVICE_DCOM 0                        // DComm is device 0 on this I2C bus


int main() {
  ClearTick();

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;
  

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;

  BrickPi.SensorType       [I2C_PORT]    = TYPE_SENSOR_I2C;
  BrickPi.SensorI2CSpeed   [I2C_PORT]    = I2C_SPEED;

  BrickPi.SensorI2CDevices [I2C_PORT]    = 1;
  
  
/* Getting continuous values from a HMC5883L:
1. Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default, normal measurement)
2. Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
3. Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
4. Wait 6 ms or monitor status register or DRDY hardware interrupt pin
5. Loop
Send 0x3D 0x06 (Read all 6 bytes. If gain is changed then this data set is using previous gain)
Convert three 16-bit 2’s compliment hex values to decimal values and assign to X, Z, Y, respectively.
Send 0x3C 0x03 (point to first data register 03)
Wait about 67 ms (if 15 Hz rate) or monitor status register or DRDY hardware interrupt pin
End_loop */

  BrickPi.SensorSettings   [I2C_PORT][I2C_DEVICE_DCOM] = 0;  
  BrickPi.SensorI2CAddr    [I2C_PORT][I2C_DEVICE_DCOM] = 0x3C;	//address for writing
  
  if(BrickPiSetupSensors())
    return 0;
	
  //1
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_DCOM]    = 2;	//number of bytes to write
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_DCOM]    = 0;	//number of bytes to read
  
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][0] = 0x00;	//byte to write
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][1] = 0x70;	//byte to write
  if(BrickPiUpdateValues())		//writing
    return 0;
  if(!(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_DCOM)))	//BrickPi.Sensor[PORT] has an 8 bit number consisting of success(1) or failure(0) on all ports in bus
    return 0;
  
  //2
  //we're writing 2 bytes again, so there's no need to redefine number of butes
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][0] = 0x01;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][1] = 0xA0;  
  if(BrickPiUpdateValues())
    return 0;
  if(!(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_DCOM)))
    return 0;  
  //3
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][0] = 0x02;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][1] = 0x00;  
  if(BrickPiUpdateValues())
    return 0;
  if(!(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_DCOM)))
    return 0;  
	
  usleep(100000);
  //loop
  //writing 0x03 at address 0x3C to move the register to the first byte of data(0x03)
  //and then reading 6 bytes from 0x3D
  BrickPi.SensorSettings [I2C_PORT][I2C_DEVICE_DCOM]    = BIT_I2C_SAME;
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_DCOM]    = 1;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_DCOM]    = 6;  
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_DCOM][0] = 0x03;
  result = BrickPiSetupSensors();
  printf("BrickPiSetupSensors: %d\n", result); 
  if(!result){
    
    usleep(100000);
    
    while(1){
      result = BrickPiUpdateValues();
      if(!result){

        if(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_DCOM)){
          X = ((BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][0]&0x01)?-1:1)*(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][1]);
          Z = ((BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][2]&0x01)?-1:1)*(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][3]);
          Y = ((BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][4]&0x01)?-1:1)*(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_DCOM][5]);
          
		  angle = atan2(X,Y);
		  if(angle<0) angle += 2*PI;
		  angle *= 180/PI;
         
          printf("X: %d  Y: %d  Z: %d  H:%f \n", X, Y, Z,angle);
        
        }
      }
      usleep(100000);
    }
  }
  return 0;
}
