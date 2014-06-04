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

unsigned char SX, SY, B;
int           AX, AY, AZ;

float VX, VY, VZ;
int V1, V2, V3;

//int SpeedLeft, SpeedRight;

/*#define PSP_NXPort  PORT_4
#define PSP_NXSpeed 0//20
#define NXTChuckPort  PORT_3
#define NXTChuckSpeed 0//20*/

#define T_PORT          PORT_1

#define US_PORT         PORT_3                       // For the FW Ultrasonic sensor support, use port 3

#define US_I2C_TYPE     TYPE_SENSOR_I2C_9V           // Sensor type is I2C, with 9v pullup
#define US_I2C_PORT     PORT_4                       // Sensor port 2 (I2C bus 2)
#define US_I2C_SPEED    7                            // 7 causes the bus to run at about 40k baud, which is about the fastest the NXT ultrasonic sensor supports
#define US_I2C_DEVICE   0                            // device 0 on this bus
#define US_I2C_SETTINGS (BIT_I2C_SAME | BIT_I2C_MID) // The message and message lengths will be the same, and it needs one of those funny clock pulses mid way, between the write and the read.

#define I2C_PORT  PORT_2                             // I2C bus for the NXTChuck and PSP-Nx
#define I2C_SPEED 0                                  // delay for as little time as possible. Usually about 100k baud

#define I2C_DEVICE_NXTCHUCK 0                        // NXTChuck is device 0 on this I2C bus
#define I2C_DEVICE_PSP_NX   1                        // PSP-Nx is device 1 on this I2C bus

int main() {
  ClearTick();

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;
  
  BrickPi.Timeout = 100;                       // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;
  
/*  result = BrickPiChangeAddress(0, 2);          // For changing the address of one of the slave uCs
  printf("BrickPiChangeAddress: %d\n", result);  

  return 0;*/

  BrickPi.SensorType[PORT_1] = TYPE_SENSOR_RAW;//TYPE_SENSOR_LIGHT_ON;
  BrickPi.SensorType[PORT_2] = TYPE_SENSOR_RAW;//TYPE_SENSOR_LIGHT_ON; 
  BrickPi.SensorType[PORT_3] = TYPE_SENSOR_RAW;//TYPE_SENSOR_COLOR_FULL;
  BrickPi.SensorType[PORT_4] = TYPE_SENSOR_RAW;//TYPE_SENSOR_ULTRASONIC_CONT;

  BrickPi.SensorType       [I2C_PORT]    = TYPE_SENSOR_I2C_9V;
  BrickPi.SensorI2CSpeed   [I2C_PORT]    = I2C_SPEED;

  BrickPi.SensorI2CDevices [I2C_PORT]    = 2;
  
  BrickPi.SensorSettings   [I2C_PORT][I2C_DEVICE_NXTCHUCK] = 0;  
  BrickPi.SensorI2CAddr    [I2C_PORT][I2C_DEVICE_NXTCHUCK] = 0xA4;
  
  BrickPi.SensorSettings   [I2C_PORT][I2C_DEVICE_PSP_NX]   = 0;  
  BrickPi.SensorI2CAddr    [I2C_PORT][I2C_DEVICE_PSP_NX]   = 0x02;
  
  if(BrickPiSetupSensors())
    return 0;
  
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_NXTCHUCK]    = 2;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_NXTCHUCK]    = 0;
  
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_NXTCHUCK][0] = 0xF0;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_NXTCHUCK][1] = 0x55;  
  if(BrickPiUpdateValues())
    return 0;
  if(!(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_NXTCHUCK)))
    return 0;

  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_NXTCHUCK][0] = 0xFB;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_NXTCHUCK][1] = 0x00;  
  if(BrickPiUpdateValues())
    return 0;
  if(!(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_NXTCHUCK)))
    return 0;  
  
  BrickPi.SensorSettings [I2C_PORT][I2C_DEVICE_NXTCHUCK]    = BIT_I2C_SAME;
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_NXTCHUCK]    = 1;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_NXTCHUCK]    = 6;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_NXTCHUCK][0] = 0x00;  

  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_PSP_NX]    = 0;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_PSP_NX]    = 0;

  BrickPi.SensorSettings [I2C_PORT][I2C_DEVICE_PSP_NX]    = BIT_I2C_SAME;
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_PSP_NX]    = 1;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_PSP_NX]    = 6;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_PSP_NX][0] = 0x42;
  
  BrickPi.SensorType       [US_I2C_PORT]               = US_I2C_TYPE;
  BrickPi.SensorI2CSpeed   [US_I2C_PORT]               = US_I2C_SPEED;
  BrickPi.SensorI2CDevices [US_I2C_PORT]               = 1;
  BrickPi.SensorSettings   [US_I2C_PORT][US_I2C_DEVICE]    = US_I2C_SETTINGS;  
  BrickPi.SensorI2CAddr    [US_I2C_PORT][US_I2C_DEVICE]    = 0x02;
  BrickPi.SensorI2CWrite   [US_I2C_PORT][US_I2C_DEVICE]    = 1;
  BrickPi.SensorI2CRead    [US_I2C_PORT][US_I2C_DEVICE]    = 1;
  BrickPi.SensorI2COut     [US_I2C_PORT][US_I2C_DEVICE][0] = 0x42;
  
  BrickPi.SensorType[US_PORT] = TYPE_SENSOR_ULTRASONIC_CONT;
  BrickPi.SensorType[T_PORT] = TYPE_SENSOR_TOUCH;
  
/*  BrickPi.SensorType[PORT_2] = TYPE_SENSOR_RAW;//TYPE_SENSOR_LIGHT_ON; 
  BrickPi.SensorType[PORT_3] = TYPE_SENSOR_RAW;//TYPE_SENSOR_COLOR_FULL;
  BrickPi.SensorType[PORT_4] = TYPE_SENSOR_RAW;//TYPE_SENSOR_ULTRASONIC_CONT;*/
  BrickPi.MotorEnable[PORT_A] = 1;
  BrickPi.MotorEnable[PORT_B] = 1;
  BrickPi.MotorEnable[PORT_C] = 1;
//  BrickPi.MotorEnable[PORT_D] = 0;
  
  result = BrickPiSetupSensors();
  printf("BrickPiSetupSensors: %d\n", result); 
  if(!result){
    
    usleep(10000);
    
    while(1){
      result = BrickPiUpdateValues();
//      printf("BrickPiUpdateValues: %d\n", result);
      if(!result){

/*          SpeedLeft  = ((SY - 128) + (SX - 128)) * 2;
          SpeedRight = ((SY - 128) - (SX - 128)) * 2;
          if(SpeedLeft < -255)
            SpeedLeft = -255;
          else if(SpeedLeft > 255)
            SpeedLeft = 255;
          if(SpeedRight < -255)
            SpeedRight = -255;
          else if(SpeedRight > 255)
            SpeedRight = 255;
          printf("SX: %3.1d  SY: %3.1d  AX: %4.1d  AY: %4.1d  AZ: %4.1d  B: %.1d  L: %3.1d  R: %3.1d\n", SX, SY, AX, AY, AZ, B, SpeedLeft, SpeedRight);*/
        
        printf("Results: %.1d %3.1d %.1d %.1d\n", BrickPi.Sensor[T_PORT], BrickPi.Sensor[US_PORT], BrickPi.Sensor[US_I2C_PORT], BrickPi.Sensor[I2C_PORT]);
        
        if(BrickPi.Sensor[US_I2C_PORT] & (0x01 << US_I2C_DEVICE)){
          printf("%3.1d\n", BrickPi.SensorI2CIn[US_I2C_PORT][US_I2C_DEVICE][0]);
        }
        
        if(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_PSP_NX)){
          printf("%3.1d %3.1d %3.1d %3.1d %3.1d %3.1d\n", BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][0], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][1], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][2], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][3], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][4], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][5]);
        }
        
        if(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_NXTCHUCK)){
          SX = BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][0];
          SY = BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][1];
          AX = (BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][2] << 2);
          AY = (BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][3] << 2);
          AZ = (BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][4] << 2);
          B  = ((~BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_NXTCHUCK][5]) & 0x03);
          
          VZ = 0;
          if(B & 0x01 && AX < 462)
            VZ = (AX - 462);
          else if(B & 0x01 && AX > 562)
            VZ = (AX - 562);          
          if(VZ > 128)
            VZ = 128;
          if(VZ < -128)
            VZ = -128;
          
          VY = 0;
          if(B & 0x01 && AY < 500)
            VY = (AY - 500);
          else if(B & 0x01 && AY > 562)
            VY = (AY - 562);          
          if(VY > 128)
            VY = 128;
          if(VY < -128)
            VY = -128;
          
          VX = 0;
          if(B & 0x01)
            VX = (SX - 128);
          
          VX *= 2;
          VY = ((-VY) * 2);
          VZ *= 2;
          
          V1 = VX + VZ;                         //VX + VZ;                                     // Vector Calculation for MotorA(V1)'s Power
          V2 = (-VX / 2 - sqrt(3)/2 * VY) + VZ; //((-VX / 3) - (VY * 2 / 3)) + VZ //(-VX / 2 - math.sqrt(3)/2 * VY) + VZ;        // Vector Calculation for MotorB(V2)'s Power
          V3 = (-VX / 2 + sqrt(3)/2 * VY) + VZ; //((-VX / 3) + (VY * 2 / 3)) + VZ //(-VX / 2 + math.sqrt(3)/2 * VY) + VZ;        // Vector Calculation for MotorC(V3)'s Power
          
          if (V1 <  10 && V1 > 0)
            V1 =    10;
          if (V1 > -10 && V1 < 0)
            V1 =   -10;            
          if (V2 <  10 && V2 > 0)
            V2 =    10;
          if (V2 > -10 && V2 < 0)
            V2 =   -10;
          if (V3 <  10 && V3 > 0)
            V3 =    10;
          if (V3 > -10 && V3 < 0)
            V3 =   -10;
          
          printf("SX: %3.1d  SY: %3.1d  AX: %4.1d  AY: %4.1d  AZ: %4.1d  B: %.1d\n", SX, SY, AX, AY, AZ, B);
          printf("VX: %3.1d  VY: %3.1d  VZ: %3.1d  V1: %3.1d  V2: %3.1d  V3: %3.1d\n", VX, VY, VZ, V1, V2, V3);
          
          BrickPi.MotorEnable[PORT_A] = 1;
          BrickPi.MotorEnable[PORT_B] = 1;
          BrickPi.MotorEnable[PORT_C] = 1;
          BrickPi.MotorSpeed[PORT_A] = V1;
          BrickPi.MotorSpeed[PORT_B] = V2;
          BrickPi.MotorSpeed[PORT_C] = V3;
        }
        else{
          BrickPi.MotorEnable[PORT_A] = 0;
          BrickPi.MotorEnable[PORT_B] = 0;
          BrickPi.MotorEnable[PORT_C] = 0;
        }
        /*BrickPi.MotorEnable[PORT_A] = 0;
        BrickPi.MotorEnable[PORT_B] = 1;
        BrickPi.MotorEnable[PORT_C] = 1;
        BrickPi.MotorEnable[PORT_D] = 1;
        BrickPi.MotorSpeed[PORT_D] = BrickPi.Encoder[PORT_A];
        BrickPi.MotorSpeed[PORT_B] = BrickPi.Encoder[PORT_A];
        BrickPi.MotorSpeed[PORT_C] = BrickPi.Encoder[PORT_A];        
        printf("Encoder A: %6.1d\n", BrickPi.Encoder[PORT_A]);*/
      }
      usleep(10000);
    }
  }
  return 0;
}