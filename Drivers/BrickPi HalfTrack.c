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

#define DEBUG
//#define PRINT

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

// gcc -o program "/home/root/DB_MT/Raspberry Pi/C/BrickPi HalfTrack.c" -lrt -lm
// ./program

// gcc -o program "BrickPi HalfTrack.c" -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

// gcc -o program "/home/pi/dbrpi/C/BrickPi HalfTrack.c" -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

// gcc -o /home/pi/auto_start/auto "/home/pi/dbrpi/C/BrickPi HalfTrack.c" -lrt -lm -L/usr/local/lib -lwiringPi
// sudo /home/pi/auto_start/auto


#define MOTOR_PORT_LEFT  PORT_A
#define MOTOR_PORT_RIGHT PORT_B
#define MOTOR_PORT_STEER PORT_C

#define I2C_PORT             PORT_1    // I2C bus for the NXTChuck and PSP-Nx
#define I2C_SPEED            25        // I2C bus timing delay
#define I2C_DEVICE_PSP_NX    0         // PSP-Nx is device 1 on this I2C bus

#define PSP_BYTE_B1 0
#define PSP_BYTE_B2 1
#define PSP_BYTE_LX 2
#define PSP_BYTE_LY 3
#define PSP_BYTE_RX 4
#define PSP_BYTE_RY 5

int Shutdown = 0;
int tries = 0;
int result;

int main() {
  ClearTick();

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;
  
  BrickPi.Timeout = 100;                       // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

  tries = 0;
  result = 1;
  while(result){
    result = BrickPiSetup();
#ifdef DEBUG
    printf("BrickPiSetup: %d\n", result);
#endif
    if(result)
      tries++;
    if(tries >= 5)
      return 0;
  }  
  
  BrickPi.SensorType       [PORT_1] = TYPE_SENSOR_TOUCH;
  BrickPi.SensorType       [PORT_2] = TYPE_SENSOR_TOUCH;
  BrickPi.SensorType       [PORT_3] = TYPE_SENSOR_TOUCH;
  BrickPi.SensorType       [PORT_4] = TYPE_SENSOR_TOUCH;

  BrickPi.SensorType       [I2C_PORT] = TYPE_SENSOR_I2C_9V;
  BrickPi.SensorI2CSpeed   [I2C_PORT] = I2C_SPEED;
  BrickPi.SensorI2CDevices [I2C_PORT] = 1;  
  BrickPi.SensorSettings   [I2C_PORT][I2C_DEVICE_PSP_NX]   = BIT_I2C_SAME;  
  BrickPi.SensorI2CAddr    [I2C_PORT][I2C_DEVICE_PSP_NX]   = 0x02;
  
  BrickPi.SensorI2CWrite [I2C_PORT][I2C_DEVICE_PSP_NX]    = 1;
  BrickPi.SensorI2CRead  [I2C_PORT][I2C_DEVICE_PSP_NX]    = 6;
  BrickPi.SensorI2COut   [I2C_PORT][I2C_DEVICE_PSP_NX][0] = 0x42;
  
  tries = 0;
  result = 1;
  while(result){  
    result = BrickPiSetupSensors();
#ifdef DEBUG
    printf("BrickPiSetupSensors: %d\n", result);
#endif
    if(result)
      tries++;
    if(tries >= 5)
      return 0;
  }
 
/*
  Reset encoders at program start

  tries = 0;
  result = 1;
  while(result){  
    result = BrickPiUpdateValues();
#ifdef PRINT
    printf("BrickPiUpdateValues: %d\n", result);
#endif
    if(result)
      tries++;
    if(tries >= 5)
      return 0;
  }

  BrickPi.EncoderOffset[PORT_A] = BrickPi.Encoder[PORT_A];
  BrickPi.EncoderOffset[PORT_B] = BrickPi.Encoder[PORT_B];
  BrickPi.EncoderOffset[PORT_C] = BrickPi.Encoder[PORT_C];
  BrickPi.EncoderOffset[PORT_D] = BrickPi.Encoder[PORT_D];

  tries = 0;
  result = 1;
  while(result){  
    result = BrickPiUpdateValues();
#ifdef PRINT
    printf("BrickPiUpdateValues: %d\n", result);
#endif
    if(result)
      tries++;
    if(tries >= 5)
      return 0;
  }*/  

  BrickPi.MotorEnable[MOTOR_PORT_LEFT ] = TYPE_MOTOR_SPEED;
  BrickPi.MotorEnable[MOTOR_PORT_RIGHT] = TYPE_MOTOR_SPEED;
  BrickPi.MotorEnable[MOTOR_PORT_STEER] = TYPE_MOTOR_POSITION;  
  
  while(1){
    result = BrickPiUpdateValues();
    if(Shutdown >= 100){
      system("sudo shutdown -h now");
      return 0;
    }
    if(!result){
#ifdef PRINT
      printf("Results: %.1d\n", BrickPi.Sensor[I2C_PORT]);
//      printf("Encoder A: %6.1d  B: %6.1d  C: %6.1d  D: %6.1d\n", BrickPi.Encoder[PORT_A], BrickPi.Encoder[PORT_B], BrickPi.Encoder[PORT_C], BrickPi.Encoder[PORT_D]);
#endif
      if(BrickPi.Sensor[I2C_PORT] & (0x01 << I2C_DEVICE_PSP_NX)){
#ifdef PRINT
        printf("%3.1d %3.1d %3.1d %3.1d %3.1d %3.1d\n", BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][0], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][1], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][2], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][3], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][4], BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][5]);
#endif
        if(((~BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][0]) & 0x01) && ((~BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][0]) & 0x08)){
          Shutdown++;
        }else{
          Shutdown = 0;
        }
        if(Shutdown >= 100){
          BrickPi.MotorEnable[MOTOR_PORT_LEFT ] = TYPE_MOTOR_FLOAT;
          BrickPi.MotorEnable[MOTOR_PORT_RIGHT] = TYPE_MOTOR_FLOAT;
          BrickPi.MotorEnable[MOTOR_PORT_STEER] = TYPE_MOTOR_FLOAT;
        }else{
          /*          
            Good to go...          
          */
          if(!(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][PSP_BYTE_B2] & 0x10)){        // Set the current position to home
            BrickPi.EncoderOffset[MOTOR_PORT_STEER] = BrickPi.Encoder[MOTOR_PORT_STEER];
          }else if(!(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][PSP_BYTE_B2] & 0x80)){  // Offset the encoder by 1
            BrickPi.EncoderOffset[MOTOR_PORT_STEER] = 1;
          }else if(!(BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][PSP_BYTE_B2] & 0x20)){  // Offset the encoder by -1
            BrickPi.EncoderOffset[MOTOR_PORT_STEER] = -1;
          }
          
          BrickPi.MotorEnable  [MOTOR_PORT_LEFT ] = TYPE_MOTOR_SPEED;
          BrickPi.MotorEnable  [MOTOR_PORT_RIGHT] = TYPE_MOTOR_SPEED;
          BrickPi.MotorEnable  [MOTOR_PORT_STEER] = TYPE_MOTOR_POSITION;
          
          float Speed = (BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][PSP_BYTE_LY] * 0.78125) - 100; // -100 to 100
          float Steer = 100 - (BrickPi.SensorI2CIn[I2C_PORT][I2C_DEVICE_PSP_NX][PSP_BYTE_RX] * 0.78125); // -100 to 100
          float SpeedLeft  = (Speed * 2.55);
          float SpeedRight = (Speed * 2.55);
          
          if(Steer > 0){
            SpeedLeft  /= (( Steer / 50) + 1);
          }else if(Steer < 0){
            SpeedRight /= ((-Steer / 50) + 1);
          }
          
          BrickPi.MotorSpeed [MOTOR_PORT_LEFT ] = SpeedLeft;
          BrickPi.MotorSpeed [MOTOR_PORT_RIGHT] = SpeedRight;
          
          BrickPi.MotorTarget[MOTOR_PORT_STEER] = Steer * 2.75;
#ifdef PRINT
          printf("Encoder: %d\n", BrickPi.Encoder[MOTOR_PORT_STEER]);
#endif
        }
      }else{
        Shutdown = 0;
        BrickPi.MotorEnable[MOTOR_PORT_LEFT ] = TYPE_MOTOR_FLOAT;
        BrickPi.MotorEnable[MOTOR_PORT_RIGHT] = TYPE_MOTOR_FLOAT;
        BrickPi.MotorEnable[MOTOR_PORT_STEER] = TYPE_MOTOR_FLOAT;
      }
    }else{
      Shutdown = 0;
    }
    usleep(10000);
  }
  return 0;
}