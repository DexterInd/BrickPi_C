/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: Dec  11, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*/

#ifndef __BrickPi_h_
#define __BrickPi_h_

#ifndef NUMBER_OF_BRICKPIS
  #define NUMBER_OF_BRICKPIS 1
#endif

#define HOST_RPI 1
#define HOST_BBB 2

#ifndef COMPILE_HOST
  #define COMPILE_HOST 0
#endif

//#define DEBUG

#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <string.h> 
#include <stdio.h>  
#include <linux/i2c-dev.h>  

#if COMPILE_HOST == HOST_RPI
  #include <wiringPi.h>
#endif

#ifndef delay
  #define delay(x) usleep(x * 1000)
#endif
#ifndef Max
  #define Max(x, y) (x>y?x:y)                 // Return the highest value
#endif
#ifndef Min
  #define Min(x, y) (x<y?x:y)                 // Return the lowest value
#endif
#ifndef Clip
  #define Clip(v, x, y) (Min(Max(v, x), y))   // Return the value, clipped to x <= v <= y
#endif
#ifndef Dead
  #define Dead(v, x, y) ((v<=x)?v:(v>=y?v:0)) // Return the value, with a dead spot from x to y
#endif
#ifndef Abs
  #define Abs(v) (v<0?(-v):v)                 // Return the absolute value - untested
#endif

#define LED_1   0
#define LED_2   1
#define LED_ON  1023
#define LED_OFF 0

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

#define TYPE_MOTOR_FLOAT     0    // Float motor
#define TYPE_MOTOR_SPEED     1    // Motor speed control
#define TYPE_MOTOR_POSITION  2    // Motor position control
  #define MOTOR_KP_DEFAULT   2.0  // Motor position control - Proportional Konstant
  #define MOTOR_KD_DEFAULT   5.0  // Motor position control - Derivative Konstant
  #define MOTOR_DEAD_DEFAULT 10   // A dead-spot in the active motor control. For speeds in the range of -MOTOR_DEAD_DEFAULT to MOTOR_DEAD_DEFAULT, the motor won't run. Outside that range, the motor value is then increased (from 0) by MOTOR_DEAD_DEFAULT.

#define PORT_1 0
#define PORT_2 1
#define PORT_3 2
#define PORT_4 3

#define MASK_D0_M 0x01
#define MASK_D1_M 0x02
#define MASK_9V   0x04
#define MASK_D0_S 0x08
#define MASK_D1_S 0x10

#define BYTE_MSG_TYPE               0 // MSG_TYPE is the first byte.
  #define MSG_TYPE_CHANGE_ADDR      1 // Change the UART address.
  #define MSG_TYPE_SENSOR_TYPE      2 // Change/set the sensor type.
  #define MSG_TYPE_VALUES           3 // Set the motor speed and direction, and return the sesnors and encoders.
  #define MSG_TYPE_E_STOP           4 // Float motors immidately
  #define MSG_TYPE_TIMEOUT_SETTINGS 5 // Set the timeout
  #define MSG_TYPE_BAUD_SETTINGS    6 // Set the baud rate

  // New UART address (MSG_TYPE_CHANGE_ADDR)
    #define BYTE_NEW_ADDRESS     1
  
  // Sensor setup (MSG_TYPE_SENSOR_TYPE)
    #define BYTE_SENSOR_1_TYPE   1
    #define BYTE_SENSOR_2_TYPE   2
  
  // Timeout setup (MSG_TYPE_TIMEOUT_SETTINGS)
    #define BYTE_TIMEOUT 1

#define TYPE_SENSOR_RAW                0 // - 31
#define TYPE_SENSOR_LIGHT_OFF          0
#define TYPE_SENSOR_LIGHT_ON           (MASK_D0_M | MASK_D0_S)
#define TYPE_SENSOR_TOUCH              32
#define TYPE_SENSOR_ULTRASONIC_CONT    33
#define TYPE_SENSOR_ULTRASONIC_SS      34
#define TYPE_SENSOR_RCX_LIGHT          35 // tested minimally
#define TYPE_SENSOR_COLOR_FULL         36
#define TYPE_SENSOR_COLOR_RED          37
#define TYPE_SENSOR_COLOR_GREEN        38
#define TYPE_SENSOR_COLOR_BLUE         39
#define TYPE_SENSOR_COLOR_NONE         40
#define TYPE_SENSOR_I2C                41
#define TYPE_SENSOR_I2C_9V             42

#define BIT_I2C_MID  0x01  // Do one of those funny clock pulses between writing and reading. defined for each device.
#define BIT_I2C_SAME 0x02  // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

#define INDEX_RED   0
#define INDEX_GREEN 1
#define INDEX_BLUE  2
#define INDEX_BLANK 3

#define BAUD_DEFAULT 9600

int SW_HOST = 0;
unsigned long BAUD_IDEAL = BAUD_DEFAULT;   // This will be changed, specific to the host.

int RPiRev = 0; // If the host is a RPi, this will be set to the HW revision (1 or 2).

int BrickPiSetLed(unsigned char led, int value);
void BrickPiUpdateLEDs(void);
void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]);

// BrickPi data struct
struct BrickPiStruct{
/*
  LEDs
*/
  int LED                              [2];        // The state of the two LEDs

  unsigned char Address                [NUMBER_OF_BRICKPIS * 2];        // Communication addresses
  unsigned long Timeout                                        ;        // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

/*
  Motors
*/
  int           MotorSpeed             [NUMBER_OF_BRICKPIS * 4];        // Motor speeds, from -255 to 255
  unsigned char MotorEnable            [NUMBER_OF_BRICKPIS * 4];        // Motor mode. Float, Speed, Position.
  long          MotorTarget            [NUMBER_OF_BRICKPIS * 4];        // Motor target position. This is implemented on the RPi, not in the BrickPi FW.
  long          MotorTargetLastError   [NUMBER_OF_BRICKPIS * 4];        // Value used internally for motor position regulation.
  float         MotorTargetKP          [NUMBER_OF_BRICKPIS * 4];        // Percent Konstant - used for motor position regulation.
  float         MotorTargetKD          [NUMBER_OF_BRICKPIS * 4];        // Derivative Konstant - used for motor position regulation.
  unsigned char MotorDead              [NUMBER_OF_BRICKPIS * 4];        // How wide of a gap to leave between 0 and the value speed value used for running to a target position.

/*
  Encoders
*/
  long          EncoderOffset          [NUMBER_OF_BRICKPIS * 4];        // Encoder offsets
  long          Encoder                [NUMBER_OF_BRICKPIS * 4];        // Encoder values

/*
  Sensors
*/
  long          Sensor                 [NUMBER_OF_BRICKPIS * 4];        // Primary sensor values
  long          SensorArray            [NUMBER_OF_BRICKPIS * 4][4];     // For more sensor values for the sensor (e.g. for color sensor FULL mode).
  unsigned char SensorType             [NUMBER_OF_BRICKPIS * 4];        // Sensor types
  unsigned char SensorSettings         [NUMBER_OF_BRICKPIS * 4][8];     // Sensor settings, used for specifying I2C settings.

/*
  I2C
*/
  unsigned char SensorI2CDevices       [NUMBER_OF_BRICKPIS * 4];        // How many I2C devices are on each bus (1 - 8).
  unsigned char SensorI2CSpeed         [NUMBER_OF_BRICKPIS * 4];        // The I2C speed.
  unsigned char SensorI2CAddr          [NUMBER_OF_BRICKPIS * 4][8];     // The I2C address of each device on each bus.  
  unsigned char SensorI2CWrite         [NUMBER_OF_BRICKPIS * 4][8];     // How many bytes to write
  unsigned char SensorI2CRead          [NUMBER_OF_BRICKPIS * 4][8];     // How many bytes to read
  unsigned char SensorI2COut           [NUMBER_OF_BRICKPIS * 4][8][16]; // The I2C bytes to write
  unsigned char SensorI2CIn            [NUMBER_OF_BRICKPIS * 4][8][16]; // The I2C input buffers
};

struct BrickPiStruct BrickPi;

unsigned char Array[256];
unsigned char BytesReceived;

// Tell the BrickPi to float all motors immidately
int BrickPiEmergencyStop(){
/*
  Try 3 times to send E Stop to each of the uCs.
  If failed:
    Broadcast E Stop 3 times.
*/
  
  unsigned char i = 0;
  while(i < 3){
    unsigned char ii = 0;
    while(ii < (NUMBER_OF_BRICKPIS * 2)){
      Array[BYTE_MSG_TYPE] = MSG_TYPE_E_STOP;
      BrickPiTx(BrickPi.Address[ii], 1, Array);
      if(BrickPiRx(&BytesReceived, Array, 5000)){
        goto NEXT_TRY;
      }
      if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_E_STOP)){
        goto NEXT_TRY;
      }
      if(ii == ((NUMBER_OF_BRICKPIS * 2) - 1)){
        return 0;
      }
      ii++;
    }
NEXT_TRY:
    i++;
  }
  
  i = 0;
  while(i < 3){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_E_STOP;
    BrickPiTx(0, 1, Array);
    usleep(5000);
    i++;
  }
  return -1;
}

// Change the BrickPi address, for one of the uCs
int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr){
  Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR;
  Array[BYTE_NEW_ADDRESS] = NewAddr;
  BrickPiTx(OldAddr, 2, Array);
  
  if(BrickPiRx(&BytesReceived, Array, 5000))
    return -1;
  if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR))
    return -1;
  
  return 0;
}

// Change the BrickPi timeout
int BrickPiSetTimeout(){
  unsigned char i = 0;
  while(i < (NUMBER_OF_BRICKPIS * 2)){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS;
    Array[ BYTE_TIMEOUT     ] = ( BrickPi.Timeout             & 0xFF);
    Array[(BYTE_TIMEOUT + 1)] = ((BrickPi.Timeout / 256     ) & 0xFF);
    Array[(BYTE_TIMEOUT + 2)] = ((BrickPi.Timeout / 65536   ) & 0xFF);
    Array[(BYTE_TIMEOUT + 3)] = ((BrickPi.Timeout / 16777216) & 0xFF);
    BrickPiTx(BrickPi.Address[i], 5, Array);
    if(BrickPiRx(&BytesReceived, Array, 5000))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS))
      return -1;
    i++;
  }
  return 0;
}

// Tell the BrickPi to use a new baud rate
int BrickPiSetBaud(unsigned long baud_old, unsigned long baud_new){
  unsigned char result = 0;
  int i = 0;
  while(i < (NUMBER_OF_BRICKPIS * 2)){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_BAUD_SETTINGS;
    Array[ BYTE_TIMEOUT     ] = ( baud_new             & 0xFF);
    Array[(BYTE_TIMEOUT + 1)] = ((baud_new / 256     ) & 0xFF);
    Array[(BYTE_TIMEOUT + 2)] = ((baud_new / 65536   ) & 0xFF);
    
    UART_Configure(baud_old);
    BrickPiTx(BrickPi.Address[i], 4, Array);
    UART_Configure(baud_new);
    
    if(BrickPiRx(&BytesReceived, Array, 5000))
      result |= (0x01 << i);
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_BAUD_SETTINGS))
      result |= (0x01 << i);
    
    i++;
  }
  
  if(result){                         // If it failed, then it could be the original BrickPi FW which only supports 500k Baud.
    UART_Configure(baud_old);
    if(BrickPiSetTimeout() == -1){    // Try setting the timeout, which will determine if communication is successful at the desired baud rate.
      return result;                  // If setting the timeout also failed, return the error.
    }
  }                                   
  return 0;                           // Else return 0 (no error).
}

unsigned int Bit_Offset = 0;

// Add "bits" number of bits of "value" to "Array"
void AddBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits, unsigned long value){
  unsigned char i = 0;
  while(i < bits){
    if(value & 0x01){
      Array[(byte_offset + ((bit_offset + Bit_Offset + i) / 8))] |= (0x01 << ((bit_offset + Bit_Offset + i) % 8));
    }
    value /= 2;
    i++;
  }
  Bit_Offset += bits;
}

// Extract "bits" number of bits from "Array"
unsigned long GetBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits){
  unsigned long Result = 0;
  char i = bits;
  while(i){
    Result *= 2;
    Result |= ((Array[(byte_offset + ((bit_offset + Bit_Offset + (i - 1)) / 8))] >> ((bit_offset + Bit_Offset + (i - 1)) % 8)) & 0x01);    
    i--;
  }
  Bit_Offset += bits;
  return Result;
}

// Determine how many bits are needed to store the value
unsigned char BitsNeeded(unsigned long value){
  unsigned char i = 0;
  while(i < 32){
    if(!value)
      return i;
    value /= 2;
    i++;
  }
  return 31;
}

// Configure sensors
int BrickPiSetupSensors(){
  unsigned char i = 0;
  while(i < (NUMBER_OF_BRICKPIS * 2)){
    int ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    Bit_Offset = 0;
    Array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE;
    Array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1 + (i * 2)];
    Array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2 + (i * 2)];
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C
      || Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C_9V){
        AddBits(3, 0, 8, BrickPi.SensorI2CSpeed[port]);
        
        if(BrickPi.SensorI2CDevices[port] > 8)
          BrickPi.SensorI2CDevices[port] = 8;
        
        if(BrickPi.SensorI2CDevices[port] == 0)
          BrickPi.SensorI2CDevices[port] = 1;
        
        AddBits(3, 0, 3, (BrickPi.SensorI2CDevices[port] - 1));
        
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          AddBits(3, 0, 7, (BrickPi.SensorI2CAddr[port][device] >> 1));
          AddBits(3, 0, 2, BrickPi.SensorSettings[port][device]);
          if(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME){          
            AddBits(3, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(3, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(3, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 3);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    if(BrickPiRx(&BytesReceived, Array, 1000000))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
      return -1;
    i++;
  }
  return 0;
}


unsigned char Retried = 0; // For re-trying a failed update.

// Update the BrickPi, and get the latest values
int BrickPiUpdateValues(){
  BrickPiUpdateLEDs();
  
  unsigned char i = 0;
  unsigned int ii = 0;
  while(i < (NUMBER_OF_BRICKPIS * 2)){
    Retried = 0;    
    
    __RETRY_COMMUNICATION__:
    
    ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    
    Array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES;
    
    Bit_Offset = 0;
    
//    AddBits(1, 0, 2, 0);     use this to disable encoder offset
    
    ii = 0;                 // use this for encoder offset support
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.EncoderOffset[port]){
        long Temp_Value = BrickPi.EncoderOffset[port];
        unsigned char Temp_ENC_DIR = 0;
        unsigned char Temp_BitsNeeded = 0;
        
        AddBits(1, 0, 1, 1);
        if(Temp_Value < 0){
          Temp_ENC_DIR = 1;
          Temp_Value *= (-1);
        }        
        Temp_BitsNeeded = BitsNeeded(Temp_Value);
        AddBits(1, 0, 5, Temp_BitsNeeded);
        Temp_BitsNeeded++;
        Temp_Value *= 2;
        Temp_Value |= Temp_ENC_DIR;
        AddBits(1, 0, Temp_BitsNeeded, Temp_Value);
      }
      else{
        AddBits(1, 0, 1, 0);
      }
      ii++;
    }
    
    int speed;
    unsigned char dir;    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      
      if(BrickPi.MotorEnable[port] == TYPE_MOTOR_FLOAT){
        AddBits(1, 0, 10, 0);
      }else{
        if(BrickPi.MotorEnable[port] == TYPE_MOTOR_SPEED){
          speed = BrickPi.MotorSpeed[port];
        }else if(BrickPi.MotorEnable[port] == TYPE_MOTOR_POSITION){
          long error = BrickPi.MotorTarget[port] - BrickPi.Encoder[port];
          float speed_f = (error * BrickPi.MotorTargetKP[port]) + ((error - BrickPi.MotorTargetLastError[port]) * BrickPi.MotorTargetKD[port]);
          BrickPi.MotorTargetLastError[port] = error;
          if(speed_f < BrickPi.MotorDead[port] && speed_f > -BrickPi.MotorDead[port]){
            speed_f = 0;
          }
          if(speed_f > 0){
            speed_f += BrickPi.MotorDead[port];
          }else if(speed_f < 0){
            speed_f -= BrickPi.MotorDead[port];
          }
          speed = Clip(speed_f, -255, 255); // Clip the speed to the range of -255 to 255.
/*#ifdef DEBUG
          printf("Speed: %d\n", speed);        
#endif*/
        }
        
        dir = 0;
        if(speed < 0){
          dir = 1;
          speed *= (-1);
        }
        if(speed > 255){
          speed = 255;
        }
        AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (0x01)) & 0x3FF));
      }
      ii++;
    }
    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C
      || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V){
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          if(!(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME)){
            AddBits(1, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(1, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(1, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 1);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    usleep(500);
    int result = BrickPiRx(&BytesReceived, Array, 25000);
    
    if(result != -2){                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
      BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0;
      BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0;
    }
    
    if(result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES)){
#ifdef DEBUG
      printf("BrickPiRx error: %d\n", result);
#endif
      if(Retried < 4){
        Retried++;
        goto __RETRY_COMMUNICATION__;
      }
      else{
#ifdef DEBUG
        printf("Retry failed.\n");
#endif
        return -1;
      }      
    }
    
    Bit_Offset = 0;
    
    unsigned char Temp_BitsUsed[2] = {0, 0};         // Used for encoder values
    Temp_BitsUsed[0] = GetBits(1, 0, 5);
    Temp_BitsUsed[1] = GetBits(1, 0, 5);
    unsigned long Temp_EncoderVal;
    
    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii]);
      if(Temp_EncoderVal & 0x01){
        Temp_EncoderVal /= 2;
        BrickPi.Encoder[port] = Temp_EncoderVal * (-1);}
      else{
        BrickPi.Encoder[port] = (Temp_EncoderVal / 2);}
      ii++;
    }

    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      switch(BrickPi.SensorType[port]){
        case TYPE_SENSOR_TOUCH:
          BrickPi.Sensor[port] = GetBits(1, 0, 1);
        break;
        case TYPE_SENSOR_ULTRASONIC_CONT:
        case TYPE_SENSOR_ULTRASONIC_SS:
          BrickPi.Sensor[port] = GetBits(1, 0, 8);
        break;
        case TYPE_SENSOR_COLOR_FULL:
          BrickPi.Sensor[port] = GetBits(1, 0, 3);
          BrickPi.SensorArray[port][INDEX_BLANK] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_RED  ] = GetBits(1, 0, 10);                
          BrickPi.SensorArray[port][INDEX_GREEN] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_BLUE ] = GetBits(1, 0, 10);
        break;          
        case TYPE_SENSOR_I2C:
        case TYPE_SENSOR_I2C_9V:
          BrickPi.Sensor[port] = GetBits(1, 0, BrickPi.SensorI2CDevices[port]);
          unsigned char device = 0;
          while(device < BrickPi.SensorI2CDevices[port]){
            if(BrickPi.Sensor[port] & (0x01 << device)){
              unsigned char in_byte = 0;
              while(in_byte < BrickPi.SensorI2CRead[port][device]){
                BrickPi.SensorI2CIn[port][device][in_byte] = GetBits(1, 0, 8);
                in_byte++;
              }
            }
            device++;
          }
        break;      
        case TYPE_SENSOR_LIGHT_OFF:
        case TYPE_SENSOR_LIGHT_ON:
        case TYPE_SENSOR_RCX_LIGHT:
        case TYPE_SENSOR_COLOR_RED:
        case TYPE_SENSOR_COLOR_GREEN:
        case TYPE_SENSOR_COLOR_BLUE:
        case TYPE_SENSOR_COLOR_NONE:
        default:
          BrickPi.Sensor[(ii + (i * 2))] = GetBits(1, 0, 10);
      }        
      ii++;
    }      
    i++;
  }       
  return 0;
}

int I2C_file_descriptor = -1;

int I2C_WriteArray(unsigned char addr, unsigned char ByteCount, unsigned char OutArray[]){
  if (ioctl(I2C_file_descriptor, I2C_SLAVE, (addr >> 1)) < 0)
    return -1;
  if(write(I2C_file_descriptor, OutArray, ByteCount) != 1)  
    return -2;
  return 0;
}

int I2C_ReadArray(unsigned char addr, unsigned char ByteCount, unsigned char *InArray){
  if (ioctl(I2C_file_descriptor, I2C_SLAVE, (addr >> 1)) < 0)
    return -1;
  if (read(I2C_file_descriptor, InArray, ByteCount) != 1)  
    return -2;  
  return 0;
}

int I2C_WriteReadArray(unsigned char addr, unsigned char OutByteCount, unsigned char OutArray[], unsigned char InByteCount, unsigned char *InArray){
  if (ioctl(I2C_file_descriptor, I2C_SLAVE, (addr >> 1)) < 0)
    return -1;
  if(write(I2C_file_descriptor, OutArray, OutByteCount) != OutByteCount)  
    return -2;
  if (read(I2C_file_descriptor, InArray, InByteCount) != InByteCount)  
    return -3;
  return 0; 
}

int LED_1_value_file_descriptor = -1;
int LED_2_value_file_descriptor = -1;

// Set the state of one of the LEDs
int BrickPiSetLed(unsigned char led, int value){
  if(led > LED_2){
    return -1;
  }
  BrickPi.LED[led] = value;
  BrickPiUpdateLEDs();
  return 0;
}

// Update the LEDs
void BrickPiUpdateLEDs(){
#if COMPILE_HOST == HOST_RPI
  pwmWrite    (1,  BrickPi.LED[LED_1]     );     // Set the PWM of LED 1 (0-1023)
  digitalWrite(2, (BrickPi.LED[LED_2]?1:0));     // Set the state of LED 2
#else
  if(BrickPi.LED[LED_1])
    write(LED_1_value_file_descriptor, "1", 1);
  else
    write(LED_1_value_file_descriptor, "0", 1);
  if(BrickPi.LED[LED_2])
    write(LED_2_value_file_descriptor, "1", 1);
  else
    write(LED_2_value_file_descriptor, "0", 1);
#endif
}

int UART_file_descriptor = -1; 

/*
  To safely shutdown the program, use:
    sudo killall program -s 2
  which sends signal 2 to process "program"
*/

// Turns off the LEDs, closes the open files, and exits the program
void BrickPiExitSafely(int sig)                  // Exit the program safely
{
  signal(SIGINT , SIG_IGN);                      // Disable the signal interrupt
  signal(SIGQUIT, SIG_IGN);                      // Disable the signal interrupt
#ifdef DEBUG
  printf("\nReceived exit signal %d\n", sig);    // Tell the user why the program is exiting
#endif
  BrickPiEmergencyStop();                        // Send E Stop to the BrickPi
  
  close(I2C_file_descriptor);
  I2C_file_descriptor = -1;
  
#if COMPILE_HOST == HOST_RPI
  pwmWrite    (1, 0);                            // Set the PWM of LED 1 to 0
  digitalWrite(2, 0);                            // Set the state of LED 2 to 0
  pinMode(1, INPUT);                             // Set LED 1 IO as INPUT
  pinMode(2, INPUT);                             // Set LED 2 IO as INPUT
#else 
  write(LED_1_value_file_descriptor, "0", 1);
  write(LED_2_value_file_descriptor, "0", 1);
  close(LED_1_value_file_descriptor);
  close(LED_2_value_file_descriptor);
  LED_1_value_file_descriptor = -1;
  LED_2_value_file_descriptor = -1;
  if(SW_HOST == HOST_RPI){
    system("echo in > /sys/class/gpio/gpio18/direction");    // Set GPIO as INPUT    
    system("echo 18 > /sys/class/gpio/unexport");            // Unexport the GPIO
    if(RPiRev == 1){
      system("echo in > /sys/class/gpio/gpio21/direction");    // Set GPIO as INPUT
      system("echo 21 > /sys/class/gpio/unexport");            // Unexport the GPIO
    }else{
      system("echo in > /sys/class/gpio/gpio27/direction");    // Set GPIO as INPUT
      system("echo 27 > /sys/class/gpio/unexport");            // Unexport the GPIO
    }
  }else if(SW_HOST == HOST_BBB){
    system("echo in > /sys/class/gpio/gpio50/direction");    // Set GPIO as INPUT
    system("echo in > /sys/class/gpio/gpio51/direction");    // Set GPIO as INPUT
    system("echo 50 > /sys/class/gpio/unexport");            // Unexport the GPIO
    system("echo 51 > /sys/class/gpio/unexport");            // Unexport the GPIO
  }  
#endif
  close(UART_file_descriptor);                   // Close the UART port
  UART_file_descriptor = -1;
  
//  signal(SIGINT , BrickPiExitSafely);  Don't bother to re-enable
//  signal(SIGQUIT, BrickPiExitSafely);  Don't bother to re-enable
#ifdef DEBUG
  printf("Exiting.\n");                          // Tell the user that the program is exiting
#endif
  exit(0);                                       // Exit
}

// Determine if -name- exists in -directory-
// Returns:
//   -1 error opening directory
//    0 not found
//    1 found
int FileExists(const char *directory, const char *name)
{
  DIR *dirp;
  struct dirent *dp;

  dirp = opendir(directory);
  if (dirp == NULL)
    return -1;
  
  while(1){
    dp = readdir(dirp);
    if(dp == NULL){
      closedir(dirp);
      return 0;
    }else{
      if (strcmp(dp->d_name, name) == 0){
        closedir(dirp);
        return 1;
      }
    }
  }
}

unsigned long BAUD_RATES[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000, 1500000, 2000000, 3000000};

// Convert from e.g. 9600 to B9600
long BaudCompute(unsigned long baud){
  switch (baud)
  {
    case      50: return      B50;
    case      75: return      B75;
    case     110: return     B110;
    case     134: return     B134;
    case     150: return     B150;
    case     200: return     B200;
    case     300: return     B300;
    case     600: return     B600;
    case    1200: return    B1200;
    case    1800: return    B1800;
    case    2400: return    B2400;
    case    4800: return    B4800;
    case    9600: return    B9600;
    case   19200: return   B19200;
    case   38400: return   B38400;
    case   57600: return   B57600;
    case  115200: return  B115200;
    case  230400: return  B230400;
    case  460800: return  B460800;
    case  500000: return  B500000;
    case  921600: return  B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    default:
      return -1;
  }
}

unsigned long BaudRate;

// Configure the local UART
int UART_Configure(unsigned long baud){
  long result = BaudCompute(baud);
  if(result == -1)
    return -1;
  
  BaudRate = baud;
  
  struct termios options;
  int     status;  
  fcntl (UART_file_descriptor, F_SETFL, O_RDWR);

// Get and modify current options:
  tcgetattr (UART_file_descriptor, &options);

  cfmakeraw   (&options);
  cfsetispeed (&options, result);
  cfsetospeed (&options, result);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  options.c_cc [VMIN]  =  0;
  options.c_cc [VTIME] = 10; // One second (10 deciseconds) // MT was 100

  tcsetattr (UART_file_descriptor, TCSANOW | TCSAFLUSH, &options);

  ioctl (UART_file_descriptor, TIOCMGET, &status);

  status |= TIOCM_DTR;
  status |= TIOCM_RTS;

  ioctl (UART_file_descriptor, TIOCMSET, &status);
  return 0;
}

// Attempt to force the BrickPi to use a specific baud rate
int BrickPiForceBaud(unsigned long baud){
  int i = 0;
  while(i < 15){
    if(BaudCompute(BAUD_RATES[i]) != -1){
      if(BrickPiSetBaud(BAUD_RATES[i], baud) == 0){
        return 0;
      }
      usleep(10000);
    }
    i++;
  }
  return -1;
}

// GetRPiRev() // Figure out the RPi hardware revision
unsigned int GetRPiRev(){
  FILE * filp;
  unsigned rev;
  char buf[512];
  char term;
  rev = 0;
  filp = fopen ("/proc/cpuinfo", "r");
  if (filp != NULL){
    while (fgets(buf, sizeof(buf), filp) != NULL){
      if (!strncasecmp("revision\t", buf, 9)){
        if (sscanf(buf+strlen(buf)-5, "%x%c", &rev, &term) == 2){
          if (term == '\n') break;
          rev = 0;
        }
      }
    }
    fclose(filp);
  }
  return ((rev>=4)?2:1);
}

int StringFind(char *src, char *fnd){
  unsigned char i = 0;
  unsigned char ii = 0;
  while(src[i] != 0){
    while(src[i + ii] == fnd[ii]){
      ii++;
      if(fnd[ii] == 0){
        return i;
      }
    }
    ii = 0;
    i++;
  }
  return -1;
}

int Enable_ttyO4(){
//  system("echo enable-uart5 > /sys/devices/bone_capemgr.*/slots");

  char name[40];                 // Should only need 33 + NULL terminator = 34
  strcpy(name, "/sys/devices");
  // open /sys/devices
  DIR *dirp;
  dirp = opendir(name);
  if(dirp == NULL){
    return 0;
  }
  
  // look for bone_capemgr.*
  struct dirent *dp;
  unsigned char done = 0;
  while(!done){
    dp = readdir(dirp);
    if(dp == NULL){
      closedir(dirp);
      return 0;
    }else{
      if(StringFind(dp->d_name, "bone_capemgr.") == 0){    // sometimes it's "bone_capemgr.8" and sometimes it's "bone_capemgr.9"
        closedir(dirp);
        strcat(name, "/");                                 // append "/" to name
        strcat(name, dp->d_name);                          // append dp->d_name to name
        done = 1;
      }
    }
  }
  
  strcat(name, "/slots"); // append "/slots" to name
  
  // open the file with the exact directory name  
  int fd = open(name, O_RDWR);
  if(fd == -1){   
    return 0;
  }
  
  // write "enable-uart5"
  #ifdef DEBUG
    printf("enable-uart5 > %s\n", name);
  #endif  
  write(fd, "enable-uart5", 12);
  close(fd);
  return 1;
}

int Enable_i2c_1(){
//  system("echo BB-I2C1 > /sys/devices/bone_capemgr.*/slots");

  char name[40];                 // Should only need 33 + NULL terminator = 34
  strcpy(name, "/sys/devices");
  // open /sys/devices
  DIR *dirp;
  dirp = opendir(name);
  if(dirp == NULL){
    return 0;
  }
  
  // look for bone_capemgr.*
  struct dirent *dp;
  unsigned char done = 0;
  while(!done){
    dp = readdir(dirp);
    if(dp == NULL){
      closedir(dirp);
      return 0;
    }else{
      if(StringFind(dp->d_name, "bone_capemgr.") == 0){    // sometimes it's "bone_capemgr.8" and sometimes it's "bone_capemgr.9"
        closedir(dirp);
        strcat(name, "/");                                 // append "/" to name
        strcat(name, dp->d_name);                          // append dp->d_name to name
        done = 1;
      }
    }
  }
  
  strcat(name, "/slots"); // append "/slots" to name
  
  // open the file with the exact directory name  
  int fd = open(name, O_RDWR);
  if(fd == -1){   
    return 0;
  }
  
  // write "BB-I2C1"
  #ifdef DEBUG
    printf("BB-I2C1 > %s\n", name);
  #endif  
  write(fd, "BB-I2C1", 7);
  close(fd);
  return 1;
}

// Determine the SW host. If it's RPI, then also enable i2c if necessary (every time it boots). If it's BBB, then also enable ttyO4 and i2c-1 if necessary (every time it boots).
int Get_SW_HOST(){
  // Determine SW_HOST
  #if ((COMPILE_HOST == HOST_RPI) || (COMPILE_HOST == HOST_BBB))
    SW_HOST = COMPILE_HOST;
  #else
    #ifdef DEBUG
      printf("Determining SW_HOST...\n");
    #endif
    SW_HOST = 0;
    // Determine by SW which host the program is running on (either RPi or BBB)
    if(FileExists("/dev", "ttyAMA0") == 1){
      SW_HOST = HOST_RPI;
      #ifdef DEBUG
        printf("SW_HOST = RPI\n");
      #endif
      if(FileExists("/dev", "i2c-0") != 1 || FileExists("/dev", "i2c-1") != 1){
        system("gpio load i2c 10");
      }
      #ifdef DEBUG
        if(FileExists("/dev", "i2c-0") != 1 || FileExists("/dev", "i2c-1") != 1){
          printf("Enable i2c: failure\n");
          return -1;
        }else{
          printf("Enable i2c: success\n");
        }
      #else
        if(FileExists("/dev", "i2c-0") != 1 || FileExists("/dev", "i2c-1") != 1){
          return -1;
        }
      #endif
    }else{
      if(FileExists("/dev", "ttyO4") == 1){
        SW_HOST = HOST_BBB;
        #ifdef DEBUG
          printf("SW_HOST = HOST_BBB\n");
        #endif
      }else{
        if(FileExists("/dev", "ttyO0") == 1){
          #ifdef DEBUG
            if(Enable_ttyO4() == 1){
              printf("Enable ttyO4: success\n");
            }else{
              printf("Enable ttyO4: failure\n");          
            }            
          #else
            Enable_ttyO4();
          #endif
          if(FileExists("/dev", "ttyO4") == 1){
            SW_HOST = HOST_BBB;
            #ifdef DEBUG
              printf("SW_HOST = HOST_BBB\n");
            #endif
          }else{
            return -1;
          }
        }
        else{
          return -1;
        }
      }
      if(SW_HOST == HOST_BBB){
        if(FileExists("/dev", "i2c-2") != 1){
          #ifdef DEBUG
            if(Enable_i2c_1() == 1){
              printf("Enable i2c-1: success\n");
            }else{
              printf("Enable i2c-1: failure\n");          
            }            
          #else
            Enable_i2c_1();
          #endif
          if(FileExists("/dev", "i2c-2") != 1){
            return -1;
          }
        }
      }
    }
  #endif

  // Print the SW_HOST
  #ifdef DEBUG  
    printf("SW_HOST = %d\n", SW_HOST);  
  #endif

  // Is SW_HOST supported?
  if(((SW_HOST != HOST_RPI) && (SW_HOST != HOST_BBB))){
    #ifdef DEBUG
      printf("SW_HOST not supported.\n");
    #endif
    return -1;
  }
  
  return 0;
}

int BrickPiSetupLEDs(){
  // Setup the LED GPIOs based on the COMPILE_HOST or SW_HOST
  #if COMPILE_HOST == HOST_RPI
    if(wiringPiSetup() == -1)                                     // If wiringPiSetup failed
      return -1;                                                  //   return -1  
    pinMode(1, PWM_OUTPUT);                                       // LED 1
    pinMode(2, OUTPUT);                                           // LED 2
  #else
    
    // If LED 1 file is already open, close it
    if(LED_1_value_file_descriptor != -1){
      close(LED_1_value_file_descriptor);
      LED_1_value_file_descriptor = -1;
    }
    // If LED 2 file is already open, close it
    if(LED_2_value_file_descriptor != -1){
      close(LED_2_value_file_descriptor);
      LED_2_value_file_descriptor = -1;
    }
    
    // Setup the LED GPIOs specific to the host platform
    if(SW_HOST == HOST_RPI){
      system("echo 18 > /sys/class/gpio/export");                   // Export the GPIO for use
      system("echo low > /sys/class/gpio/gpio18/direction");        // Enable the GPIO as OUTPUT and set LOW    
      LED_1_value_file_descriptor = open("/sys/class/gpio/gpio18/value", O_RDWR | O_CREAT);
      // Setup LED 2 specific to the RPi Revision
      if(RPiRev == 1){
        system("echo 21 > /sys/class/gpio/export");                   // Export the GPIO for use    
        system("echo low > /sys/class/gpio/gpio21/direction");        // Enable the GPIO as OUTPUT and set LOW
        LED_2_value_file_descriptor = open("/sys/class/gpio/gpio21/value", O_RDWR | O_CREAT);
      }else{        // RPiRev == 2
        system("echo 27 > /sys/class/gpio/export");                   // Export the GPIO for use    
        system("echo low > /sys/class/gpio/gpio27/direction");        // Enable the GPIO as OUTPUT and set LOW    
        LED_2_value_file_descriptor = open("/sys/class/gpio/gpio27/value", O_RDWR | O_CREAT);
      }   
    }else if(SW_HOST == HOST_BBB){
      system("echo 50 > /sys/class/gpio/export");                   // Export the GPIO for use
      system("echo low > /sys/class/gpio/gpio50/direction");        // Enable the GPIO as OUTPUT and set LOW  
      LED_1_value_file_descriptor = open("/sys/class/gpio/gpio50/value", O_RDWR | O_CREAT);  
      system("echo 51 > /sys/class/gpio/export");                   // Export the GPIO for use    
      system("echo low > /sys/class/gpio/gpio51/direction");        // Enable the GPIO as OUTPUT and set LOW    
      LED_2_value_file_descriptor = open("/sys/class/gpio/gpio51/value", O_RDWR | O_CREAT);
    }
    // If there was an error opening one of the LED GPIO files, then close the other.
    if(LED_1_value_file_descriptor == -1){
      if(LED_2_value_file_descriptor != -1){
        close(LED_2_value_file_descriptor);
        LED_2_value_file_descriptor = -1;
      }
      return -1;
    }else if(LED_2_value_file_descriptor == -1){
      close(LED_1_value_file_descriptor);
      LED_1_value_file_descriptor = -1;
      return -1;
    }
  #endif
  return 0;
}

int BrickPiSetupI2C(unsigned long speed){

  
  // If I2C file is already open, close it
  if(I2C_file_descriptor != -1){
    close(I2C_file_descriptor);
    I2C_file_descriptor = -1;
  }  
  
  // Setup the HW I2C based on the SW_HOST
  if(SW_HOST == HOST_RPI){
    if(speed < 5000)
      speed = 5000;
    if(speed > 1000000)
      speed = 1000000;
    char str[80];
    
    sprintf(str, "sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=%d", speed);
    system(str);
    
    // Setup HW I2C specific to the RPi Revision
    if(RPiRev == 1){
      I2C_file_descriptor = open("/dev/i2c-0", O_RDWR);
    }else{
      I2C_file_descriptor = open("/dev/i2c-1", O_RDWR);
    }
  }else if(SW_HOST == HOST_BBB){
    I2C_file_descriptor = open("/dev/i2c-2", O_RDWR);
  }

  if(I2C_file_descriptor == -1)
    return -1;
  return 0;
}

int BrickPiOpenUART(){
  // If UART port is open already, then close it
  if(UART_file_descriptor != -1){                  
    close(UART_file_descriptor);
    UART_file_descriptor = -1;
  }

  // Open the UART port specific to the host, and set BAUD_IDEAL accordingly
  if(SW_HOST == HOST_RPI){
    UART_file_descriptor = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    BAUD_IDEAL = 500000;
  }else if(SW_HOST == HOST_BBB){
    UART_file_descriptor = open ("/dev/ttyO4", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
//    UART_file_descriptor = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    BAUD_IDEAL = 115200;
  }  
  
  // If it failed to open the UART port
  if (UART_file_descriptor == -1){
    return -1;
  }
  return 0;
}

int BrickPiConfigBaud(){
  // Set the UART Baud rate to BAUD_IDEAL
  unsigned char i = 0;
  while(i < 5){    
    if(!BrickPiSetBaud(BAUD_IDEAL, BAUD_IDEAL))
      break;
    if(!BrickPiSetBaud(BAUD_DEFAULT, BAUD_IDEAL))
      break;
    if(!BrickPiSetBaud(BAUD_IDEAL, BAUD_IDEAL))
      break;
    if(!BrickPiForceBaud(BAUD_IDEAL))
      break;
    if(i == 4)
      return -1;
    i++;
  }
  return 0;
}

// Setup the host and the BrickPi. Determine the host, setup the LED GPIO, setup UART, set the BrickPi timeout, initialize the exit signal handlers.
int BrickPiSetup(){
  // Setup the exit signal handlers  
  if(signal(SIGINT , BrickPiExitSafely) == SIG_ERR ||           // Setup exit signal SIGINT
    signal(SIGQUIT, BrickPiExitSafely) == SIG_ERR){             // and SIGQUIT
    #ifdef DEBUG
      printf("Exit signal install error\n");
    #endif
    return -1;                                                  // and return -1
  }

  // Determine the SW_HOST  
  if(Get_SW_HOST()){
    return -2;
  }
  
  if(SW_HOST == HOST_RPI){
    RPiRev = GetRPiRev();
  }
  
  // Setup the LED IOs  
  if(BrickPiSetupLEDs()){
    return -3;
  }
  
  // Setup the HW I2C for sensor port 5
  if(BrickPiSetupI2C(100000)){
    return -4;
  }
  
  // Open the UART port  
  if(BrickPiOpenUART()){
    return -5;
  }
  
  // Set the BAUD rate to BAUD_IDEAL  
  if(BrickPiConfigBaud()){
    return -6;
  }
  
  // Set the BrickPi timeout
  if(BrickPiSetTimeout() == -1){
    return -7;
  }
  
  // Setup the motor defaults
  int i = 0;
  while(i < (NUMBER_OF_BRICKPIS * 4)){
    BrickPi.MotorTargetKP[i] = MOTOR_KP_DEFAULT;           // Set to default
    BrickPi.MotorTargetKD[i] = MOTOR_KD_DEFAULT;           //      ''
    BrickPi.MotorDead    [i] = MOTOR_DEAD_DEFAULT;         //      ''
    i++;
  }
  return 0;                                                // return 0
}

int BrickPiSetupAddress(unsigned char OldAddr, unsigned char NewAddr){
  // Setup the exit signal handlers  
  if(signal(SIGINT , BrickPiExitSafely) == SIG_ERR ||           // Setup exit signal SIGINT
    signal(SIGQUIT, BrickPiExitSafely) == SIG_ERR){             // and SIGQUIT
    #ifdef DEBUG
      printf("Exit signal install error\n");
    #endif
    return -1;                                                  // and return -1
  }

  // Determine the SW_HOST  
  if(Get_SW_HOST()){
    return -1;
  }
  
  // Setup the LED IOs  
  if(BrickPiSetupLEDs()){
    return -1;
  }
  
  // Open the UART port  
  if(BrickPiOpenUART()){
    return -1;
  }
  
  UART_Configure(BAUD_DEFAULT);  
  if(BrickPiChangeAddress(OldAddr, NewAddr)){
    UART_Configure(BAUD_IDEAL);
    if(BrickPiChangeAddress(OldAddr, NewAddr)){
      return -1;
    }
  }
  return 0;
}

// Send an array of data to the BrickPi. Trash any rx bytes, transmit the message, and wait until is is sent.
void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]){
  unsigned char tx_buffer[256];
  tx_buffer[0] = dest;
  tx_buffer[1] = dest + ByteCount;
  tx_buffer[2] = ByteCount;  
  unsigned char i = 0;
  while(i < ByteCount){
    tx_buffer[1] += OutArray[i];
    tx_buffer[i + 3] = OutArray[i];
    i++;
  }  
  ByteCount += 3;  
//  BrickPiSetLed(LED_1, 1);  
  BrickPiRxFlush();
  write(UART_file_descriptor, tx_buffer, ByteCount);  
  usleep((((1000000 * 10) / BaudRate) * ByteCount));  
//  BrickPiSetLed(LED_1, 0);
}

// Determine how many bytes are in the Rx buffer, ready to be read
int BrickPiRxBytes(){
  int result;
  if (ioctl (UART_file_descriptor, FIONREAD, &result) == -1)
    return -1;
  return result;
}

// Trash any data in the Rx buffer
int BrickPiRxFlush(){
  int result = BrickPiRxBytes();
  if(result > 255)
    result = 255;
  if(result == -1)
    return -1;
  
  while(result){
    unsigned char rx_trash_buffer[256];
    read(UART_file_descriptor, rx_trash_buffer, result);
    result = BrickPiRxBytes();
    if(result == -1)
      return -1;
    if(result > 255)
      result = 255;
  }
  return 0;
}

// Receive a UART message
int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout){  // timeout in uS, not mS
  unsigned char rx_buffer[256];
  unsigned char RxBytes = 0;
  unsigned char CheckSum = 0;
  unsigned char i = 0;
  int result;
  unsigned long OrigionalTick = CurrentTickUs();

  result = BrickPiRxBytes();
  while(result == 0){
    if(timeout && ((CurrentTickUs() - OrigionalTick) >= timeout))return -2;
    usleep(100);
    result = BrickPiRxBytes();    
  }
  
  if(result == -1)return -1;
  
  RxBytes = 0;
  while(RxBytes < result){                       // If it's been <<<2 times a single byte time>>> since the last data was received, assume it's the end of the message.
    RxBytes = result;
    usleep((((1000000 * 10) / BaudRate) * 2));
    result = BrickPiRxBytes();
    if(result == -1)return -1;
  }

  if (read(UART_file_descriptor, rx_buffer, RxBytes) != RxBytes)
    return -1;

  if(RxBytes < 2)
    return -4;
  
  if(RxBytes < (rx_buffer[1] + 2))
    return -6;
  
  CheckSum = rx_buffer[1];
  
  i = 0;
  while(i < (RxBytes - 2)){
    CheckSum += rx_buffer[i + 2];
    InArray[i] = rx_buffer[i + 2];
    i++;
  }
  
  if(CheckSum != rx_buffer[0])
    return -5;
  
  *InBytes = (RxBytes - 2);

  return 0;  
}

#endif