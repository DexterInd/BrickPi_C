/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: June 18, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*/

#ifndef __BrickPi_h_
#define __BrickPi_h_

#define DEBUG

#include <wiringPi.h>

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

#define PORT_1 0
#define PORT_2 1
#define PORT_3 2
#define PORT_4 3

#define MASK_D0_M 0x01
#define MASK_D1_M 0x02
#define MASK_9V   0x04
#define MASK_D0_S 0x08
#define MASK_D1_S 0x10

#define BYTE_MSG_TYPE 0          // MSG_TYPE is the first byte.
  #define MSG_TYPE_CHANGE_ADDR 1 // Change the UART address.
  #define MSG_TYPE_SENSOR_TYPE 2 // Change/set the sensor type.
  #define MSG_TYPE_VALUES      3 // Set the motor speed and direction, and return the sesnors and encoders.
  #define MSG_TYPE_E_STOP      4 // Float motors immidately

  // New UART address (MSG_TYPE_CHANGE_ADDR)
    #define BYTE_NEW_ADDRESS     1
  
  // Sensor setup (MSG_TYPE_SENSOR_TYPE)
    #define BYTE_SENSOR_1_TYPE   1
    #define BYTE_SENSOR_2_TYPE   2

#define TYPE_MOTOR_PWM                 0
#define TYPE_MOTOR_SPEED               1
#define TYPE_MOTOR_POSITION            2

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

#define BIT_I2C_MID  0x01  // defined for each device
#define BIT_I2C_SAME 0x02  // defined for each device

#define INDEX_RED   0
#define INDEX_GREEN 1
#define INDEX_BLUE  2
#define INDEX_BLANK 3

void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]);

struct BrickPiStruct{
  unsigned char Address        [2];     // Communication addresses

/*
  Motors
*/
  int           MotorSpeed     [4];     // Motor speeds, from -255 to 255
  unsigned char MotorEnable    [4];     // Motor enable/disable

/*
  Encoders
*/
  long          EncoderOffset  [4];     // Encoder offsets (not yet implemented)
  long          Encoder        [4];     // Encoder values

/*
  Sensors
*/
  long          Sensor         [4];     // Primary sensor values
  long          SensorArray    [4][4];  // For more sensor values for the sensor (e.g. for color sensor FULL mode).
  unsigned char SensorType     [4];     // Sensor types
  unsigned char SensorSettings [4][8];  // Sensor settings, used for specifying I2C settings.

/*
  I2C
*/
  unsigned char SensorI2CDevices[4];        // How many I2C devices are on each bus (1 - 8).
  unsigned char SensorI2CSpeed  [4];        // The I2C speed.
  unsigned char SensorI2CAddr   [4][8];     // The I2C address of each device on each bus.  
  unsigned char SensorI2CWrite  [4][8];     // How many bytes to write
  unsigned char SensorI2CRead   [4][8];     // How many bytes to read
  unsigned char SensorI2COut    [4][8][16]; // The I2C bytes to write
  unsigned char SensorI2CIn     [4][8][16]; // The I2C input buffers
};

struct BrickPiStruct BrickPi;

unsigned char Array[256];
unsigned char BytesReceived;

int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr){
  unsigned char i = 0;
  Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR;
  Array[BYTE_NEW_ADDRESS] = NewAddr;
  BrickPiTx(OldAddr, 2, Array);
    
  if(BrickPiRx(&BytesReceived, Array, 5000))
    return -1;
  if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR))
    return -1;

  return 0;
}

unsigned int Bit_Offset = 0;

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

int BrickPiSetupSensors(){
  unsigned char i = 0;
  while(i < 2){
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
    if(BrickPiRx(&BytesReceived, Array, 500000))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
      return -1;
    i++;
  }
  return 0;
}

unsigned char Retried = 0; // For re-trying a failed update.

int BrickPiUpdateValues(){
  unsigned char i = 0;
  unsigned int ii = 0;
  while(i < 2){
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
        unsigned char Temp_ENC_DIR;
        unsigned char Temp_BitsNeeded;
        
        AddBits(1, 0, 1, 1);
        if(Temp_Value < 0){
          Temp_ENC_DIR = 1;
          Temp_Value *= (-1);
        }
        Temp_BitsNeeded = (BitsNeeded(Temp_Value) + 1);
        AddBits(1, 0, 5, Temp_BitsNeeded);
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
      speed = BrickPi.MotorSpeed[port];
      dir = 0;
      if(speed < 0){
        dir = 1;
        speed *= (-1);
      }
      if(speed > 255){
        speed = 255;
      }
      AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (BrickPi.MotorEnable[port] & 0x01)) & 0x3FF));
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
    
    int result = BrickPiRx(&BytesReceived, Array, 7500);
    
    if(result != -2){                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
      BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0;
      BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0;
    }
    
    if(result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES)){
#ifdef DEBUG
      printf("BrickPiRx error: %d\n", result);
#endif
      if(Retried < 2){
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
      Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii]);
      if(Temp_EncoderVal & 0x01){
        Temp_EncoderVal /= 2;
        BrickPi.Encoder[ii + (i * 2)] = Temp_EncoderVal * (-1);}
      else{
        BrickPi.Encoder[ii + (i * 2)] = (Temp_EncoderVal / 2);}      
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

int UART_file_descriptor = 0; 

int BrickPiSetup(){
  UART_file_descriptor = serialOpen("/dev/ttyAMA0", 500000);
  if(UART_file_descriptor == -1){
    return -1;
  }
  return 0;
}

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
  i = 0;
  while(i < (ByteCount + 3)){
    serialPutchar(UART_file_descriptor, tx_buffer[i]);
    i++;
  }
}

int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout){  // timeout in uS, not mS
  unsigned char rx_buffer[256];
  unsigned char RxBytes = 0;
  unsigned char CheckSum = 0;
  unsigned char i = 0;
  int result;
  unsigned long OrigionalTick = CurrentTickUs();
  while(serialDataAvail(UART_file_descriptor) <= 0){
    if(timeout && ((CurrentTickUs() - OrigionalTick) >= timeout))return -2;
  }

  RxBytes = 0;
  while(RxBytes < serialDataAvail(UART_file_descriptor)){                                   // If it's been 1 ms since the last data was received, assume it's the end of the message.
    RxBytes = serialDataAvail(UART_file_descriptor);
    usleep(75);
  }
  
  i = 0;  
  while(i < RxBytes){
    result = serialGetchar(UART_file_descriptor);
    if(result >= 0){
      rx_buffer[i] = result;
    }
    else{      
      return -1;    
    }
    i++;    
  }

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