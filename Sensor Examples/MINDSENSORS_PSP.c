/*
########################################################################
#                                                                      #
# Program Name: MINDSENSORS_PSP.c                                       	   #
# ===========================                                          #
#                                                                      #
# Copyright (c) 2013 by dexterindustries.com                           #
#                                                                      #
# This program is free software. You can redistribute it and/or modify #
# it under the terms of the GNU General Public License as published by #
# the Free Software Foundation; version 3 of the License.              #
# Read the license at: http://www.gnu.org/licenses/gpl.txt             #
#                                                                      #
########################################################################
# History
# ------------------------------------------------
# Author     Date      Comments
# Karan      04/11/13  Initial Authoring
#
# Ported from the C Library Provided by mindsensors.com: 
# Email: info (<at>) mindsensors (<dot>) com 
# History
# ------------------------------------------------
# Author     Date      Comments
# Deepak     04/08/09  Initial Authoring.
#
#--------------------------------------
  Controller button layout:
----------------------------------------

      L1                R1
      L2                R2

      d                 triang
   a     c         square     circle
      b                  cross

     l_j_b              r_j_b
     l_j_x              r_j_x
     l_j_y              r_j_y

-------------------------------------- #
#
  bits as follows:
	b1:   a b c d x r_j_b l_j_b x
	b2:   square cross circle triang R1 L1 R2 L2
*/

#include <stdio.h>
#include "tick.h"
#include <wiringPi.h>
#include "BrickPi.h"


// gcc -o program MINDSENSORS_PSP.c -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

#define I2C_PORT  PORT_1                             // I2C port for the dCompass
#define I2C_SPEED 6                                  // delay for as little time as possible. Usually about 100k baud

struct button
{
	char   l1;
	char   l2;
	char   r1;
	char   r2;
	char   a;
	char   b;
	char   c;
	char   d;
	char   tri;
	char   sqr;
	char   cir;
	char   cro;
	char   ljb;  // joystick button state
	char   rjb;  // joystick button state

	int   ljx;   // analog value of joystick scaled from 0 to 100
	int   ljy;   // analog value of joystick scaled from 0 to 100
	int   rjx;   // analog value of joystick scaled from 0 to 100
	int   rjy;   // analog value of joystick scaled from 0 to 100
}b;

//Initialize all the buttons to 0
void init (void)
{
	b.l1 = 0;
	b.l2 = 0;
	b.r1 = 0;
	b.r2 = 0;
	b.a = 0;
	b.b = 0;
	b.c = 0;
	b.d = 0;
	b.tri = 0;
	b.sqr = 0;
	b.cir = 0;
	b.cro = 0;
	b.ljb = 0;
	b.rjb = 0;
	b.ljx = 0;
	b.ljy = 0;
	b.rjx = 0;
	b.rjy = 0;
}

//Update all the buttons
//For all buttons:
//	0:	Unpressed
//	1:	Pressed
//	
//	Left and right joystick: -127 to 127 
void upd(void)
{
	//Left and right joystick button press
	b.ljb = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 1) & 0x01;
	b.rjb = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 2) & 0x01;
	
	//For buttons a,b,c,d
	b.d = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 4) & 0x01;
	b.c = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 5) & 0x01;
	b.b = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 6) & 0x01;
	b.a = ~(BrickPi.SensorI2CIn[I2C_PORT][0][0] >> 7) & 0x01;
 
	//For buttons l1,l2,r1,r2
	b.l2    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] ) & 0x01;
	b.r2    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 1) & 0x01;
	b.l1    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 2) & 0x01;
	b.r1    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 3) & 0x01;
	
	//For buttons square,triangle,cross,circle
	b.tri    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 4) & 0x01;
	b.cir    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 5) & 0x01;
	b.cro = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 6) & 0x01;
	b.sqr    = ~(BrickPi.SensorI2CIn[I2C_PORT][0][1] >> 7) & 0x01;
   
   //Left joystick x and y , -127 to 127
	b.ljx = (BrickPi.SensorI2CIn[I2C_PORT][0][2]&0xff) - 128;
	b.ljy = (BrickPi.SensorI2CIn[I2C_PORT][0][4]&0xff) - 128;
	
	//Right joystick x and y , -127 to 127
	b.rjx = -1*((BrickPi.SensorI2CIn[I2C_PORT][0][3]&0xff) - 128);
	b.rjy = -1*((BrickPi.SensorI2CIn[I2C_PORT][0][5]&0xff) - 128);
}

//Show button values
void show_val(void)
{
	printf("ljb rjb d c b a l2 r2 l1 r1 tri cir cro sqr\tljx\tljy\trjx\trjy\n");
	printf("%d ",b.ljb);
	printf("  %d ",b.rjb);
	printf("  %d ",b.d);
	printf("%d ",b.c);
	printf("%d ",b.b);
	printf("%d ",b.a);
	         
	printf("%d ",b.l2);
	printf(" %d ",b.r2);
	printf(" %d ",b.l1);
	printf(" %d ",b.r1);
	printf(" %d ",b.tri);
	printf("  %d ",b.cir);
	printf("  %d ",b.cro);
	printf("  %d ",b.sqr);
	printf("\t%d ",b.ljx);
	printf("\t%d ",b.rjx);
	printf("\t%d ",b.ljy);
	printf("\t%d\n\n",b.rjy);
}
int main() {
	int result;
	ClearTick();
	printf("This program is free software. You can redistribute it and/or modify it under the terms of \nthe GNU General Public License as published by the Free Software Foundation; version 3 of the License. \nRead the license at: http://www.gnu.org/licenses/gpl.txt\n\n");
	BrickPi.Address[0] = 1;
	BrickPi.Address[1] = 2;

	result = BrickPiSetup();
	printf("BrickPiSetup: %d\n", result);
	BrickPi.SensorType       [I2C_PORT]    = TYPE_SENSOR_I2C;
	BrickPi.SensorI2CSpeed   [I2C_PORT]    = I2C_SPEED;
	BrickPi.SensorI2CDevices [I2C_PORT]    = 1;

	BrickPi.SensorSettings   [I2C_PORT][0] = 0;  
	BrickPi.SensorI2CAddr    [I2C_PORT][0] = 0x02;	//address for writing

	BrickPiSetupSensors();

	while(1)
	{
		//Send 0x42 to get a response back
		BrickPi.SensorI2CWrite [I2C_PORT][0]    = 1;	//number of bytes to write
		BrickPi.SensorI2CRead  [I2C_PORT][0]    = 6;	//number of bytes to read
		BrickPi.SensorI2COut   [I2C_PORT][0][0] = 0x42;	//byte to write
		BrickPiUpdateValues();
		upd();			//Update the button values
		show_val();		//#Show the values 
						//To use the button values in you own program just call it, 
						//eg x=b.ljx will fetch and store the value of the Left Joystick position in the X-axis in the variable x
		init();			//Initialize all buttons to 0
		usleep(100000);	//Give a delay of 100ms
	}
	return 0;
}
