#define HiTechnic_Gyro

const tSensors Gyro = S2;
const float your_wheel_diameter = 54;

//This is the Segway Driver code. Place in same directory as this program
#include "segwaydriverlv.h"

task main()
{
  StartTask(balancing);
  while(starting_balancing_task){}

  while(true)
  {
	  speed = 70;
	  while(nMotorEncoder[motorB] >= -1080)
	  	{wait1Msec(100);}

	  // Going forwards means that the motors run backwards, because of their orientation.
	  // So to go forward four times (1080 degrees), the encoder must reach -1080 degrees.

	  speed = -70;
	  while(nMotorEncoder[motorB] <= 0)
	  	{wait1Msec(100);}
	 }
}
