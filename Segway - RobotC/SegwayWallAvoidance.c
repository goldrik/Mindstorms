#define HiTechnic_Gyro

const tSensors Gyro = S2;
const float your_wheel_diameter = 54;

//This is the Segway Driver code. Place in same directory as this program
#include "segwaydriverlv.h"

task main()
{
  StartTask(balancing);
  while(starting_balancing_task){}

  const tSensors Sonar = S1;
  SensorType[Sonar] = sensorSONAR;

  acceleration = 60;
  steering = 0;
  wait1Msec(3000);
  PlaySound(soundBeepBeep);
  speed = 60;

  while(true)
  	{
  		if(SensorValue[Sonar] < 40)
  			{
  				speed = -50;
  				wait1Msec(2000);
  				speed = 0;
  				wait1Msec(500);
  				steering = 15;
  				wait1Msec(1000);
  				steering = 0;
  				speed = 50;
  			}
  			wait1Msec(100);
  	}
}
