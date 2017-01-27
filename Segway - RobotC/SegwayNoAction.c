#define HiTechnic_Gyro

const tSensors Gyro = S2;
const float your_wheel_diameter = 54;

//This is the Segway Driver code. Place in same directory as this program
#include "segwaydriverlv.h"

task main()
{
  StartTask(balancing);
  while(starting_balancing_task){}

  // Default speed is set to 0, so the robot stays in one position

}
