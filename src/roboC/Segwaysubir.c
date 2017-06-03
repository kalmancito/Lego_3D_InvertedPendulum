
#define HiTechnic_Gyro


/*SELECT SENSOR PORT (S1, S2, S3 or S4), and WHEEL DIAMETER (milimeters).*/
const tSensors Gyro = S2;
//const float your_wheel_diameter = 42;

#include "segway-driver-lvV2.h"



task main()
{
  //Start balancing and wait for configuration to finish
  StartTask(control);
  while(starting_balancing_task){}

	 ClearTimer(T2);
  	while(time1[T2]<2000){
  speed =20;
  pivotar = 0;
  }
  ClearTimer(T2);
  	while(time1[T2]<5000){
  speed =0;
  pivotar = -10;
  }

  ClearTimer(T2);
  	while(time1[T2]<4000){
  speed =10;
  pivotar = 0;
  }

}
