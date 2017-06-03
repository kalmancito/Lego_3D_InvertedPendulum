

/*
Files:
The task in this driver should run parallel to the main program task.
It will handle all balancing control and leaves the main task free for
custom user programs, allowing the Segway to move around and, for example
avoiding walls. The following examples make use of this driver:

 - Example #1: Segway-Explanation(Wall-Avoidance).c (Includes description of commands to control Segway movement)
 - Example #2: Segway-NoAction.c
 - Example #3: Segway-Encoders.c
*/

/*
LEEME!!


codigo nuevo, funcionando poniendo depie el robot, en RoboC

la estrategia de control es exactamente la que difinio Fran. Lleva solo un
gyro, con un filtro paso altos, para eliminar el bias, ademas de
calibrarlo al principio.

Lee los dos motores para ver la diferencia de avance y compensarla,
y tambien lo usa para girar.
*/
#define MD motorB
#define MI motorC

int pivotar = 0;
int acceleration = 50;
int speed = 0;
bool starting_balancing_task = true;

//globales
float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,bias,dt;

int calibrar();


task control()
{
  ///////////////////////////
  //ADVANCED USER CONTROL
  ///////////////////////////

  /*
  The following values can be modified for advanced control. Many users will not use these features,
  which is why I do not require these to be set in the main program. You can just modify them here.
  if you are unsure about what they do, just leave them at the default value.
  */

   // Set the time each loop cycle should last. You can set it up to 0.03 seconds or even higher, if you really
  // need to. If you add code to the control loop below (such as to read another sensor), make sure that
  // all code can run in under dt seconds. If it takes more time, set dt to a higher value.
  // Default is 0.010 seconds (10 miliseconds).
  dt = 0.010;

  // Customize PID constants. These variables are global, so you can optionally dynamically change them in your main task.
  gn_dth_dt = 0.23;
  gn_th = 25.00;
  gn_y = 272.8;
  gn_dy_dt = 24.6;
  kp = 0.0336;
  ki = 0.2688;
  kd = 0.000504;

  ///////////////////////////
  //END ADVANCED USER CONTROL
  ///////////////////////////

  //MOTOR SETUP
  nMotorPIDSpeedCtrl[MD] = mtrNoReg;
  nMotorPIDSpeedCtrl[MI] = mtrNoReg;
  nMotorEncoder[MI] = 0;
  nMotorEncoder[MD] = 0;

  //SENSOR SETUP
  SensorType[Gyro] = sensorRawValue;
	    // The following sets the average HiTechnic sensor value. If you know the average, you can avoid the calibration
	    // next time like so: bias = 593.82; (if that's your sensor average).
	    bias = calibrar();



  //MATH CONSTANTS
  const float radius = 54/2/1000;
  const float degtorad = PI/180;

  //SETUP VARIABLES FOR CALCULATIONS
  float u = 0;                    // Sensor Measurement (raw)
  float th = 0,//Theta            // Angle of robot (degree)
        dth_dt = 0;//dTheta/dt    // Angular velocity of robot (degree/sec)
  float e = 0,//Error             // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0,//dError/dt     // Change of above error
        _edt = 0,//Integral Error // Accumulated error in time
        e_prev = 0;//Previous Error/ Error found in previous loop cycle
  float pid = 0;                  // SUM OF PID CALCULATION
  float y = 0,//y                     // Measured Motor position (degrees)
        dy_dt = 0,//dy/dt             // Measured motor velocity (degrees/sec)
	      v = 0,//velocity          // Desired motor velocity (degrees/sec)
	      y_ref = 0;//reference pos // Desired motor position (degrees)
  int motorpower = 0,             // Power ultimately applied to motors
      last_pivotar = 0,          // pivotar value in previous cycle
      straight = 0,               // Average motor position for synchronizing
      d_pwr = 0;                  // Change in power required for synchronizing
  int   encoder,encoderAnt;            // Intermediate variables needed to compute measured motor speed

  starting_balancing_task = false;// We're done configuring. Main task now resumes.


  ClearTimer(T4);                 // This timer is used in the driver. Do not use it for other purposes!

  while(true)
  {

    //READ GYRO SENSOR
		  u =   SensorRaw[Gyro];wait1Msec(2);
      u = u+SensorRaw[Gyro];
		////////////


		//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
  	dth_dt = u/2 - bias;
  	bias = bias*0.999 + (0.001*(dth_dt+bias));
  	th = th + dth_dt*dt;

    //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
    if(v < speed*10){
    v = v + acceleration*10*dt;}
    else if(v > speed*10){
    v = v - acceleration*10*dt;}
    y_ref = y_ref + v*dt;

  	//COMPUTE MOTOR ENCODER POSITION AND SPEED
  	encoder = nMotorEncoder[MD] + nMotorEncoder[MI] + y_ref;
  	y = encoder*degtorad*radius;
  	dy_dt = (encoder  - encoderAnt)/(dt)*degtorad*radius;
  	encoderAnt=encoder;

  	//COMPUTE COMBINED ERROR AND PID VALUES
  	e = gn_th * th + gn_dth_dt * dth_dt + gn_y * y + gn_dy_dt * dy_dt;
  	de_dt = (e - e_prev)/dt;
  	_edt = _edt + e*dt;
  	e_prev = e;
  	pid = (kp*e + ki*_edt + kd*de_dt)/radius*1;

  	//ADJUST MOTOR SPEED TO pivotar AND SYNCHING
    if(pivotar == 0){
        if(last_pivotar != 0){
	        straight = nMotorEncoder[MI] - nMotorEncoder[MD];}
		    d_pwr = (nMotorEncoder[MI] - nMotorEncoder[MD] - straight)/(radius*10/1);}
    else{d_pwr = pivotar/(radius*10/1);}
    last_pivotar = pivotar;

  	//CONTROL MOTOR POWER AND pivotar
  	motorpower = 	pid;
    motor[MD] = motorpower + d_pwr;
    motor[MI] = motorpower - d_pwr;

    //ERROR CHECKING OR SHUTDOWN
    if(abs(th)>60 || abs(motorpower) > 2000){
      StopAllTasks();}

   //WAIT THEN REPEAT
  	while(time1[T4] < dt*1000){
  	  wait1Msec(1);}
  	ClearTimer(T4);
  }
}


int calibrar()
{
 //Function for finding the HiTechnic Gyro offset
 int bias = 0, p = 0;

 while(nNxtButtonPressed == kEnterButton){}
 nxtDisplayTextLine(0,"Put Segway Down");
 wait1Msec(500);
 nxtDisplayTextLine(2,"Calibrating");
 nxtDisplayTextLine(3,"HiTechnic Gyro..");

 for(p = 0; p < 40;p++){
    bias = bias + SensorRaw[Gyro];
    wait1Msec(50);}
 bias = bias/40;
 PlayTone(500,50);

 if(bias < 550 || bias > 640){
    nxtDisplayTextLine(4,"FAILED. Exiting...");
    nxtDisplayTextLine(6,"Sensor Connected?");
    wait1Msec(2000);StopAllTasks();}

 eraseDisplay();
 nxtDisplayTextLine(2,"Done! Hold Segway");
 nxtDisplayTextLine(4,"upright and press");
 nxtDisplayTextLine(6,"the orange button");
 while(nNxtButtonPressed != kEnterButton){}
 while(nNxtButtonPressed == kEnterButton){}
 eraseDisplay();

 return(bias);
}
