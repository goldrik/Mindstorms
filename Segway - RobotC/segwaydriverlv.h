/* LEGO MINDSTORMS NXT Segway program driver */

int steering = 0;
int acceleration = 50;
int speed = 0;
bool starting_balancing_task = true;

//GLOBAL VARIABLE SETUP
float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading,gear_down_ratio,dt;

#ifdef HiTechnic_Gyro
//Prototype (see full code below)
int calibrate_hitechnic();
#endif

task balancing()
{
	gear_down_ratio = 1;

	// If code is added to the control loop below, make sure it runs under dt seconds. Default is 0.010 seconds.
  dt = 0.01;

  // PID constants
  gn_dth_dt = 0.23;
  gn_th = 25.00;
  gn_y = 272.8;
  gn_dy_dt = 24.6;
  kp = 0.0336;
  ki = 0.2688;
  kd = 0.000504;

  //MOTOR SETUP
  nMotorPIDSpeedCtrl[motorB] = mtrNoReg;
  nMotorPIDSpeedCtrl[motorC] = mtrNoReg;
  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorB] = 0;

  //SENSOR SETUP
  int nSensorsDefined = 0;
	#ifdef HiTechnic_Gyro
	    SensorType[Gyro] = sensorRawValue;
	    // Avoid calibration by replacing "calibrate_hitechnic()" below with the mean_reading value
	    mean_reading = 593;
	    nSensorsDefined++;
	#endif

	if(nSensorsDefined != 1){
	  nxtDisplayTextLine(0,"Check Sensor");
	  nxtDisplayTextLine(1,"definition");
	  wait1Msec(5000);StopAllTasks();
	}

  //MATH CONSTANTS
  const float radius = your_wheel_diameter/1000;
  const float degtorad = PI/180;

  //SETUP VARIABLES FOR CALCULATIONS
  float u = 0;                      // Sensor Measurement (raw)
  float th = 0,//Theta              // Angle of robot (degree)
        dth_dt = 0;//dTheta/dt      // Angular velocity of robot (degree/sec)
  float e = 0,//Error               // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0,//dError/dt       // Change of above error
        _edt = 0,//Integral Error   // Accumulated error in time
        e_prev = 0;//Previous Error // Error found in previous loop cycle
  float pid = 0;                    // SUM OF PID CALCULATION
  float y = 0,//y                      // Measured Motor position (degrees)
        dy_dt = 0,//dy/dt              // Measured motor velocity (degrees/sec)
	      v = 0,//velocity            // Desired motor velocity (degrees/sec)
	      y_ref = 0;//reference pos   // Desired motor position (degrees)
  int motorpower = 0,               // Power ultimately applied to motors
      last_steering = 0,            // Steering value in previous cycle
      straight = 0,                 // Average motor position for synchronizing
      d_pwr = 0;                    // Change in power required for synchronizing
  const int n_max = 7;              // Number of measurement used for floating motor speed average
  int n = 0,n_comp = 0,             // Intermediate variables needed to compute measured motor speed
  encoder[n_max];                   // Array containing last n_max motor positions
  memset(&encoder[0],0,sizeof(encoder));
  starting_balancing_task = false;  // Finish configuration. Main task  resumes.

  ClearTimer(T4);                   // This timer is used in the driver. Do not use it for other purposes

  while(true)
  {
    //READ GYRO SENSOR
		#ifdef HiTechnic_Gyro
      u =   SensorRaw[Gyro];wait1Msec(2);
      u = u+SensorRaw[Gyro];
		#endif

		//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
  	dth_dt = u/2 - mean_reading;
  	mean_reading = mean_reading*0.999 + (0.001*(dth_dt+mean_reading));
  	th = th + dth_dt*dt;

    //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
    if(v < speed*10){
    v = v + acceleration*10*dt;}
    else if(v > speed*10){
    v = v - acceleration*10*dt;}
    y_ref = y_ref + v*dt;

  	//COMPUTE MOTOR ENCODER POSITION AND SPEED
  	n++;if(n == n_max){n = 0;}
  	encoder[n] = nMotorEncoder[motorB] + nMotorEncoder[motorC] + y_ref;
  	n_comp = n+1;if(n_comp == n_max){n_comp = 0;}
  	y = encoder[n]*degtorad*radius/gear_down_ratio;
  	dy_dt = (encoder[n] - encoder[n_comp])/(dt*(n_max-1))*degtorad*radius/gear_down_ratio;

  	//COMPUTE COMBINED ERROR AND PID VALUES
  	e = gn_th * th + gn_dth_dt * dth_dt + gn_y * y + gn_dy_dt * dy_dt;
  	de_dt = (e - e_prev)/dt;
  	_edt = _edt + e*dt;
  	e_prev = e;
  	pid = (kp*e + ki*_edt + kd*de_dt)/radius*gear_down_ratio;

  	//ADJUST MOTOR SPEED TO STEERING AND SYNCHING
    if(steering == 0){
        if(last_steering != 0){
	        straight = nMotorEncoder[motorC] - nMotorEncoder[motorB];}
		    d_pwr = (nMotorEncoder[motorC] - nMotorEncoder[motorB] - straight)/(radius*10/gear_down_ratio);}
    else{d_pwr = steering/(radius*10/gear_down_ratio);}
    last_steering = steering;

  	//CONTROL MOTOR POWER AND STEERING
  	motorpower = 	pid;
    motor[motorB] = motorpower + d_pwr;
    motor[motorC] = motorpower - d_pwr;

    //ERROR CHECKING OR SHUTDOWN
    if(abs(th)>60 || abs(motorpower) > 2000){
      StopAllTasks();}

    //WAIT THEN REPEAT
  	while(time1[T4] < dt*1000){
  	  wait1Msec(1);}
  	ClearTimer(T4);
  }
}

#ifdef HiTechnic_Gyro
int calibrate_hitechnic()
{
 //Function for finding the HiTechnic Gyro offset
 int mean_reading = 0, p = 0;

 while(nNxtButtonPressed == kEnterButton){}
 nxtDisplayTextLine(0,"Put Segway Down");
 wait1Msec(500);
 nxtDisplayTextLine(2,"Calibrating");
 nxtDisplayTextLine(3,"HiTechnic Gyro..");

 for(p = 0; p < 40;p++){
    mean_reading = mean_reading + SensorRaw[Gyro];
    wait1Msec(50);}
 mean_reading = mean_reading/40;
 PlayTone(500,50);

 if(mean_reading < 550 || mean_reading > 640){
    nxtDisplayTextLine(4,"FAILED");
    nxtDisplayTextLine(6,"Sensor Connected?");
    wait1Msec(2000);StopAllTasks();}

 string varA = "Gyro Value:";
 int varB = mean_reading;

 eraseDisplay();
 nxtDisplayTextLine(2,"%s %d", varA, varB);
 nxtDisplayTextLine(4,"Hold NXT upright");
 nxtDisplayTextLine(5,"and press start");
 while(nNxtButtonPressed != kEnterButton){}
 while(nNxtButtonPressed == kEnterButton){}
 eraseDisplay();

 return(mean_reading);
}
#endif
