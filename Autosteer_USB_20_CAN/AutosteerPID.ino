void calcSteeringPID(void) 
 {  
  //Proportional only
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int)pValue;
  
  errorAbs = abs(steerAngleError);
  int newMax = 0; 
   
  if (errorAbs < LOW_HIGH_DEGREES)
  {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  }
  else newMax = steerSettings.highPWM;
    
  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0 ) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0 ) pwmDrive += steerSettings.minPWM;
  
  //Serial.print(newMax); //The actual steering angle in degrees
  //Serial.print(",");

  //limit the pwm drive
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;

  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

//+++++++++++++++++++++++++++ CAN +++++++++++++++++++++++++++++
  int16_t shift = (sq(pwmDrive)/32);
  if (pwmDrive< 0) shift *= -1;
  canSteerPosition = canSteerPosition + shift;
    if (canSteerPosition > 32767) canSteerPosition = 32767;
    if (canSteerPosition < -32768) canSteerPosition = -32768;

//-------------------------- CAN ---------------------------------

  if (steerConfig.isDanfoss)
  {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2; // Devide by 4
    pwmDrive += 128;          // add Center Pos.
  }
 }

//#########################################################################################
