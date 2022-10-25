void enableLimitSwitch()
{
    if(X_waitingOnHome == false)
    {
        limitSwitch(motorX);
    }

    if(Y_waitingOnHome == false)
    {
        limitSwitch(motorY);
    }

    if(Z_waitingOnHome == false)
    {
        limitSwitch(motorZ);
    }
}

int degreesTo43ReduceWithMicroSteps(float d){
  int result = int(d * 382.2222222);
  return result;
}

//WIP
int degreesTo30ReduceWithMicroSteps(float d){
  int result = int(d * 382.2222222);
  return result;
}

//WIP
int degreesTo43ReduceWithMicroSteps(float d){
  int result = int(d * 382.2222222);
  return result;
}

//WIP
int degreesTo30ReduceSteps(float d){
  int result = int(d * 382.2222222);
  return result;
}


// Detect limit switch being pressed and take action to rectify.
void limitSwitch(int motorNum)
{
  limitSwitchArray[motorNum].loop();
  
  int state = limitSwitchArray[motorNum].getState();
  
  if(state == LOW)
  {
    emergencyStop(motorNum);
    moveOffLimitSwitch(motorNum);
  }
}

// Perform desired action on desired motor.
void newAction(long target, int motorNum)
{
  stepperArray[motorNum].moveTo(target);

  // Keep doing desired action until limit switch is hit
  while(stepperArray[motorNum].distanceToGo() != 0)
  {

    limitSwitchArray[motorNum].loop();
    stepperArray[motorNum].run();
    
    // Limit Switch hit, stop immediately
    limitSwitch(motorNum);
  }

  Serial.println("");
  Serial.print(motorNames[motorNum]);
  Serial.print(": ");
  Serial.print(stepperArray[motorNum].currentPosition()); 
  Serial.println("");  
}

// Function to simplify "home" behavior by calling each motor's respective
// home function.
void allHome()
{
  X_waitingOnHome = true;
  motorGoHome(motorX);
  X_waitingOnHome = false;

  Y_waitingOnHome = true;
  motorGoHome(motorY);
  Y_waitingOnHome = false;

  //newAction(-500, motorY);
  Z_waitingOnHome = true;
  motorGoHome(motorZ);
  Z_waitingOnHome = false;
}

void motorGoHome(int motorNum)
{ 
 limitSwitchArray[motorNum].loop();
  while(limitSwitchArray[motorNum].isPressed() == false)
  {
    limitSwitchArray[motorNum].loop();
    
    int state = limitSwitchArray[motorNum].getState();
    if(state == HIGH){
      motorArray[motorNum].setDir(-1);
      motorArray[motorNum].stepOn();
      delayMicroseconds(_delay);
      motorArray[motorNum].stepOff();
    }
  }

  stepperArray[motorNum].setCurrentPosition(0);

  Serial.println("");
  Serial.print(motorNames[motorNum]);
  Serial.print(": ");
  Serial.print(stepperArray[motorNum].currentPosition()); 
  Serial.println(" Home - Done!"); 
  
  moveOffLimitSwitch(motorNum);
}


void emergencyStop(int motorNum)
{
  Serial.println("");
  Serial.print(motorNames[motorNum]);
  Serial.println(": E-Stopping!");
  stepperArray[motorNum].setAcceleration(8000);
  stepperArray[motorNum].stop();

  stepperArray[motorNum].setAcceleration(1000);
}

void  moveOffLimitSwitch(int motorNum)
{
  while(limitSwitchArray[motorNum].isReleased() == false)
  {
    limitSwitchArray[motorNum].loop();

    motorArray[motorNum].setDir(1);
    motorArray[motorNum].stepOn();
    delayMicroseconds(_delay);
    motorArray[motorNum].stepOff();
  }
  stepperArray[motorNum].setCurrentPosition(0);

  Serial.print(motorNames[motorNum]);
  Serial.print(" (Moved away from Limit Switch): ");
  Serial.println(stepperArray[motorNum].currentPosition());
  Serial.print(motorNames[motorNum]); 
  Serial.println(": Moving Away - Done!");
}


// ------------------------- //
// Custom Movement Functions //
// ------------------------- //

// Place a Paver (Simple)
void place()
{
  //newAction(-29000, motorX);
  //newAction(-10000, motorY);
  //newAction(-47000, motorZ);

  //newAction(-47000, motorZ);
  //newAction(-40000, motorX);
  //newAction(-24000, motorY);

  newAction(-40000, motorZ);
  newAction(-20000, motorX);
  newAction(-12000, motorY);
  newAction(-47000, motorZ);
  newAction(-24000, motorY);
  newAction(-40000, motorX);
}

void pickup()
{
  //newAction(-29000, motorX);
  //newAction(-10000, motorY);
  //newAction(-47000, motorZ);

  //newAction(-47000, motorZ);
  //newAction(-40000, motorX);
  //newAction(-24000, motorY);

  newAction(-40000, motorZ);
  newAction(-20000, motorX);
  newAction(-12000, motorY);
  newAction(-47000, motorZ);
  newAction(-24000, motorY);
  newAction(-40000, motorX);
}

void autoMove(){
  newAction(3000, motorQ);
  newAction(0, motorQ);
}
