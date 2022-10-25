// Custom defined motions for each stepper motor, includes both clockwise and anticlockwise movements
void clockwiseX()
{
  int state = limitSwitchArray[0].getState();
  if(state == HIGH){
    ramps.motorX.setDir(-1);
    ramps.motorX.stepOn();
    delayMicroseconds(_delay);
    if(limitSwitchArray[0].isPressed()){
      ramps.motorX.stepOff();
    }
    ramps.motorX.stepOff();
  }
}

void anticlockwiseX()
{
  ramps.motorX.setDir(1);
  ramps.motorX.stepOn();
  delayMicroseconds(_delay);
  ramps.motorX.stepOff();
}

void clockwiseY()
{
  int state = limitSwitchArray[1].getState();
  if(state == HIGH){
    ramps.motorY.setDir(1);
    ramps.motorY.stepOn();
    delayMicroseconds(_delay);
    if(limitSwitchArray[1].isPressed()){
      ramps.motorY.stepOff();
    }
    ramps.motorY.stepOff();
  }
}

void anticlockwiseY()
{
  ramps.motorY.setDir(-1);
  ramps.motorY.stepOn();
  delayMicroseconds(_delay);
  ramps.motorY.stepOff();
}

void clockwiseZ()
{
  int state = limitSwitchArray[2].getState();
  if(state == HIGH){
    ramps.motorZ.setDir(-1);
    ramps.motorZ.stepOn();
    delayMicroseconds(_delay);
    if(limitSwitchArray[2].isPressed()){
      ramps.motorZ.stepOff();
    }
    ramps.motorZ.stepOff();
  }
}

void anticlockwiseZ()
{
  ramps.motorZ.setDir(1);
  ramps.motorZ.stepOn();
  delayMicroseconds(_delay);
  ramps.motorZ.stepOff();
}

void clockwiseE()
{
  ramps.motorE.setDir(1);
  ramps.motorE.stepOn();
  delayMicroseconds(_delay);
  ramps.motorE.stepOff();
}

void anticlockwiseE()
{
  ramps.motorE.setDir(-1);
  ramps.motorE.stepOn();
  delayMicroseconds(_delay);
  ramps.motorE.stepOff();

  readSerial();
}

void clockwiseQ()
{
  ramps.motorQ.setDir(1);
  ramps.motorQ.stepOn();
  delayMicroseconds(_delay);
  ramps.motorQ.stepOff();
}

void anticlockwiseQ()
{
  ramps.motorQ.setDir(-1);
  ramps.motorQ.stepOn();
  delayMicroseconds(_delay);
  ramps.motorQ.stepOff();
}

// ----------- //
// move Motors //
// ----------- //
void moveArm(String parameter[]) {
  // check if all positions are valid
  boolean valid = true;
  
  for (int i=0;i<6;i++){
    if (parameter[i] == ""){
      valid = false;
    }
  }

  // all positions are valid
  if(valid){
    int p1 = parameter[1].toInt();
    int p2 = parameter[2].toInt();
    int p3 = parameter[3].toInt();
    int p4 = parameter[4].toInt();
    int p5 = parameter[5].toInt();
    Serial.println("Echo: Move to " + parameter[1] + " " + parameter[2] + " " + parameter[3] + " " + parameter[4] + " " + parameter[5]);
    
    // If the number provided for each stepper motor (x,y,z,e,q) is positive, spin clockwise.
    // If it's negative, spin counter clockwise. 0 means no movement.
    if (p1 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        anticlockwiseX();
      }
    }
    else if (p1 > 0) {
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        clockwiseX();
      }
    }
    if (p2 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        anticlockwiseY();
      }
    }
    else if (p2 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        clockwiseY();
      }
    }
    if (p3 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        anticlockwiseZ();
      }
    }
    else if (p3 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        clockwiseZ();
      }
    }
    if (p4 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        anticlockwiseE();
      }
             
    }
    else if (p4 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        clockwiseE();
      }
    }
    if (p5 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        anticlockwiseQ();
      }
    }
    else if (p5 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 100){
        clockwiseQ();
      }
    }
  } 
  else {
    Serial.println("Echo: wrong amount of input arguments or nonvalid inputs");
  }
}
