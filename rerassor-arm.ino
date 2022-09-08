#include <Wire.h>
#include <AccelStepper.h>
#include <ezButton.h>
#include "Ramps.h"
#include "PololuDriver.h"

float newSpeed = 150.0;
float newAccel = 60.0;
double slwFkt = 0.77;
String serialData;
int cmdInt;
boolean stateChanged = true;
int _delay = 100;

// Create the motor shield object
Ramps ramps = Ramps();

// Create limit switches objects
ezButton limitswitchX(X_MAX_PIN);
ezButton limitswitchY(Y_MAX_PIN);
ezButton limitswitchZ(Z_MAX_PIN);

// Custom defined motions for each stepper motor, includes both clockwise and anticlockwise movements
void clockwiseX()
{
  int state = limitswitchX.getState();
  if(state == HIGH){
    ramps.motorX.setDir(1);
    ramps.motorX.stepOn();
    delayMicroseconds(_delay);
    if(limitswitchX.isPressed()){
      ramps.motorX.stepOff();
    }
    ramps.motorX.stepOff();
  }
}

void anticlockwiseX()
{
  ramps.motorX.setDir(-1);
  ramps.motorX.stepOn();
  delayMicroseconds(_delay);
  ramps.motorX.stepOff();
}

void clockwiseY()
{
  int state = limitswitchY.getState();
  if(state == HIGH){
    ramps.motorY.setDir(1);
    ramps.motorY.stepOn();
    delayMicroseconds(_delay);
    if(limitswitchY.isPressed()){
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
  int state = limitswitchZ.getState();\
  if(state == HIGH){
    ramps.motorZ.setDir(1);
    ramps.motorZ.stepOn();
    delayMicroseconds(_delay);
    if(limitswitchZ.isPressed()){
      ramps.motorZ.stepOff();
    }
    ramps.motorZ.stepOff();
  }
}

void anticlockwiseZ()
{
  ramps.motorZ.setDir(-1);
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

AccelStepper stepperX(clockwiseX, anticlockwiseX);
AccelStepper stepperY(clockwiseY, anticlockwiseY);
AccelStepper stepperZ(clockwiseZ, anticlockwiseZ);
AccelStepper stepperE(clockwiseE, anticlockwiseE);
AccelStepper stepperQ(clockwiseQ, anticlockwiseQ);

// ----- //
// Setup //
// ----- //
void setup() {
  //set up Serial library at 9600 bps
  Serial.begin(9600);

  limitswitchX.setDebounceTime(50);
  limitswitchY.setDebounceTime(50);
  limitswitchZ.setDebounceTime(50);
  
  // initialize the speed and acceleration of the motors
  stepperX.setMaxSpeed(newSpeed * slwFkt);
  stepperX.setAcceleration(newAccel * slwFkt);
  stepperY.setMaxSpeed(newSpeed * 43 / 30);
  stepperY.setAcceleration(newAccel * 43 / 30);
  stepperZ.setMaxSpeed(newSpeed * slwFkt);
  stepperZ.setAcceleration(newAccel * slwFkt);
  stepperE.setMaxSpeed(newSpeed * slwFkt);
  stepperE.setAcceleration(newAccel * slwFkt);
  stepperQ.setMaxSpeed(newSpeed * slwFkt);
  stepperQ.setAcceleration(newAccel * slwFkt);
}

// ---- //
// Loop //
// ---- //
void loop() {
  // Loop the limit switches
  limitswitchX.loop();
  limitswitchY.loop();
  limitswitchZ.loop();
  // run the steppers until target reached, then check and wait for new commands
  if ((stepperX.distanceToGo() == 0) && (stepperY.distanceToGo() == 0) && (stepperZ.distanceToGo() == 0) && (stepperE.distanceToGo() == 0) && (stepperQ.distanceToGo() == 0))
  {
    // send the current postion to the GUI once when target reached
    if (stateChanged) {      
      Serial.print("Echo: Positions ");
      Serial.print(stepperX.currentPosition());
      Serial.print(" ");
      Serial.print(stepperY.currentPosition());
      Serial.print(" ");
      Serial.print(stepperZ.currentPosition());
      Serial.print(" ");
      Serial.print(stepperE.currentPosition());
      Serial.print(" ");
      Serial.print(stepperQ.currentPosition());
      Serial.println(" 0");
      stateChanged = false;
    }

    // Check and wait for new commands
    readSerial();
        
  } else {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
    stepperE.run();
    stepperQ.run();
  }
}

// -------------------- //
// Handle Serial Inputs //
// -------------------- //
void readSerial() {
  String rxString = ""; 
  
  if (Serial.available()) {
    // Bytes are available in the Serial
        
    while (Serial.available()) {
      // Fill the buffer until all bytes are received and Serial is empty
      
      //Delay to allow byte to arrive in input buffer
      delay(2);
      
      //Read a single character from the buffer
      char ch = Serial.read();
      
      //Append that single character to a string
      rxString += ch;
    }

    // Process received data
    onDataRead(rxString);    
  }
}

void onDataRead(String rxString){  
  // process the read data and save all parameters inside cmd Array
  String cmdArray[10] = {""};
  int stringStart = 0;
  int arrayIndex = 0;

  // Loop through the entire read data
  for (int i = 0; i < rxString.length(); i++) {
    // check current character if it's the split character
    if (rxString.charAt(i) == ' ') {
      // clear previous values from array
      cmdArray[arrayIndex] = "";

      // save substring into array
      cmdArray[arrayIndex] = rxString.substring(stringStart, i);

      // set new string starting point
      stringStart = (i + 1);
      arrayIndex++;
    }
  }
  cmdArray[arrayIndex] = rxString.substring(stringStart,rxString.length());
  
  // parse to corresponding function
  if (cmdArray[0] == "S"){
    // change settings    
    changeSettings(cmdArray);
    
  } else if (cmdArray[0] == "M"){
    // move the arm    
    moveArm(cmdArray);    
    
  } else {
    // unknown command
    Serial.println("Echo: Unknown Command");
  }
}


// --------------- //
// Change Settings //
// --------------- //
void changeSettings(String parameter[]) {
  char p1 = parameter[1].charAt(0);
  int p2 = parameter[2].toInt();
  Serial.println("Echo: " + parameter[1] + "-" + p2);
  
  //Commmands to change settings
  switch (p1) {
    case 'S':
      // Change the max speed of the motors
      newSpeed = p2;
      stepperX.setMaxSpeed(newSpeed * slwFkt);
      stepperY.setMaxSpeed(newSpeed * 43 / 30);
      stepperZ.setMaxSpeed(newSpeed * slwFkt);
      stepperE.setMaxSpeed(newSpeed * slwFkt);
      stepperQ.setMaxSpeed(newSpeed * slwFkt);
      break;

    case 'A':
      // Change the max acceleration of the motor
      newAccel = p2;
      stepperX.setAcceleration(newAccel * slwFkt);
      stepperY.setAcceleration(newAccel * 43 / 30);
      stepperZ.setAcceleration(newAccel * slwFkt);
      stepperE.setAcceleration(newAccel * slwFkt);
      stepperQ.setAcceleration(newAccel * slwFkt);
      break;

    case 'T':
      // change the steptype of the motors
      break;

    case 'R':
      // release all motors
      break;

    case 'E':
      // reset all motor positions
      stepperX.setCurrentPosition(0);
      stepperY.setCurrentPosition(0);
      stepperZ.setCurrentPosition(0);
      stepperE.setCurrentPosition(0);
      stepperQ.setCurrentPosition(0);
      Serial.print("Echo: Positions ");
      Serial.print(stepperX.currentPosition());
      Serial.print(" ");
      Serial.print(stepperY.currentPosition());
      Serial.print(" ");
      Serial.print(stepperZ.currentPosition());
      Serial.print(" ");
      Serial.print(stepperE.currentPosition());
      Serial.print(" ");
      Serial.print(stepperQ.currentPosition());
      Serial.println(" 0");
      break;
    default:
      break;
  }
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
      while (millis() - curtime < 5000){
        anticlockwiseX();
      }
    }
    else if (p1 > 0) {
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        clockwiseX();
      }
    }
    if (p2 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        anticlockwiseY();
      }
    }
    else if (p2 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        clockwiseY();
      }
    }
    if (p3 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        anticlockwiseZ();
      }
    }
    else if (p3 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        clockwiseZ();
      }
    }
    if (p4 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        anticlockwiseE();
      }
    }
    else if (p4 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        clockwiseE();
      }
    }
    if (p5 < 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        anticlockwiseQ();
      }
    }
    else if (p5 > 0){
      unsigned long curtime;
      curtime = millis();
      while (millis() - curtime < 5000){
        clockwiseQ();
      }
    }
    stateChanged = true;
  } 
  else {
    Serial.println("Echo: wrong amount of input arguments or nonvalid inputs");
  }
}
