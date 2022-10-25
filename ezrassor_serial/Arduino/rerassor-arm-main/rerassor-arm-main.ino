#include <AccelStepper.h>
#include "Ramps.h"
#include <ezButton.h>
#include "PololuDriver.h"

// The X Stepper pins
#define STEPPER_X_DIR_PIN 55
#define STEPPER_X_STEP_PIN 54
#define X_ENABLE_PIN 38

// Y Stepper Pins
#define STEPPER_Y_DIR_PIN 61
#define STEPPER_Y_STEP_PIN 60
#define Y_ENABLE_PIN 56

// Z Stepper Pins
#define STEPPER_Z_DIR_PIN 48
#define STEPPER_Z_STEP_PIN 46
#define Z_ENABLE_PIN 62

// E Stepper Pins
#define STEPPER_E_DIR_PIN 26
#define STEPPER_E_STEP_PIN 28
#define E_ENABLE_PIN 24

// Q Stepper Pins
#define STEPPER_Q_DIR_PIN 36
#define STEPPER_Q_STEP_PIN 34
#define Q_ENABLE_PIN 30

int _delay = 100;
boolean buttonPressed = false;
String tempMoveArr[10] = {""};

boolean X_waitingOnHome = false;
boolean Y_waitingOnHome = false;
boolean Z_waitingOnHome = false;

Ramps ramps = Ramps();

ezButton limitSwitchArray[] = {
  ezButton(X_MAX_PIN),
  ezButton(Y_MAX_PIN),
  ezButton(Z_MAX_PIN)
};

AccelStepper stepperArray[] = {
  AccelStepper(1, STEPPER_X_STEP_PIN, STEPPER_X_DIR_PIN),
  AccelStepper(1, STEPPER_Y_STEP_PIN, STEPPER_Y_DIR_PIN),
  AccelStepper(1, STEPPER_Z_STEP_PIN, STEPPER_Z_DIR_PIN),
  AccelStepper(1, STEPPER_E_STEP_PIN, STEPPER_E_DIR_PIN),
  AccelStepper(1, STEPPER_Q_STEP_PIN, STEPPER_Q_DIR_PIN)  
};

enum {
  motorX = 0,
  motorY = 1,
  motorZ = 2,
  motorE = 3,
  motorQ = 4
};

const String motorNames[] = {
  "[Motor X]",
  "[Motor Y]",
  "[Motor Z]",
  "[Motor E]",
  "[Motor Q]"
};

PololuStepper motorArray[] = {
  ramps.motorX,
  ramps.motorY,
  ramps.motorZ,
  ramps.motorE,
  ramps.motorQ
};


void setup() {
    Serial.begin(9600);

    limitSwitchArray[0].setDebounceTime(50);
    limitSwitchArray[1].setDebounceTime(50);
    limitSwitchArray[2].setDebounceTime(50);

    // Stepper X
    stepperArray[0].setMaxSpeed(8000.0);
    stepperArray[0].setAcceleration(1000);

    stepperArray[0].setEnablePin(X_ENABLE_PIN);
    stepperArray[0].setPinsInverted(false, false, true);
    stepperArray[0].enableOutputs();

    // Stepper Y
    stepperArray[1].setMaxSpeed(8000.0);
    stepperArray[1].setAcceleration(1000);
    
    stepperArray[1].setEnablePin(Y_ENABLE_PIN);
    stepperArray[1].setPinsInverted(false, false, true);
    stepperArray[1].enableOutputs();


    // Stepper Z
    stepperArray[2].setMaxSpeed(9000.0);
    stepperArray[2].setAcceleration(1000);
    
    stepperArray[2].setEnablePin(Z_ENABLE_PIN);
    stepperArray[2].setPinsInverted(false, false, true);
    stepperArray[2].enableOutputs();
//
//    // Stepper E
//    stepperArray[3].setMaxSpeed(9000.0);
//    stepperArray[3].setAcceleration(1000);
//    
//    stepperArray[3].setEnablePin(E_ENABLE_PIN);
//    stepperArray[3].setPinsInverted(false, false, true);
//    stepperArray[3].enableOutputs();

    // Stepper Q
    stepperArray[4].setMaxSpeed(9000.0);
    stepperArray[4].setAcceleration(1000);
    
    stepperArray[4].setEnablePin(Q_ENABLE_PIN);
    stepperArray[4].setPinsInverted(false, false, true);
    stepperArray[4].enableOutputs();

    initialActions();
}


void loop() {
    limitSwitchArray[motorX].loop();
    limitSwitchArray[motorY].loop();
    limitSwitchArray[motorZ].loop();

    // When not going home, check limit switch for every other action.
    enableLimitSwitch();

    // Handle input from serial and choose operation
    readSerial();

    // Keep moving during manual controller operation
    if (buttonPressed == true)
        moveArm(tempMoveArr);

}

// This will run immediately after setup, once.
void initialActions()
{
  //allHome();

  //newAction(-10000, motorX);
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

// ----------------------------------- //
// Process Serial and Choose Operation //
// ----------------------------------- //
void onDataRead(String rxString){  
  
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
    //changeSettings(cmdArray);
    
  } 
  else if (cmdArray[0] == "M"){
    // move the arm    
    buttonPressed = true;
    moveArm(cmdArray);  
  
    size_t cmdLength = sizeof(cmdArray) / sizeof(cmdArray[0]);
      
    for (int i = 0; i < cmdLength; i++) {
      tempMoveArr[i] = cmdArray[i];
    }

  } 
  else if (cmdArray[0] == "H"){
    Serial.println("All Motors Going Home");
    allHome();
    
  } 
   else if (cmdArray[0] == "F"){
    Serial.println("Place");
    place();
    
  } 
   else if (cmdArray[0] == "P"){
    Serial.println("Pickup");
    pickup();
    
  } 
  else if(cmdArray[0] == "A"){
    Serial.println("Automove");
    autoMove();
  }
  else if (cmdArray[0] == "B") {
    buttonPressed = false;
    Serial.println("Echo: Stopped");
  }
  else {
    // unknown command
    Serial.println("-----------");
    Serial.println(cmdArray[0]);
    Serial.println("-----------");
    Serial.println("Echo: Unknown Command");
  }
}
