/*
   This is the header file that is being used to store the global variables,
   configuration variables, kinematic chain map object and EEPROM logger object.
   These objects and variables are used by almost all the other files and hence
   are stored here.
*/
#ifndef _FUNCTION_H
#define _FUNCTION_H

//row below was commented out causing issues with the XBOXONE and USB variables
// #include <XBOXONE.h>
// Including the libraries for the IMU - acclerometer
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>

// Including library for the matrix elements in the code
#include <BasicLinearAlgebra.h>

#include <AccelStepper.h>
#include <EEPROM.h>

// state machine enumeration.
enum StateMachineState {
  MenuMode = 0,
  RobotRun = 1,
  EyeCalibrate = 2,
  ShoulderCalibrate = 4,
};

// EEPROM address enumeration for bytes containing calibration values and last known positions
enum PromAddress {
  LXCenterAddress = 12,
  LZCenterAddress = 16,
  RXCenterAddress = 20,
  RZCenterAddress = 24,
  XStepperPositionAddress = 0,
  YStepperPositionAddress = 4,
  ZStepperPositionAddress = 8,
  LastPhiR = 28,
  LastPhiS = 32,
  LastPhiD = 36,
  YawServoCalibrationCenter = 40,
};

// declaring global variables as extern. They are declared
// in the source file for this header.

// Hardware Serial object for comms with API
// Hardware Serial is just a pointer to Serial1.
extern HardwareSerial *SerialTerminal;


// String array containing the state names.
extern String stateNames[12];

// Declaring Servo Relat Pin
extern int servoRelay;

// Declaring Shoulder Limit Switches and Stepper Pins - check wiring diagram if you have any questions
// Limit switch pins
extern int xEnd;
extern int yEnd;
extern int zEnd;
// Stepper Pins for X,Y,Z steppers; Dir-Direction;
extern int xDir;
extern int xPulse;
extern int yDir;
extern int yPulse;
extern int zDir;
extern int zPulse;

// Declaring the Neck Stepper Direction and Pulse Pins.

// NECK Stepper Direction and Pulse Pins
extern int frontDir; 
extern int frontPulse; 
extern int backRightDir; 
extern int backRightPulse; 
extern int backLeftDir;  
extern int backLeftPulse; //50 on PCB;

//Used to invert a matrix. We assume that the matrix is always going to be non singular. actual function in function.cpp
BLA::Matrix<4,4> InvNonSingular(BLA::Matrix<4,4> in);


#endif
