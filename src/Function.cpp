/*
   This source file contains declarations for all extern variables
   declared in the header file.
*/

#include "Function.h"

//Adafruit_BNO055 bno = Adafruit_BNO055(55);

HardwareSerial *SerialTerminal = &Serial1;

// String array containing state names corresponding
// to enumeration.
String stateNames[12] = {"MenuMode", "ServoCalibration", "Auto", "StepperHome", "StepperManual", "FindCoordinates",
                         "SetCoordinates", "ServoManual", "NeckCalibration", "MoveToCalibrationState", "Neck", "SpinnyBoi"
                        };

//Declaring Servo relay pin
int servoRelay = 8;

// Declaring Shoulder Limit Switches and Stepper Pins - check wiring diagram if you have any questions
// Limit switch pins
int xEnd = 2; // X axis limit switch pin
int zEnd = 4; // Z axis limit switch pin
int yEnd = 7; // Y axis limit switch pin
// SHOULDER Stepper Direction and Pulse Pins
int xDir = A1;
int xPulse = A0;
int yDir = A3;
int yPulse = A2;
int zDir = A5;
int zPulse = A4;

BLA::Matrix<4,4> InvNonSingular(BLA::Matrix<4,4> in)
{
  BLA::Matrix<4,4> out = in;
  Invert(out);
  return out;
}
