/*
   This is the main.ino file for the Arduino.
   This file will instantiate an object of class Robot, that subsequently contains
   the Eye, Shoulder and Neck subsystem classes. Communication with the arduino is
   accomplished through a serial channel-> Serial1 that receives messages from a Windows API.
*/

#include "EyeTracker.h"
#include "Function.h"

/*
   Instantiating the Robot class object.
*/
EyeTracker eyeTrackerRobot;

void InitializeComponents();

/*
   Setup() establishes connection with the XBOX controller, Windows API serial comm line,
   Debugging serial comm line, and IMU if available.
*/
void setup() {
  InitializeComponents();
  // initializing EyeTracker Class object
  eyeTrackerRobot.init();
}

/*
   Loop() function will constantly call RunState function from the Robot class.
   This function performs all the decision making for the State Machine.
*/
void loop() {
  eyeTrackerRobot.RunState();
  //Serial.println(eyeTrackerRobot.GetState());
}

/*
   Function to initialize the serial ports, XBOX, BNO objects and turn on
   the servo relay.
*/
void InitializeComponents()
{
  // Debugging Serial
  Serial.begin(115200); //max value

  // Windows API Serial
  Serial1.begin(115200); // should test 230400

  // Dynamixel Serial
  Serial3.begin(1000000);

  // checking for serial connection
  while (!Serial) {
    ;
  }
  Serial.println("Serial ports initialized");

  // Turning on the servo relay 
  // Leftover code for weird startup power issues in the system from many semesters ago
  pinMode(servoRelay, OUTPUT);
  digitalWrite(servoRelay, LOW); //Servo enable using relay - using that to prevent jitter when arduino starts
}
