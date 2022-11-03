/*
   Robot source file containing definitions for all function declarations in
   header file.
*/

#include "EyeTracker.h"
#include "KinematicChain.h"

/*
   Robot class constructor.
*/
EyeTracker::EyeTracker()
{
  this->robotState = MenuMode;
  this->screenDotPos = {0, 0, 0, 1};
  SerialTerminal->println("Created EyeTracker Object");
}

EyeTracker::~EyeTracker()
{
}

/*
   Initialization function for the robot class that
   will create the robot eye, shoulder and neck objects.
*/
void EyeTracker::init()
{
  this->robotEyes.init();
  this->robotShoulder.init();

  // Read the calibration offsets for the eyes from prom
  this->robotEyes.ReadEyeCalibrationVariablesFromProm();

  // Read the last known shoulder position from prom
  this->robotShoulder.ReadShoulderPositionFromProm();

  // Update the kinematic chain after restoring last know positions and calibration offsets
  this->updateKinematicChain();

  this->g_BS = KinematicChain::xform(0, 0, 0, (0.14123), 0.562, (0.23123));
}

/*
   Function to get a command from the SerialTerminal
   that obtains commands from the windows API
*/
char EyeTracker::getSerialCommand()
{
  char command = ' ';
  if (SerialTerminal->available()) {
    command = SerialTerminal->read();
  }
  return command;
}

/*
   Function that will be used to update the kinematic chain instance.
   Calling this function will recalculate the entire kinematic chain for the robot
   going from the eyes to the screen. Therefore this function must be called whenever the robot has moved.
*/
void EyeTracker::updateKinematicChain()
{
//  this->gLS = this->robotEyes.GetInverseLeftEyeTransformation() * this->robotNeck.GetInverseNeckTransformation() * this->robotShoulder.GetInverseShoulderTransformation();
//  this->gRS = this->robotEyes.GetInverseRightEyeTransformation() * this->robotNeck.GetInverseNeckTransformation() * this->robotShoulder.GetInverseShoulderTransformation();
//
//  BLA::Matrix<4, 4> inverseRight = this->robotEyes.GetInverseLeftEyeTransformation();

//  Serial.println("Inverse Right");
//  for (int i =0; i< 4; i++)
//  {
//    for (int j = 0; j<4; j++)
//    {
//      Serial.print(inverseRight(i,j));
//      Serial.print(" ");
//    }
//    Serial.println();
//  }
//
//  BLA::Matrix<4, 4> inverseShoulder = this->robotShoulder.GetInverseShoulderTransformation();
//
//  Serial.println("Inverse Shoulder");
//  for (int i =0; i< 4; i++)
//  {
//    for (int j = 0; j<4; j++)
//    {
//      Serial.print(inverseShoulder(i,j));
//      Serial.print(" ");
//    }
//    Serial.println();
//  }
  
  // : Update this to make the transformation from the screen to the base of the robot separate from the rest of the
  //       transformations. In the current scenario, it is lumped into the robotShoulder transformation. As we move to the
  //       robot arm, this would ideally have transformations of screen to robot base, robot base to robot end effector,
  //       robot end effector to each of the eyes. 
  
  //this->gCS = InvNonSingular(this->gBC) * (this->gBS);
  //this->gLS = this->robotEyes.GetInverseLeftEyeTransformation() *  this->robotShoulder.GetInverseShoulderTransformation();

  // Variable g_BS is the transformation from base to screen, the transformation was moved to Eyetracker to separate it from the Shoulder file.

  this->gLS = this->robotEyes.GetInverseLeftEyeTransformation() *  this->robotShoulder.GetInverseShoulderTransformation() * this->g_BS;
  this->gRS = this->robotEyes.GetInverseRightEyeTransformation() * this->robotShoulder.GetInverseShoulderTransformation() * this->g_BS;

  // // Printout of debugging info 
  // Serial.println("gRS");
  // for (int i =0; i< 4; i++)
  // {
  //   for (int j = 0; j<4; j++)
  //   {
  //     Serial.print(gRS(i,j));
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // }
}



// TODO: I would like to see this whole serial command structure revamped. Ideally, it would read in an entire line of code,
//       split it up into an array of values, and then let each command determine how to process it. Currently, it does an 
//       approach where a single character is read in and then the individual handlers read it element by element. It seems
//       prone to having and error if one command doesn't consume enough data

/*
   Function to obtain serial communication signals from Windows API.
   Resembles the idle state of the machine. A command is sent to change the state
   to achieve the desired functionality.
*/
void EyeTracker::runMenuModeState()
{
  char command = this->getSerialCommand();

  if (command == 'm') { //returns current mode (for now should always be menuMode)
    SerialTerminal->println("Mode: " + stateNames[this->GetState()]);
  }

  else if (command == 's') {
    int stateNumber = SerialTerminal->parseInt();
    this->SetState(stateNumber);
  }
}

/*
   Function to move the desired subsystem to their desired positions.
   This function is called immediately after the g code is sent from the API or serial communication.
*/
void EyeTracker::setRobotPosition(char command)
{
  switch (command) {
    case ('e'): {
        float goX = SerialTerminal->parseFloat();
        float goY = SerialTerminal->parseFloat();

/*
         If Statement for the current bounds of the tablet screen
*/
//        if (goX > .15605 or goX < -.15605 or goY > .110 or goY < -.110)
//        {
//         SerialTerminal->println("Error, values entered outside of max bound");
//         SerialTerminal->println("The Maximum X Bound is -.15605 to .15605 and you entered: ");
//         SerialTerminal->println("The Maximum Y Bound is -.110 to .110 and you entered: ");
//        }
//        
//        else
//        {
          this->screenDotPos(0) = goX;
          this->screenDotPos(2) = goY;
//        }
        
        break;
      }
    case ('s'): {
        float goX = SerialTerminal->parseFloat();
        float goY = SerialTerminal->parseFloat();
        float goZ = SerialTerminal->parseFloat();

        this->robotShoulder.MoveShoulderToPosition(-goX, -goY, -goZ);
        break;
      }
    default: break;
  }

  this->updateKinematicChain();
  // JPS: Why was this commented out?
  // this->robotEyes.ParallaxEyesToPos(this->screenDotPos, this->gLS, this->gRS);
}

/*
   Function to get the current position of the desired subsystem. This function is called
   immediately after the p command is sent from the API or serial communication.
*/
void EyeTracker::getRobotPosition(char command)
{
  switch (command) {
    case ('e'): {
        SerialTerminal->print(this->screenDotPos(0));
        SerialTerminal->print(", ");
        SerialTerminal->println(this->screenDotPos(2));

        break;
      }
    case ('s'): {
        SerialTerminal->print(this->robotShoulder.GetShoulderPosition('x'));
        SerialTerminal->print(", ");
        SerialTerminal->print(this->robotShoulder.GetShoulderPosition('y'));
        SerialTerminal->print(", ");
        SerialTerminal->println(this->robotShoulder.GetShoulderPosition('z'));

        break;
      }
    default: break;
  }
}

/*
   Function that executes the RunRobot state. This function can be used to set the positions
   of the desired subsystem of the robot i.e. obtain manual control of the desired subsystem
   and can also be used to get the positions of the desired subsystem.
*/
void EyeTracker::runRobotRunState()
{
  char command = this->getSerialCommand();

  this->updateKinematicChain();

  if (command == 'g') {
    char newCommand[1];
    SerialTerminal->readBytes(newCommand, 1);

    this->setRobotPosition(newCommand[0]);
  }

  else if (command == 'p') {
    char newCommand[1];
    SerialTerminal->readBytes(newCommand, 1);

    this->getRobotPosition(newCommand[0]);
  }

  else if (command == 'm') {
    this->robotShoulder.WriteShoulderPositionToProm();

    this->SetState(static_cast<int>(MenuMode));
  }

  this->robotEyes.ParallaxEyesToPos(this->screenDotPos, this->gLS, this->gRS);
}

/*
   Function to obtain calibration commands from API and
   subsequently pass message onto Eye subsystem class.
*/
void EyeTracker::runEyeCalibrationState()
{
  char eyeCalCommand = this->getSerialCommand();

  // calling robotEyes object to perform calibration of eyes.
  this->robotEyes.CalibrateEyes(eyeCalCommand, this->gLS, this->gRS);

  // if calibration has been stopped, return to MenuMode;
  if (eyeCalCommand == 'm')
  {
    this->robotEyes.WriteEyeCalibrationVariablesToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Shoulder Home state will home the steppers. Homing the steppers will include moving the
   steppers till they hit the limit switches. Hitting the limit switches will reset the zero locations
   of the steppers.
*/
void EyeTracker::runShoulderCalibrationState()
{
  // Home all steppers function will return false when steppers have finished homing
  bool calibrating = this->robotShoulder.HomeShoulder();

  char command = this->getSerialCommand();

  if (calibrating == false)
  {
    this->updateKinematicChain();
    this->robotShoulder.WriteShoulderPositionToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to obtain calibration commands for the neck from the API
   and subsequently pass them onto the neck object.
*/
//void EyeTracker::runNeckCalibrationState()
//{
//  char neckCalCommand = this->getSerialCommand();
//
//  // calling robotEyes object to perform calibration of eyes.
//  this->robotNeck.CalibrateNeck(neckCalCommand);
//
//  // if calibration has been stopped, return to MenuMode;
//  if (neckCalCommand == 'm')
//  {
//    this->updateKinematicChain();
//
//    // writing last neck position variables to prom
//    this->robotNeck.WriteNeckPositionToProm();
//    this->SetState(static_cast<int>(MenuMode));
//  }
//}

/*
   Function to set the state for the Robot.
*/
void EyeTracker::SetState(int stateNumber)
{
  this->robotState = static_cast<StateMachineState>(stateNumber);
  //SerialTerminal->println("Set New State");
}

/*
   Function to get the current state of the Robot.
*/
int EyeTracker::GetState()
{
  return static_cast<int>(this->robotState);
  //SerialTerminal->println("Current State is " + String(this->robotState));
}

/*
   Function to determine the state of the robot, and run the
   corresponding action of the state.
*/
void EyeTracker::RunState()
{
  switch (this->robotState)
  {
    case (MenuMode):
      {
        (this)->runMenuModeState();
        break;
      }
    case (EyeCalibrate):
      {
        (this)->runEyeCalibrationState();
        break;
      }
    case (ShoulderCalibrate):
      {
        (this)->runShoulderCalibrationState();
        break;
      }
//    case (NeckCalibrate):
//      {
//        (this)->runNeckCalibrationState();
//        break;
//      }
    case (RobotRun):
      {
        (this)->runRobotRunState();
        break;
      }
    default :
      {
        break;
      }
  }
}
