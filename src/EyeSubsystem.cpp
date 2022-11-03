#include "EyeSubsystem.h"
/*
   variable that indicates which motor is being calibrated.
*/
EyeMotor calibrationMotor = rightZ;

/*
   Eye class constructor.
*/
Eyes::Eyes()
{
}

/*
   Initialization function for eye subsystem. Attaches the servo objects
   to their respective pins and sets default value for the center locations.
*/
void Eyes::init()
{
  this->dxl = new Dynamixel2Arduino(Serial3, DXL_DIR_PIN);

  this->dxl->begin(1000000);
  this->dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  /*
     Intializing dynamixel objects.
  */
  // Turn off torque when configuring items in EEPROM area LEFT YAW
  if (dxl->write(DXL_ID_YAW_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left yaw");
  else
    Serial.println("Error: Torque off failed for left yaw");

  // Turn off torque when configuring items in EEPROM area LEFT PITCH
  if (dxl->write(DXL_ID_PITCH_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left pitch");
  else
    Serial.println("Error: Torque off failed for left pitch");

  // Turn off torque when configuring items in EEPROM area RIGHT YAW
  if (dxl->write(DXL_ID_YAW_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left yaw");
  else
    Serial.println("Error: Torque off failed for right yaw");

  // Turn off torque when configuring items in EEPROM area RIGHT PITCH
  if (dxl->write(DXL_ID_PITCH_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left pitch");
  else
    Serial.println("Error: Torque off failed for right pitch");

  // Set to Joint Mode LEFT YAW
  //  if(dxl->write(DXL_ID_YAW_L, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
  //        && dxl.write(DXL_ID_YAW_L, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
  //    Serial.println("Set operating mode");
  //  else
  //    Serial.println("Error: Set operating mode failed for left yaw");

  // Set to Joint Mode Left PITCH
  //  if(dxl->write(DXL_ID_PITCH_L, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
  //        && dxl.write(DXL_ID_YAW_L, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
  //    Serial.println("Set operating mode");
  //  else
  //    Serial.println("Error: Set operating mode failed for left yaw");

  //SECTION TO TURN ON TORQUE (ENABLE MOVEMENT)
  // Turn on torque LEFT YAW
  
  if (dxl->write(DXL_ID_YAW_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for left yaw");
  else
    Serial.println("Error: Torque on failed for left yaw");

  // Turn on torque LEFT PITCH
  if (dxl->write(DXL_ID_PITCH_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for left PITCH");
  else
    Serial.println("Error: Torque on failed for left PITCH");

  // Turn on torque RIGHT YAW
  if (dxl->write(DXL_ID_YAW_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for right yaw");
  else
    Serial.println("Error: Torque on failed for right yaw");

  // Turn on torque RIGHT PITCH
  if (dxl->write(DXL_ID_PITCH_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for right PITCH");
  else
    Serial.println("Error: Torque on failed for right pitch");
      /*
   * Dynamixel PID
   */
   //This namespace is required to use Control Table Item names
   using namespace ControlTableItem;

   // TODO: I DON'T THINK WE NEED TO HAVE THESE IN THE MULTI-ROTATION MODE
   // Turn off multi-turn mode by changing 4095 to 2085


   dxl->writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID_YAW_R, 0);
   dxl->writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID_YAW_L, 0);
   dxl->writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID_PITCH_R, 0);
   dxl->writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID_PITCH_L, 0);

   dxl->writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID_YAW_R, 4095);
   dxl->writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID_YAW_L, 4095);
   dxl->writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID_PITCH_R, 4095);
   dxl->writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID_PITCH_L, 4095);
   
  // left horizontal ID = 1
  dxl->writeControlTableItem(P_GAIN, DXL_ID_YAW_L, LH_P);
  dxl->writeControlTableItem(I_GAIN, DXL_ID_YAW_L, LH_I);
  dxl->writeControlTableItem(D_GAIN, DXL_ID_YAW_L, LH_D);
  dxl->writeControlTableItem(MOVING_SPEED, DXL_ID_YAW_L, Move_speed);

  //left vertical ID = 2

  dxl->writeControlTableItem(P_GAIN, DXL_ID_PITCH_L, LV_P);
  dxl->writeControlTableItem(I_GAIN, DXL_ID_PITCH_L, LV_I);
  dxl->writeControlTableItem(D_GAIN, DXL_ID_PITCH_L, LV_D);
  dxl->writeControlTableItem(MOVING_SPEED, DXL_ID_PITCH_L, Move_speed);

  //right vertical ID = 4

  dxl->writeControlTableItem(P_GAIN, DXL_ID_PITCH_R, RV_P);
  dxl->writeControlTableItem(I_GAIN, DXL_ID_PITCH_R, RV_I);
  dxl->writeControlTableItem(D_GAIN, DXL_ID_PITCH_R, RV_D);
  dxl->writeControlTableItem(MOVING_SPEED, DXL_ID_PITCH_R, Move_speed);


  int tune_P = dxl->readControlTableItem(P_GAIN, DXL_ID_PITCH_L);
  int tune_I = dxl->readControlTableItem(I_GAIN, DXL_ID_PITCH_L);
  int tune_D = dxl->readControlTableItem(D_GAIN, DXL_ID_PITCH_L);


  Serial.print("TUNE PID VALUES:   ");
  Serial.println(tune_P);
  Serial.println(tune_I);
  Serial.println(tune_D);
  
  
  //right horizontal ID = 3

  dxl->writeControlTableItem(P_GAIN, DXL_ID_YAW_R, RH_P);
  dxl->writeControlTableItem(I_GAIN, DXL_ID_YAW_R, RH_I);
  dxl->writeControlTableItem(D_GAIN, DXL_ID_YAW_R, RH_D);
  dxl->writeControlTableItem(MOVING_SPEED, DXL_ID_YAW_R, Move_speed);

  

  // By default go to the center position of the motor.
  this->lXCenter = 2048;
  this->lZCenter = 2048;
  this->rXCenter = 2048;
  this->rZCenter = 2048;

  // setting up the default values for microSecondsPerDegree (empirically determined)
  this->countsPerDegree = 34.1333333;
  
  // writing all servos to middle position
  this->dxl->write(DXL_ID_YAW_L, GOAL_POSITION_ADDR, (uint8_t*)&lXCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT); //writes to left yaw
  this->dxl->write(DXL_ID_PITCH_L, GOAL_POSITION_ADDR, (uint8_t*)&lZCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to left pitch
  this->dxl->write(DXL_ID_YAW_R, GOAL_POSITION_ADDR, (uint8_t*)&rXCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right yaw
  this->dxl->write(DXL_ID_PITCH_R, GOAL_POSITION_ADDR, (uint8_t*)&rZCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right pitch

/*
      T - will be the frame of reference located at the center of the gantry.
      D - will be the frame of reference at the center of the eye mechanisms,
      with Z up, X to the right and Y along the axis of the straight 90-degree eyes
      L - will be the left eye's frame of reference with the origin at the center of rotation
      and is aligned with the D frame when the 90-degree angle is commanded
      R - will be the right eye's frame of reference with the origin at the center of rotation
      and is aligned with D frame when the 90-degree angle is commanded
    */

  // setting initial values to the eye subsystem transformation matrices, units in meters
  this->gTD = KinematicChain::xform(0, 0, 0, (-0.001778), (0.19914), (0.09679)); 
  this->gDL = KinematicChain::xform(0, 0, 0, (-0.03475), 0, 0);
  this->gDR = KinematicChain::xform(0, 0, 0, (0.03475), 0, 0);

  this->gLT = InvNonSingular(gDL) * InvNonSingular(gTD);
  this->gRT = InvNonSingular(gDR) * InvNonSingular(gTD);

  //SerialTerminal->println("Finished setting up eyes");
}

/*
   Parallax function to make eye servos look at
   desired coordinate position. Left and right dot positions indicate the desired
   position on the screen in the frame of reference of the left and right , that the eyes need to look at.
*/
void Eyes::parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos)
{
  // calculating angles of rotation for the left eye
  float alphaLeft = atan2(-leftDotPos(0), leftDotPos(1)) * (180 / M_PI);
  float betaLeft = atan2(leftDotPos(2), sqrt(((leftDotPos(0) * leftDotPos(0))) + (leftDotPos(1) * leftDotPos(1)))) * (180 / M_PI);

  // calculating angles of rotation for the right eye
  float alphaRight  = atan2(-rightDotPos(0), rightDotPos(1)) * (180 / M_PI);
  float betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1)))) * (180 / M_PI);

  Serial.print("bLrad:");
  Serial.print(betaLeft);
  Serial.print("    bRrad:");
  Serial.print(betaRight);


  // converting degrees to counts
  alphaLeft = alphaLeft * this->countsPerDegree;
  betaLeft = betaLeft * this->countsPerDegree;
  alphaRight = alphaRight * this->countsPerDegree;
  betaRight = betaRight * this->countsPerDegree;


  Serial.print("        aLcnt:");
  Serial.print(alphaLeft);
  Serial.print("    bLcnt:");
  Serial.println(betaLeft);
  

  this->goalPosition = this->lXCenter + static_cast<uint16_t>(alphaLeft);
  this->dxl->write(DXL_ID_YAW_L, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT); //writes to left yaw

    Serial.print("LX GOAL POSITION :");
  Serial.println(goalPosition);
  
  this->goalPosition = this->lZCenter - static_cast<uint16_t>(betaLeft);
  this->dxl->write(DXL_ID_PITCH_L, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to left pitch

  
  Serial.print("LZ GOAL POSITION :");
  Serial.println(goalPosition);
  
  this->goalPosition = this->rXCenter + static_cast<uint16_t>(alphaRight);
  this->dxl->write(DXL_ID_YAW_R, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right yaw


  
  this->goalPosition = this->rZCenter + static_cast<uint16_t>(betaRight);
  this->dxl->write(DXL_ID_PITCH_R, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right pitch
  
  int i_present_position = this->dxl->getPresentPosition(1);
  Serial.print("LX PRESENT POSITION: ");
  Serial.println(i_present_position);

  
  i_present_position = this->dxl->getPresentPosition(2);
  Serial.print("LZ PRESENT POSITION: ");
  Serial.println(i_present_position);
  
  delay(50);
}

/*
   Gets the inverse transformation that goes from center of left eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseLeftEyeTransformation()
{
  this->gLT = InvNonSingular(gDL) * InvNonSingular(gTD);
  return this->gLT;
}

/*
   Gets the inverse transformation that goes from the center of right eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseRightEyeTransformation()
{
  this->gRT = InvNonSingular(gDR) * InvNonSingular(gTD);
  return this->gRT;
}

/*
   Function to write the calibration variables
   for Eye Servos to EEPROM.
*/
void Eyes::WriteEyeCalibrationVariablesToProm()
{
  // TODO: I'm not a fan of using this incrementing address. We already have the explicit addresses
  //       in the "enum PromAddress" in Function.h. Use those instead
  // TODO: Find all the locations with EEPROM.put and change to EEPROM.update so that it only writes
  //       if the value is different from the current value (saves cycles on the EEPROM)
  PromAddress eeAddressLX = LXCenterAddress;
  PromAddress eeAddressLZ = LZCenterAddress;
  PromAddress eeAddressRX = RXCenterAddress;
  PromAddress eeAddressRZ = RZCenterAddress;

  EEPROM.update(eeAddressLX, this->lXCenter);
  EEPROM.update(eeAddressLZ, this->lZCenter);
  EEPROM.update(eeAddressRX, this->rXCenter);
  EEPROM.update(eeAddressRZ, this->rZCenter);
  // incrementing the address variable by 4 because
  // it is the size of a floating point data type
  // eeAddress = (PromAddress)((int)eeAddress + 4);
  // EEPROM.put((PromAddress)((int)LZCenterAddress), this->lZCenter);
  // // eeAddress = (PromAddress)((int)eeAddress + 4);
  // EEPROM.put(eeAddress, this->rXCenter);
  // // eeAddress = (PromAddress)((int)eeAddress + 4);
  // EEPROM.put(eeAddress, this->rZCenter);

  //SerialTerminal->println("Eye Calibration Variables written to Prom");
}

/*
   Function to read the calibration variables
   for Eye Servos from EEPROM.
*/
void Eyes::ReadEyeCalibrationVariablesFromProm()
{
  // TODO: See other comment about using the explicit address
  PromAddress eeAddressLX = LXCenterAddress;
  PromAddress eeAddressLZ = LZCenterAddress;
  PromAddress eeAddressRX = RXCenterAddress;
  PromAddress eeAddressRZ = RZCenterAddress;

  //PromAddress eeAddress = LXCenter;
  long int servoCenter = 0.0;

  EEPROM.get(eeAddressLX, servoCenter);
  this->lXCenter = servoCenter;
  //eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddressLZ, servoCenter);
  this->lZCenter = servoCenter;
  //eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddressRX, servoCenter);
  this->rXCenter = servoCenter;
  //eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddressRZ, servoCenter);
  this->rZCenter = servoCenter;

  //SerialTerminal->println("Eye Calibration Variables read from Prom");
}

/*
   Function to make the eyes parallax to a single point
   on the screen. Parameter screenDotPos gives the coordinates of
   the desired position.
*/
void Eyes::ParallaxEyesToPos(BLA::Matrix<4> screenDotPos, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS)
{
  BLA::Matrix<4> leftDotPos = gLS * screenDotPos;
  BLA::Matrix<4> rightDotPos = gRS * screenDotPos;

//  for (int i =0; i< 4; i++)
//  {
//    for (int j = 0; j<4; j++)
//    {
//      Serial.print(gRS(i,j));
//      Serial.print(" ");
//    }
//    Serial.println();
//  }

  // calling private helper function to write to servos.
  this->parallax(leftDotPos, rightDotPos);
}

/*
   Function to calibrate the eye servos. The function takes eyeCalCommand
   as a parameter, which is the signal transmitted by the Windows API.
   This character is used as input to the calibration sequence.
*/
void Eyes::CalibrateEyes(char eyeCalCommand, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS)
{
  // Dot position array corresponding to desired screen location
  // set to zero for calibration to calibrate to center of screen.
  BLA::Matrix<4> screenDotPos = {0, 0, 0, 1};

  if (eyeCalCommand == '1' || eyeCalCommand == '2' || eyeCalCommand == '3' || eyeCalCommand == '4') {
    // converting integer to enumeration
    int motorChoice = eyeCalCommand - '0';
    calibrationMotor = static_cast<EyeMotor>(motorChoice);
  }

  //Serial.println(calibrationMotor);

  // switch case to calibrate the center location of respective eye servo.
  // the center locations are incremented/decremented by 5 microseconds for each press.
  switch (calibrationMotor)
  {
    case (rightZ) : {
        if (eyeCalCommand == 'u') {
          this->rZCenter -= 2;
        }
        if (eyeCalCommand == 'd') {
          this->rZCenter += 2;
        }
        Serial.print("RIGHT Z CENTER TARGET :");
        Serial.println(this->rZCenter);
        break;
      }
    case (leftZ) : {
        if (eyeCalCommand == 'u') {
          this->lZCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->lZCenter -= 2;
        }
        Serial.print("LEFT Z CENTER TARGET :");
        Serial.println(this->lZCenter);
        break;
      }
    case (rightX) : {
        if (eyeCalCommand == 'u') {
          this->rXCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->rXCenter -= 2;
        }
        Serial.print("RIGHT X CENTER TARGET :");
        Serial.println(this->rXCenter);
        break;
      }
    case (leftX) : {
        if (eyeCalCommand == 'u') {
          this->lXCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->lXCenter -= 2;
        }
        Serial.print("LEFT X CENTER TARGET :");
        Serial.println(this->lXCenter);
        break;
      }
    default : break;
  }

  this->ParallaxEyesToPos(screenDotPos, gLS, gRS);
}
