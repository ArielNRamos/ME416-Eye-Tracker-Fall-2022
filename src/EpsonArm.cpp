#include "EpsonArm.h"

EpsonArm::EpsonArm()
{
  // Initialize joint values (RADIANS)
  theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0;

  // Initialize the homogeneous transformation matrices
  updateTransformations();
}

void EpsonArm::begin(Stream &serialPort)
{
  _serial = &serialPort;
}

bool EpsonArm::moveArm(float j1, float j2, float j3, float j4, float j5, float j6, int timeout)
{
  // Convert the values from degrees to radians
  j1 = j1 * M_PI / 180;
  j2 = j2 * M_PI / 180;
  j3 = j3 * M_PI / 180;
  j4 = j4 * M_PI / 180;
  j5 = j5 * M_PI / 180;
  j6 = j6 * M_PI / 180;
  
  // Bounds checking on the angles
  j1  = boundInput(j1, joint1_min, joint1_max);
  j2  = boundInput(j2, joint1_min, joint1_max);
  j3  = boundInput(j3, joint1_min, joint1_max);
  j4  = boundInput(j4, joint1_min, joint1_max);
  j5  = boundInput(j5, joint1_min, joint1_max);
  j6  = boundInput(j6, joint1_min, joint1_max);
    
  // Send transmission to the Epson Arm (convert float to ascii, create one long character array) (dtostrf())
  /* NEEDS IMPLEMENTED */

  // Wait for a response w/ timeout
  long start = millis();
  int i = 0;
  uint8_t msg[256] = { '\0' };

  while (millis() - start < timeout)
    if (_serial->available() > 0)
    {
      while (_serial->available() > 0)
      {
        // Read in characters until full message is received, or max message length is read
        msg[i++] = _serial->read();

        if (i > 255)
          break;
      }

      break;
    }

  // If received a response
  if (i != 0 && i < 256)
  {
    // Parse the response (tokenize the string, convert ascii values to floats for joint angles)
    const char delim = ' ';
    char* token = strtok(msg, delim);

    j1 = (float)atof(token);
    
    token = strtok(NULL, delim);
    j2 = (float)atof(token);

    token = strtok(NULL, delim);
    j3 = (float)atof(token);

    token = strtok(NULL, delim);
    j4 = (float)atof(token);

    token = strtok(NULL, delim);
    j5 = (float)atof(token);

    token = strtok(NULL, delim);
    j6 = (float)atof(token);
    
    // Convert degrees to radians
    theta1 = j1 * M_PI / 180;
    theta2 = j2 * M_PI / 180;
    theta3 = j3 * M_PI / 180;
    theta4 = j4 * M_PI / 180;
    theta5 = j5 * M_PI / 180;
    theta6 = j6 * M_PI / 180;

    // Update the transformation matrices
    updateTransformations();

    // Complete!!
    return true;
  }

  // Else, transmission failed or response was longer than expected
  else
  {
    return false;
  }
}

float EpsonArm::boundInput(float j, float lower_bound, float upper_bound)
{
  if (j < lower_bound)
    j = lower_bound;

  else if (j > upper_bound)
    j = upper_bound;

  return j;
}

void EpsonArm::updateTransformations()
{
  // Angles are in radians, distances are in millimeters
  // H01
  H01(0, 0) = sin(theta1);   H01(0, 1) = 0;  H01(0, 2) = cos(theta1); H01(0, 3) = -100*sin(theta1);
  H01(1, 0) = -cos(theta1);  H01(1, 1) = 0;  H01(1, 2) = sin(theta1); H01(1, 3) =  100*cos(theta1);
  H01(2, 0) = 0;             H01(2, 1) = -1; H01(2, 2) = 0;           H01(2, 3) = 472;
  H01(3, 0) = 0;             H01(3, 1) = 0;  H01(3, 2) = 0;           H01(3, 3) = 1;

  // H12
  H12(0, 0) = -sin(theta2);  H12(0, 1) = -cos(theta2);  H12(0, 2) = 0; H12(0, 3) =  400*sin(theta2);
  H12(1, 0) = cos(theta2);   H12(1, 1) = -sin(theta2);  H12(1, 2) = 0; H12(1, 3) = -400*cos(theta2);
  H12(2, 0) = 0;             H12(2, 1) = 0;             H12(2, 2) = 1; H12(2, 3) = 0;
  H12(3, 0) = 0;             H12(3, 1) = 0;             H12(3, 2) = 0; H12(3, 3) = 1;

  // H23
  H23(0, 0) = -cos(theta3);  H23(0, 1) = 0;  H23(0, 2) = sin(theta3);   H23(0, 3) = -30*cos(theta3);
  H23(1, 0) = -sin(theta3);  H23(1, 1) = 0;  H23(1, 2) = -cos(theta3);  H23(1, 3) = -30*sin(theta3);
  H23(2, 0) = 0;             H23(2, 1) = -1; H23(2, 2) = 0;             H23(2, 3) = 0;
  H23(3, 0) = 0;             H23(3, 1) = 0;  H23(3, 2) = 0;             H23(3, 3) = 1;

  // H34
  H34(0, 0) = -cos(theta4);  H34(0, 1) = 0;  H34(0, 2) = sin(theta4);   H34(0, 3) = 0;
  H34(1, 0) = -sin(theta4);  H34(1, 1) = 0;  H34(1, 2) = -cos(theta4);  H34(1, 3) = 0;
  H34(2, 0) = 0;             H34(2, 1) = -1; H34(2, 2) = 0;             H34(2, 3) = -400;
  H34(3, 0) = 0;             H34(3, 1) = 0;  H34(3, 2) = 0;             H34(3, 3) = 1;

  // H45
  H45(0, 0) = -cos(theta5);  H45(0, 1) = 0;  H45(0, 2) = sin(theta5);   H45(0, 3) = 0;
  H45(1, 0) = -sin(theta5);  H45(1, 1) = 0;  H45(1, 2) = -cos(theta5);  H45(1, 3) = 0;
  H45(2, 0) = 0;             H45(2, 1) = -1; H45(2, 2) = 0;             H45(2, 3) = 0;
  H45(3, 0) = 0;             H45(3, 1) = 0;  H45(3, 2) = 0;             H45(3, 3) = 1;

  // H56
  H56(0, 0) = cos(theta6); H56(0, 1) = -sin(theta6);  H56(0, 2) = 0; H56(0, 3) = 0;
  H56(1, 0) = sin(theta6); H56(1, 1) = cos(theta6);   H56(1, 2) = 0; H56(1, 3) = 0;
  H56(2, 0) = 0;           H56(2, 1) = 0;             H56(2, 2) = 1; H56(2, 3) = -80;
  H56(3, 0) = 0;           H56(3, 1) = 0;             H56(3, 2) = 0; H56(3, 3) = 1;
}

BLA::Matrix<4, 4> EpsonArm::getForwardArmTransformation()
{
  BLA::Matrix<4, 4> H06;

  H06 = H01 * H12; // result holds H02
  H06 = H06 * H23; // result holds H03
  H06 = H06 * H34; // result holds H04
  H06 = H06 * H45; // result holds H05
  H06 = H06 * H56; // result holds H06;

  return H06;  
}

BLA::Matrix<4, 4> EpsonArm::getInverseArmTransformation()
{
  BLA::Matrix<4, 4> H06_inv = getForwardArmTransformation();

  return Invert(H06_inv);
}

void EpsonArm::updateJointAngles(float j1, float j2, float j3, float j4, float j5, float j6)
{
  // Convert the values from degrees to radians
  theta1 = j1 * M_PI / 180;
  theta2 = j2 * M_PI / 180;
  theta3 = j3 * M_PI / 180;
  theta4 = j4 * M_PI / 180;
  theta5 = j5 * M_PI / 180;
  theta6 = j6 * M_PI / 180;

  updateTransformations();
}