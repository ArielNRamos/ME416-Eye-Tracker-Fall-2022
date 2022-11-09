#ifndef _EPSONARM_H
#define _EPSONARM_H

#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Arduino.h>

class EpsonArm
{
  protected:
    /*
      H01 is the homogeneous transformation matrix for the first joint
      H12 is the homogeneous transformation matrix for the first joint
      H23 is the homogeneous transformation matrix for the first joint
      H34 is the homogeneous transformation matrix for the first joint
      H45 is the homogeneous transformation matrix for the first joint
      H56 is the homogeneous transformation matrix for the first joint

      H06 is the homogeneous transformation matrix for the entire arm, from base to end effector
    */
    BLA::Matrix<4, 4> H01, H12, H23, H34, H45, H56;

  private:
    /* Variables*/
    Stream *_serial; // Base class for serial communication

    float theta1, theta2, theta3, theta4, theta5, theta6;

    float joint1_max = 240, joint1_min = -240;
    float joint2_max = 65, joint2_min = -168;
    float joint3_max = 202, joint3_min = -61;
    float joint4_max = 200, joint4_min = -200;
    float joint5_max = 135, joint5_min = -135;
    float joint6_max = 360, joint6_min = -360;

    /* Functions*/
    // Sends a transmission with a pose to the Epson arm
    bool sendTransmission(float j1, float j2, float j3, float j4, float j5, float j6);
    //bool sendTransmission(float x, float y, float z); // Needs finished as of 11/6/22

    // Bounds the input of the passed joint angle
    float boundInput(float j, float lower_bound, float upper_bound);

    // Updates all the homogeneous transformation matrices
    void updateTransformations();

    // For testing purposes. Updates the joint angles
    void updateJointAngles(float j1, float j2, float j3, float j4, float j5, float j6);

  public:
    // Constructor
    EpsonArm();

    // Pass the serial object used to communicate with the Epson controller
    void begin(Stream &serialPort = Serial3); // Default to Serial3

    // Tells the Epson arm to move to a new position and waits to get the joint angles as a response. timeout in ms
    bool moveArm(float j1 = 0, float j2 = 0, float j3 = 0, float j4 = 0, float j5 = 0, float j6 = 0, int timeout = 15000); 
    //bool moveArm(float x, float y, float z); // Needs finished as of 11/6/22

    // Get the forward transformation of the arm (end effector position in the base frame)
    BLA::Matrix<4, 4> getForwardArmTransformation();

    // Get the inverse transformation of the arm (base frame position in the end effector frame)
    BLA::Matrix<4, 4> getInverseArmTransformation();

    // For testing purposes. Updates the joint angles
    //void updateJointAngles(float j1, float j2, float j3, float j4, float j5, float j6);
};

#endif