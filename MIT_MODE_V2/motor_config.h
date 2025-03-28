/******************************************************************************************
  AUTHOR: Rajiv Joshi

  PURPOSE: Header file defines prototype functions for motor_config.cpp file.

  UPDATED: 03/05/2025
******************************************************************************************/
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// Includes:
#include <Arduino.h>




/******************************************************************************************
                                  Data Strucutres:
******************************************************************************************/
struct MotorParams { // Structure of motor parameters
  const char* motor_type;
  float position_limits[2];  // {min, max}
  float velocity_limits[2];  // {min, max}
  float torque_limits[2];    // {min, max}
  float rated_torque_limits[2]; // {min, max}
  float kp_limits[2];       // {min, max}
  float kd_limits[2];       // {min, max}
};




/******************************************************************************************
                                  Funciton Prototypes:
******************************************************************************************/
MotorParams* getMotorLimits(const char* motor_type);
void printMotorParams(MotorParams* motor);




#endif // end of header file



