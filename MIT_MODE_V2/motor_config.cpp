/******************************************************************************************
  AUTHOR: Rajiv Joshi
  DATE: 03/03/2025

  PURPOSE: This list of parameters was last updated on: 26 February 2024
  Sites from which information was sourced: 
  - CubeMars AK page: https://www.cubemars.com/category-155-AK+Series.html

  NOTES: Units of below limits: 
  - Position [rad]
  - Velocity [rad/s]
  - Torque [Nm]
******************************************************************************************/

// Includes:
#include "motor_config.h"




/******************************************************************************************
  STRUCTURE: List that holds all the motor parameters
******************************************************************************************/
MotorParams motor_list[] = {
  {"AK80-9", // 48V operation
    {-12.5, 12.5},    // position_limits
    {-50.0, 50.0},    // velocity_limits
    {-18.0, 18.0},    // torque_limits
    {-9.0, 9.0},      // rated_torque_limits
    {0.0, 500.0},     // kp_limits
    {0.0, 5.0}        // kd_limits
  },
  {"AK80-64", // 24V/48V operation
    {-12.5, 12.5},    // position_limits
    {-50.0, 50.0},    // velocity_limits
    {-120.0, 120.0},  // torque_limits
    {-48.0, 48.0},    // rated_torque_limits
    {0.0, 500.0},     // kp_limits
  {0.0, 5.0}          // kd_limits
  }
};
int num_motors = sizeof(motor_list) / sizeof(MotorParams);




/******************************************************************************************
  FUNCTION: Getter function to aquire a desired motor's parameters.
******************************************************************************************/
MotorParams* getMotorLimits(const char* motor_type) {
  for (int i = 0; i < num_motors; i++) {
    if (strcmp(motor_list[i].motor_type, motor_type) == 0) { // compare if strings match
      return &motor_list[i]; // if they match we want to return the motors parameters, return exits function
    }
  }
  // below only occurs when they don't match
  Serial.print(motor_type); //print the motor name
  Serial.println(" is not a valid motor type."); //Error message
  return nullptr;
}




/******************************************************************************************
  FUNCTION: Prints out the desired motor's paramters to the Serial Monitor.
******************************************************************************************/
void printMotorParams(MotorParams* motor) {
  Serial.print("Motor Type: "); Serial.println(motor->motor_type);
  Serial.print("Position Limits: "); Serial.print(motor->position_limits[0]); Serial.print(", "); Serial.println(motor->position_limits[1]);
  Serial.print("Velocity Limits: "); Serial.print(motor->velocity_limits[0]); Serial.print(", "); Serial.println(motor->velocity_limits[1]);
  Serial.print("Torque Limits: "); Serial.print(motor->torque_limits[0]); Serial.print(", "); Serial.println(motor->torque_limits[1]);
  Serial.print("Rated Torque Limits: "); Serial.print(motor->rated_torque_limits[0]); Serial.print(", "); Serial.println(motor->rated_torque_limits[1]);
  Serial.print("Kp Limits: "); Serial.print(motor->kp_limits[0]); Serial.print(", "); Serial.println(motor->kp_limits[1]);
  Serial.print("Kd Limits: "); Serial.print(motor->kd_limits[0]); Serial.print(", "); Serial.println(motor->kd_limits[1]);
}




// END OF FILE