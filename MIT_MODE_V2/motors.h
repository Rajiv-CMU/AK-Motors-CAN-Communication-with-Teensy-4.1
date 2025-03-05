/******************************************************************************************
  AUTHOR: Rajiv Joshi
  DATE: 03/03/2025

  PURPOSE: Header file defines prototype functions for motors.cpp file.
******************************************************************************************/
#ifndef MOTORS_H
#define MOTORS_H

// Includes:
#include "motor_config.h"
#include <FlexCAN_T4.h>

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // uses CAN 1 on Teensy (CTX1 and CRX1)
extern CAN_message_t msg;




/******************************************************************************************
                                  Data Strucutres:
******************************************************************************************/
/*********************************************
  STRUCTURE: Motor Data

  NOTE: what information can a motor have?

*********************************************/
struct MotorData {
  int8_t motor_id;        // motor CAN ID [0x__]
  const char* motor_type; // motor type ["AK_-_"]
  float pos;              // motor position [rad]
  float vel;              // motor velocity [rad/s]
  float torque;           // motor torque [Nm]
};




/******************************************************************************************
                                  Funciton Prototypes:
******************************************************************************************/
float uint_to_float(uint16_t x_int, float x_min, float x_max, int num_bits);
unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int num_bits);

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);

void unpack_motor_message(MotorData data, const CAN_message_t &msg, const char* motor_type) ;
void pack_motor_message(uint8_t controller_id, const char* motor_type, float p_des, float v_des, float kp, float kd, float t_ff);




#endif // end of header file
