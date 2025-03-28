/****************************************************************************************** 
  AUTHOR: Rajiv Joshi

  PURPOSE: For understanding of how Arduino implementaion of AK motors CAN + Teensy 
  works.

  UPDATED: 03/05/2025
******************************************************************************************/

// Includes:
#include "motors.h"
// check header file for other include definitions.




/************************************************************************************************************************************************************************************ 
                                                                                  Helper Functions:
************************************************************************************************************************************************************************************/
/******************************************************************************************
  FUNCTION: Interpolates an unsigned integer of num_bits length to a floating point number
  between  x_min and x_max.

  ARGS:
      x (int): The int number to convert
      x_min (int): The minimum value for the floating point number
      x_max (int): The maximum value for the floating point number
      num_bits (int): The number of bits for the unsigned integer

  RETURNS:
      float: The floating point representation of the unsigned integer
******************************************************************************************/
float uint_to_float(uint16_t x_int, float x_min, float x_max, int num_bits) {

    float span = x_max - x_min;
    return float(x_int * span / ((1 << num_bits) - 1) + x_min);
}




/******************************************************************************************
  FUNCTION: Interpolates a floating point number to an unsigned integer of num_bits length.
  A number of x_max will be the largest integer of num_bits, and x_min would be 0.

  ARGS:
    x (float): The floating point number to convert
    x_min (float): The minimum value for the floating point number
    x_max (float): The maximum value for the floating point number
    num_bits (int): The number of bits for the unsigned integer

  RETURNS:
    int: The unsigned integer representation of the floating point number
******************************************************************************************/
unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int num_bits) {

    float span = x_max - x_min;
    float bitratio = float((1 << num_bits)/span);
    x = constrain(x, x_min, x_max-(2/bitratio));
    return constrain(int((x - x_min)*(bitratio)), 0, int((x_max - x_min) * bitratio)); 
}




/******************************************************************************************
  FUNCTION: Adds data to the message buffer, specifies the CAN message length, and writes 
  the CAN signal to the bus.
******************************************************************************************/
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) { // pass in the ID, data, and length of the data
  msg.id = id; // idea for arbitration (aka deciding which CAN signal will get to use the bus)
  msg.len = len; // how long the legth of the data is
  memcpy(msg.buf, data, len); // copys the data to the message buffer at the specified length
  can1.write(msg); // execution of writing the CAN signal with data to the bus
}




/******************************************************************************************
  FUNCTION: Sets the current parameters for a given motor to zero.
******************************************************************************************/
void zero_motor_data(MotorData data) {
  data.pos = 0;
  data.vel = 0;
  data.torque = 0;
}




/******************************************************************************************
  FUNCTION: Set motor control mode to on
  NOTE: must do this before before CAN communication.
******************************************************************************************/
void set_mit_mode_run(uint8_t controller_id) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
    comm_can_transmit_sid(controller_id, buffer, 8);
}




/******************************************************************************************
  FUNCTION: Turns off MIT mode
******************************************************************************************/
void set_mit_mode_idle(uint8_t controller_id) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };
    comm_can_transmit_sid(controller_id, buffer, 8);
}




/******************************************************************************************
  FUNCTION: Sets the current postion as the zero point.
******************************************************************************************/
void set_mit_current_position_zero_positon(uint8_t controller_id) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE };
    comm_can_transmit_sid(controller_id, buffer, 8);
}




/******************************************************************************************
  FUNCTION: To unpack a recieved motor command for an AK80-9 to assign its position,
  velocity, and torque to the corresponding variables. Takes in a desired motor that we have
  set up in the "setup" funciton as well as the type of motor it is to update its variables
  for position, velocity, and torque accordingly.
******************************************************************************************/
void unpack_motor_message(MotorData &data, const CAN_message_t &msg, const char* motor_type) {

  MotorParams* motor = getMotorLimits(motor_type);

  // int8_t motor_ID = msg.buf[0]; // Grabbing the ID from the can message ID bit
  uint16_t position_uint = (uint16_t)((msg.buf[1] << 8) | (msg.buf[2])); // Grabbing the position from the CAN message position bits
  uint16_t velocity_uint = ((msg.buf[3] << 8) | (msg.buf[4] >> 4)) >> 4; // Grabbing the velocity from the CAN message velocity bits
  uint16_t torque_uint = ((msg.buf[4] & 0xF) << 8) | (msg.buf[5]); // Grabbing the torque from the CAN message torque bits

  // Assign the converted data to the correct data of our desired motor
  data.pos = uint_to_float(position_uint, motor->position_limits[0], motor->position_limits[1], 16); // convert postion to float
  data.vel = uint_to_float(velocity_uint, motor->velocity_limits[0], motor->velocity_limits[1], 16); // convert velocity to float
  data.torque = uint_to_float(torque_uint, motor->torque_limits[0], motor->torque_limits[1], 16); // convert torque to float
}




/******************************************************************************************
  FUNCTION: To send a desired position, velocity, or torque command to a motor with
  set gains, Kp and Kd.
******************************************************************************************/
void pack_motor_message(uint8_t controller_id, const char* motor_type, float p_des, float v_des, float kp, float kd, float t_ff) {

  MotorParams* motor = getMotorLimits(motor_type); // Grabbing the motor limits based on the input for the motor type (what type of AK motor are we dealing with...)

  // Conversion to uint and clamping the variables to be in the correct limits based on the motor's specifications
  int p_int = float_to_uint(p_des, motor->position_limits[0], motor->position_limits[1], 16); // convert and clamp postion
  int v_int = float_to_uint(v_des, motor->velocity_limits[0], motor->velocity_limits[1], 12); // convert and clamp velocity
  int kp_int = float_to_uint(kp, motor->kp_limits[0], motor->kp_limits[1], 12); // convert and clamp gain Kp
  int kd_int = float_to_uint(kd, motor->kd_limits[0], motor->kd_limits[1], 12); // convert and clamp gain Kd
  int t_int = float_to_uint(t_ff, motor->torque_limits[0], motor->torque_limits[1], 12); // convert and clamp torque

  uint8_t buffer[8]; // Create a buffer of size 8 (we will be adding our inputs for positon, velocity, gains, and torque to this)

  /// pack ints into the can buffer ///
  buffer[0] = p_int >> 8;                               // position high 8 bits
  buffer[1] = p_int & 0x00FF;                           // position low 8 bits
  buffer[2] = v_int >> 4;                               // speed high 8 bits
  buffer[3] = ((v_int & 0x00F) << 4) | (kp_int >> 8);   // speed low 4 bits KP high 4bits
  buffer[4] = kp_int & 0x0FF;                           // KP low 8 bits
  buffer[5] = kd_int >> 4;                              // KD high 8 bits
  buffer[6] = ((kd_int & 0x00F) << 4) | (t_int >> 8);   // KP low 4 bits Torque High 4 bits
  buffer[7] = t_int & 0x0FF;                            // Torque low 8 bits

  comm_can_transmit_sid(controller_id, buffer, 8); // calls function to send can message with relavant information
}




// END OF FILE