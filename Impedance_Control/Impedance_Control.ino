/******************************************************************************************
  AUTHOR: Rajiv Joshi

  PURPOSE: For understanding of how Arduino implementaion of AK motors CAN + Teensy 
  works. This is the 

  UPDATED: 03/05/2025
******************************************************************************************/

// Includes:
#include <FlexCAN_T4.h>
#include "motor_config.h"
#include "motors.h"

// Define Global Variables:
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // uses CAN 1 on Teensy (CTX1 and CRX1)
CAN_message_t msg;
bool impedance_control = false;

MotorData Motor_1; // Motor 1 created




void Impedance_controler(MotorData motor) {
  float torque_spring;
  float torque_damp;
  float torque_d;
  float pos_actual;
  float velocity_actual;

  const float spring = 5;
  const float damp = 0.005;
  const float pos_d = 0.0;

  pos_actual = motor.pos;
  velocity_actual = motor.vel;

  torque_spring = spring*(pos_d - pos_actual);
  torque_damp = damp*(velocity_actual);
  torque_d = torque_spring + torque_damp;
  pack_motor_message( motor.motor_id, motor.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
  pack_motor_message( motor.motor_id, motor.motor_type, 0.0, 0.0, 0.0, 0.5, torque_d); // sample torque control
  Serial.print("Postion (rad): ");
  Serial.println(motor.pos);
}




/******************************************************************************************
  FUNCTION: Setup function that intitializes motor, parameters, and other variables upon
  the first startup of the code.
******************************************************************************************/
void setup() {

  Serial.begin(115200); // enables serial communication
  delay(1000);

  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps
  delay(1000);

  // Enable FIFO Interrupts
  can1.enableFIFO(); // enable First in First Out
  can1.enableFIFOInterrupt(); // Enable Interrupts
  can1.onReceive(canMessageReceived); // When we interrupt on a revieced message from other devices
  can1.mailboxStatus(); // setting up all mailboxes (look more into how mailboxes work for FlexCAN library)
  delay(1000);
  // Label/Link motors to ID and type
  Motor_1.motor_id = 0x02; // decalring motor 1 to have an ID of 1. Note that the ID of the motor must be set using CubeMars software. This is just so that my code recognizes 0x01 ID as motor 1.
  Motor_1.motor_type = "AK80-9"; // declaring motor 1 to be an AK80-9 <- mainly for reteriving correct limits/specifications for motor.

  // Clear all relavent Motors information
  zero_motor_data(Motor_1);

  set_mit_mode_run(Motor_1.motor_id); // start MIT mode
  set_mit_current_position_zero_positon(Motor_1.motor_id); // set current position as the zero position

  Serial.print("Motor 1: ");
  Serial.println(Motor_1.motor_type);
  Serial.print("CAN ID: ");
  Serial.println(Motor_1.motor_id);
}




/******************************************************************************************
  FUNCTION: Depending on message ID that is recieved we want to call the correct function
  to unpack and read that message.
******************************************************************************************/
void canMessageReceived(const CAN_message_t &msg) {
  if(msg.buf[0] == 0x02) { // if ID of message is motor 1
    unpack_motor_message(Motor_1, msg, Motor_1.motor_type); // function call to corresponding motor 1 message unpack
  }
}




/******************************************************************************************
  FUNCTION: Main Loop that runs indefinetly for the MCU. Anything in here will just repeat
  untill the power is cut from the system.
******************************************************************************************/
void loop() {
  if (Serial.available() > 0) {
    char rc = Serial.read(); // based on the input we send from serial monitor
    Serial.println(rc);

    // depending on rc value that was read enter that case.
    // CAN ID for Motor: 104 <-- get this by using Cubemars software and clicking read parameters then clicking application >> controller ID is the id of the motor
    switch(rc) {
      case 'z':
        set_mit_current_position_zero_positon(Motor_1.motor_id); // set current position as the zero position
        zero_motor_data(Motor_1); // Set the stored motor data to 0
        break;
      case 'i':
        impedance_control = true;
        break;
      case 'b':
        impedance_control = false;

        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        set_mit_mode_idle(Motor_1.motor_id); // stop MIT mode
        zero_motor_data(Motor_1); // Set the stored motor data to 0
        break;
      case 's':
        impedance_control = false;
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        break;
    }
  }

  if (impedance_control){
    Impedance_controler(Motor_1);
  }

  delay(10);
}




// END OF FILE