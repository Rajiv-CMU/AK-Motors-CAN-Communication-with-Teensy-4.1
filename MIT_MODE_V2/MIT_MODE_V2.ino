/******************************************************************************************
  AUTHOR: Rajiv Joshi
  DATE: 03/03/2025

  PURPOSE: For understanding of how Arduino implementaion of AK motors CAN + Teensy 
  works.
******************************************************************************************/

// Includes:
#include <FlexCAN_T4.h>
#include "motor_config.h"
#include "motors.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // uses CAN 1 on Teensy (CTX1 and CRX1)
CAN_message_t msg;
MotorData Motor_1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // enables serial communication
  delay(1000); // 1 second dely to allow initialization

  // Enable FIFO Interrupts
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canMessageReceived); // when we recieve a CAN message from the Motor
  can1.mailboxStatus();

  Motor_1.motor_id = 0x01;
  Motor_1.motor_type = "AK80-9";
}


void canMessageReceived(const CAN_message_t &msg) {
  if(msg.buf[0] == 0x01) {
    unpack_motor_message(Motor_1, msg, Motor_1.motor_type);
  }
}

void loop() {

  if (Serial.available() > 0) {
    char rc = Serial.read(); // based on the input we send from serial monitor
    Serial.println(rc);

    // depending on rc value that was read enter that case.
    // CAN ID for Motor: 104 <-- get this by using Cubemars software and clicking read parameters then clicking application >> controller ID is the id of the motor
    switch(rc) {
      case 'a':
        // set_mit_mode_run(); // start MIT mode
        break;
      case 's':
        // set_mit_current_position_zero_positon(); // set current position as the zero position
        break;
      case 'd':
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 1.0, 5.0); // sample torque control
        break;
      case 'f':
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 3.14, 0.0, 4.0, 1.0, 0.0); // sample move to pi rad position control
        break;
      case 'g':
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 4.0, 1.0, 0.0); // sample move to 0 rad position control
        break;
      case 'h':
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        break;
      case 'j':
        pack_motor_message( Motor_1.motor_id, Motor_1.motor_type, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        // set_mit_mode_idle(); // stop MIT mode
        break;
    }
  }
}




// END OF FILE