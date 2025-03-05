/****************************************************************************************** 
  AUTHOR: Rajiv Joshi

  PURPOSE: For understanding of how Arduino implementaion of AK motors CAN + Teensy 
  works. Implements both SERVO mode and MIT mode for control.

  ISSUES: MIT mode is having a delay issue when requesting to read position. Causes a bump
  to be felt in the motors motion when rotating.

  UPDATED: 02/25/2025
******************************************************************************************/
// Includes:
#include <FlexCAN_T4.h> // include the FlexCAN_T4 library for teensy board

// Formatting CAN channel to use and Buffer + Tx size
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // uses CAN 3 on Teensy (CTX3 and CRX3)

// Global Variables
CAN_message_t msg;
float motor_pos;
float motor_spd;
float motor_t;

/******************************************************************************************** Helper Functions ********************************************************************************************/
/*********************************************
  FUNCTION: Adds data to the message buffer, 
  specifies the CAN message length, and writes 
  the CAN signal to the bus.
*********************************************/
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) { // pass in the ID, data, and length of the data
  msg.id = id; // idea for arbitration (aka deciding which CAN signal will get to use the bus)
  msg.len = len; // how long the legth of the data is
  memcpy(msg.buf, data, len); // copys the data to the message buffer at the specified length
  can1.write(msg); // execution of writing the CAN signal with data to the bus
}

/*********************************************
  FUNCTION: Extended Version, Adds data to 
  the message buffer, specifies the CAN 
  message length, and writes the CAN signal 
  to the bus.
*********************************************/
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) { // pass in the ID, data, and length of the data
  msg.id = id; // idea for arbitration (aka deciding which CAN signal will get to use the bus)
  msg.len = len; // how long the legth of the data is
  msg.flags.extended = 1; // 29 for extended CAN ID length
  memcpy(msg.buf, data, len); // copys the data to the message buffer at the specified length
  can1.write(msg); // execution of writing the CAN signal with data to the bus
}

/*********************************************
  FUNCTION: Adding a int16 to the data buffer
  Consdier the int16_t = 0x1234
  This function will split the data in to 4
  parts since our buffer is a uint8_t: 
  [0x12 0x34]
*********************************************/
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) { // variables passed in, buffer size is passed in here (how long is the data buffer), index is where we start filling in
  buffer[(*index)++] = number >> 8; // using our example: 0x1234 >> 8 = 0x12 so this is stored into buffer[0]. index pointer is incremented to 1
  buffer[(*index)++] = number; // Next: 0x1234 = 0x1234 so the leats significat byte is stored: 0x78 into buffer[1], index = 2
}

/*********************************************
  FUNCTION: Adding a uint16 to the data buffer
  Consdier the uint16_t = 0x1234
  This function will split the data in to 4
  parts since our buffer is a uint8_t: 
  [0x12 0x34]
*********************************************/
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index) { // variables passed in, buffer size is passed in here (how long is the data buffer), index is where we start filling in
  buffer[(*index)++] = number >> 8; // using our example: 0x1234 >> 8 = 0x12 so this is stored into buffer[0]. index pointer is incremented to 1
  buffer[(*index)++] = number; // Next: 0x1234 = 0x1234 so the leats significat byte is stored: 0x78 into buffer[1], index = 2
}

/*********************************************
  FUNCTION: Adding a int32 to the data buffer
  Consdier the int32_t = 0x12345678
  This function will split the data in to 4
  parts since our buffer is a unit8_t: 
  [0x12 0x34 0x56 0x78]
*********************************************/
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) { // variables passed in, buffer size is passed in here (how long is the data buffer)
  buffer[(*index)++] = number >> 24; // using our example: 0x12345678 >> 24 = 0x12 so this is stored into the buffer[0], the index pointer gets incremented to 1
  buffer[(*index)++] = number >> 16; // Next: 0x12345678 >> 16 = 0x1234 so the leats significat byte is stored: 0x34 into buffer[1], index = 2
  buffer[(*index)++] = number >> 8; // Next: 0x12345678 >> 8 = 0x123456 so the leats significat byte is stored: 0x56 into buffer[2], index = 3
  buffer[(*index)++] = number; // Next: 0x12345678 = 0x12345678 so the leats significat byte is stored: 0x78 into buffer[3], index = 4
}

/*********************************************
  FUNCTION: Adding a uint32 to the data buffer
  Consdier the uint32_t = 0x12345678
  This function will split the data in to 4
  parts since our buffer is a unit8_t: 
  [0x12 0x34 0x56 0x78]
*********************************************/
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) { // variables passed in, buffer size is passed in here (how long is the data buffer)
  buffer[(*index)++] = number >> 24; // using our example: 0x12345678 >> 24 = 0x12 so this is stored into the buffer[0], the index pointer gets incremented to 1
  buffer[(*index)++] = number >> 16; // Next: 0x12345678 >> 16 = 0x1234 so the leats significat byte is stored: 0x34 into buffer[1], index = 2
  buffer[(*index)++] = number >> 8; // Next: 0x12345678 >> 8 = 0x123456 so the leats significat byte is stored: 0x56 into buffer[2], index = 3
  buffer[(*index)++] = number; // Next: 0x12345678 = 0x12345678 so the leats significat byte is stored: 0x78 into buffer[3], index = 4
}

/*********************************************************************************************** SERVO MODE ***********************************************************************************************/
/*********************************************
AK 80-9 has 7 different Servo Contorl Modes
*********************************************/
enum CAN_PACKET_ID { // enum assigns automatically a paired named-integer constant to this predefined limited list
  CAN_PACKET_SET_DUTY = 0, // 0: Specifies the motors Duty Cycle voltage in a square wave driving form.
  CAN_PACKET_SET_CURRENT, // 1: Specifies the Iq current of the motor. As the motor output torque = Iq * KT, it can be used as a torque loop.
  CAN_PACKET_SET_CURRENT_BRAKE, // 2: Specifies the braking current of the motor to fix the motor at the current position (pay attention to motor temperature during use).
  CAN_PACKET_SET_RPM, // 3: Specifies the running speed of the motor.
  CAN_PACKET_SET_POS, // 4: Specifies the position of the motor, and the motor will run to the specified position at the maximum speed.
  CAN_PACKET_SET_ORIGIN_HERE, // 5: Specifies a temporary origin (zero) position (clears after power-off), or a permanent zero point (for dual encoder modes only).
  CAN_PACKET_SET_POS_SPD // 6: Specifies the position, speed, and acceleration of the motor. The motor will run to the specified position with the given acceleration and maximum speed.
};

/*********************************************
  FUNCTION: Servo Mode set Duty Cycle for 
  AK 80-9 Motor. Check datasheet for 
  understanding communcaition data format.

  Data Buffer Format:
    Range:     [         0~0xFF      ,         0~0xFF       ,          0~0xFF       ,         0~0xFF       ]
    Variables: [duty cycle 25-32 bits, duty cycle 17-24 bits, duty cycle 9 - 16 bits, duty cycle 1 - 8 bits]

*********************************************/
void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty*100000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/*********************************************
  FUNCTION: Servo Mode set current for AK 80-9
  Motor. Check datasheet for understanding
  communcaition data format.

  Data Buffer Format:
    Range:     [       0~0xFF     ,       0~0xFF      ,        0~0xFF      ,       0~0xFF      ]
    Variables: [current 25-32 bits, current 17-24 bits, current 9 - 16 bits, current 1 - 8 bits]

*********************************************/
void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current*1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/*********************************************
  FUNCTION: Servo Mode set Brake Current for 
  AK 80-9 Motor. Check datasheet for 
  understanding communcaition data format.

  Data Buffer Format:
    Range:     [          0~0xFF        ,          0~0xFF         ,           0~0xFF         ,          0~0xFF         ]
    Variables: [brake current 25-32 bits, brake current 17-24 bits, brake current 9 - 16 bits, brake current 1 - 8 bits]

*********************************************/
void comm_can_set_cb(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current*1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/*********************************************
  FUNCTION: Servo Mode set speed for AK 80-9
  Motor. Check datasheet for understanding
  communcaition data format.

  Data Buffer Format:
    Range:     [      0~0xFF    ,      0~0xFF     ,       0~0xFF     ,      0~0xFF     ]
    Variables: [speed 25-32 bits, speed 17-24 bits, speed 9 - 16 bits, speed 1 - 8 bits]

*********************************************/
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(rpm), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

/*********************************************
  FUNCTION: Servo Mode set position for AK 80-9
  Motor. Check datasheet for understanding
  communcaition data format.

  Data Buffer Format:
    Range:     [       0~0xFF      ,       0~0xFF       ,        0~0xFF       ,       0~0xFF       ]
    Variables: [position 25-32 bits, position 17-24 bits, position 9 - 16 bits, position 1 - 8 bits]

*********************************************/
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

/*********************************************
  FUNCTION: Servo Mode setOrigin Function 
  for AK 80-9 Motor. Check datasheet for
  Understanding communcaition data format.

  Data Buffer Format:
    Range:     [                   0~0x2                   ]
    Variables: [0 (temp origin) or 1 (permanant zero point)]

*********************************************/
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
    int32_t send_index = 0; // defining our index variable and initialize to 0
    uint8_t buffer[8] = {0}; // defines a buffer of size 8 where each spot is of size uint8_t
    buffer[0] = set_origin_mode; // adds the data for origin to the first spot
    send_index = 1; // increment the index value our buffer has a length of 1

    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index); // using our funciton to piece CAN data together for writting to bus
}

/*********************************************
  FUNCTION: Servo Mode set position and speed
  for AK 80-9 Motor. Check datatsheet for
  understanding communication data format

  Data Buffer Format:
    Range:     [       0~0xFF      ,       0~0xFF       ,       0~0xFF      ,      0~0xFF      ,      0~0xFF      ,      0~0xFF     ,      0~0xFF      ,     0~0xFF      ]
    Variables: [position 25-32 bits, position 17-24 bits, position 9-16 bits, positoin 1-8 bits, speed high bits 8, speed low bits 8, accel high bits 8, accel low bits 8]

*********************************************/
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA) {
    int32_t send_index = 0;
    uint8_t buffer[8] = {0};
    
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    buffer_append_int16(buffer, (int16_t)(spd / 10.0), &send_index);
    buffer_append_int16(buffer, (int16_t)(RPA / 10.0), &send_index);
    
    Serial.print(controller_id);
    Serial.print(" pos: ");
    Serial.print(pos);
    Serial.print(" spd: ");
    Serial.print(spd);
    Serial.print(" RPA: ");
    Serial.println(RPA);

    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, 8);
    
}

/*********************************************
  FUNCTION: For when motor sends a CAN signal
  back to the Teensy board. Check datasheet 
  for seeing format of how and what 
  AK 80-9 sends information.

  Data Buffer Format: 
    Range:     [    0~0xFF       ,     0~0xFF      ,     0~0xFF       ,     0~0xFF      ,     0~0xFF           ,     0~0xFF          ,     0~0xFF        ]
    Variables: [pos high (8 bits), pos low (8 bits), spd high (8 bits), spd low (8 bits), current high (8 bits), current low (8 bits), temperature, error]

*********************************************/
void motor_receive_servo(const CAN_message_t &msg) {
  float motor_cur;
  int8_t temp;
  int8_t error;

  // Assigning data buffer to corresponding variables
  // Consider buffer = [0x12 0x34 0x56 0x78 0x98 0x76]
  int16_t pos_int = (int16_t)((msg.buf[0] << 8) | msg.buf[1]); // puts together the first two 8 bit numbers: 0x12 | 0x34 = 0x1234
  int16_t spd_int = msg.buf[2] << 8 | msg.buf[3]; // Example: 0x56 | 0x78 = 0x5678
  int16_t cur_int = msg.buf[4] << 8 | msg.buf[5]; // Example: 0x98 | 0x76 = 0x9876

  // Conversions for position, speed and current
  motor_pos = (float)(pos_int * 0.1f); // degrees
  motor_spd = (float)(spd_int * 10.0f); // ERPM
  motor_cur = (float)(cur_int * 0.01f); // ...

  // assigning last parts of data to corresponding variables
  temp = msg.buf[6]; // ...
  error = msg.buf[7]; // ...

  //Depending on the ID for the motor that was sent
  if (msg.id == 0x2968) {
    Serial.print("[Motor 1] ");
  }

  // print out the converted motor parameters
  Serial.print("Pos: ");
  Serial.print(motor_pos);
  Serial.print(" Spd: ");
  Serial.print(motor_spd);
  Serial.print(" Cur: ");
  Serial.print(motor_cur);
  Serial.print(" Temp: ");
  Serial.print(temp);
  Serial.print(" Err: ");
  Serial.println(error);
}

/************************************************************************************************ MIT MODE ************************************************************************************************/
/*********************************************
  FUNCTION: Set motor control mode to on
  NOTE: must do this before before CAN
  communication.
*********************************************/
void set_mit_mode_run(void) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
    comm_can_transmit_sid(0x01, buffer, 8);
}
/*********************************************
  FUNCTION: IDK what this does yet look into
  this.
*********************************************/
void set_mit_mode_idle(void) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };
    comm_can_transmit_sid(0x01, buffer, 8);
}
/*********************************************
  FUNCTION: Sets the current postion as the
  zero point.
*********************************************/
void set_mit_current_position_zero_positon(void) {
    uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE };
    comm_can_transmit_sid(0x01, buffer, 8);
}

/*********************************************
  FUNCTION: Conversion of float type to 
  uint type variable
*********************************************/
unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return ((x - x_min) * ((float)((((long)1 << bits) - 1) / span)));  
}

/*********************************************
  FUNCTION: Conversion of uint type to 
  float type variable
*********************************************/
float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)(((long)1 << bits) - 1)) + offset;
}

/*********************************************
  NOTE: Motor paramters. To get these use the
  CUBEMARS software...
*********************************************/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define KP_MIN 0
#define KP_MAX 500.0f
#define KD_MIN 0
#define KD_MAX 5.0f

/*********************************************
  FUNCTION: used to send a command to the
  Motor.
*********************************************/
void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff) {

    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    uint8_t buffer[8];

    /// pack ints into the can buffer ///
    buffer[0] = p_int >> 8;                            // position high 8 bits
    buffer[1] = p_int & 0xFF;                          // position low 8 bits
    buffer[2] = v_int >> 4;                            // speed high 8 bits
    buffer[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);  // speed low 4 bits KP high 4bits
    buffer[4] = kp_int & 0xFF;                         // KP low 8 bits
    buffer[5] = kd_int >> 4;                           // KD high 8 bits
    buffer[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);  // KP low 4 bits Torque High 4 bits
    buffer[7] = t_int & 0xff;                          // Torque low 8 bits

    comm_can_transmit_sid(controller_id, buffer, 8);
}

void motor_receive_mit(const CAN_message_t &msg) {
  int8_t temp;
  int8_t error;

  int8_t ID = msg.buf[0];
  uint16_t pos_int = (uint16_t)((msg.buf[1] << 8) | (msg.buf[2]));
  uint16_t spd_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
  uint16_t t_int = ((msg.buf[4] & 0xF) << 8) | (msg.buf[5]);

  motor_pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
  motor_spd = uint_to_float(spd_int, V_MIN, V_MAX, 12);
  motor_t = uint_to_float(t_int, T_MIN, T_MAX, 12);

  temp = msg.buf[6]; // ...
  error = msg.buf[7]; // ...

  Serial.print("ID: ");
  Serial.print(ID);
  Serial.print(" Position: ");
  Serial.print(motor_pos);
  Serial.print(" Speed: ");
  Serial.print(motor_spd);
  Serial.print(" Torque: ");
  Serial.print(motor_t);
  Serial.print(" Temp: ");
  Serial.print(temp);
  Serial.print(" Error: ");
  Serial.println(error);
}
/*********************************************************************************************** Setup/Loop ***********************************************************************************************/

/*********************************************
  FUNCTION: MAIN function for setting up 
  program/teensy. This will only run one
  time at the very beginning.
*********************************************/
void setup() {
  Serial.begin(115200);
  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps
}

/*********************************************
  FUNCTION: Main loop for program. This will 
  run indefinetly till the program is forced
  to stop or upon disconnect of MCU - Teensy.
*********************************************/
void loop() {

  // ...
  if (Serial.available() > 0) {
    char rc = Serial.read(); // based on the input we send from serial monitor
    Serial.println(rc);

    // depending on rc value that was read enter that case.
    // CAN ID for Motor: 104 <-- get this by using Cubemars software and clicking read parameters then clicking application >> controller ID is the id of the motor
    switch(rc) {
      case 'q': // for setting origin
        comm_can_set_origin(104, 0);
        break;
      case 'w': // for position, speed acceleration case 1
        comm_can_set_pos_spd(104, 30, 20000, 10000);
        break;
      case 'e': // for position, speed acceleration case 1
        comm_can_set_pos_spd(104, 30, 5000, 5000);
        break;
      case 'r': // for position, speed acceleration case 1
        comm_can_set_pos_spd(104, -360, 5000, 5500);
        break;
      case 'a':
        set_mit_mode_run(); // start MIT mode
        break;
      case 's':
        set_mit_current_position_zero_positon(); // set current position as the zero position
        break;
      case 'd':
        pack_cmd(0x01, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_cmd(0x01, 0.0, 0.0, 0.0, 1.0, 5.0);
        break;
      case 'f':
        pack_cmd(0x01, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_cmd(0x01, 3.14, 0.0, 4.0, 1.0, 0.0);
        break;
      case 'g':
        pack_cmd(0x01, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        pack_cmd(0x01, 0.0, 0.0, 3.0, 1.0, 0.0);
        break;
      case 'h':
        pack_cmd(0x01, 0.0, 0.0, 0.0, 0.0, 0.0); // "breaks for motor"
        break;
      case 'j':
        pack_cmd(0x01, 0.0, 0.0, 0.0, 0.0, 0.0); //break before writing a command
        set_mit_mode_idle(); // stop MIT mode
        break;
      
    }
  }

  //when we get a message from the motor
  while (can1.read(msg)) {
    // motor_receive_servo(msg); // call function to read the message
    motor_receive_mit(msg);
  }
}
