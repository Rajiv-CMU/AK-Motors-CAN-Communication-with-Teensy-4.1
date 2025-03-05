// Rajiv Joshi - Recieve CAN from AK 80-9

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize CAN bus
  can1.begin(); // must be called other wise teensy will hard fault
  can1.setBaudRate(1000000);  // Set CAN baud rate to 1Mbps

  // Enable FIFO Interrupts
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff); // when we recieve a CAN message from the Motor
  can1.mailboxStatus();

  Serial.println("CAN bus initialized with FIFO support...\n");

}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();

  // Information from the motor is sent in a buffer for information check AK80-9 sheet: 5.2.1 Servo Mode CAN Upload Message Protocol
  int16_t pos_int = (msg.buf[0] << 8) | msg.buf[1]; //position: using | operation to merge the high and low bits of data. Reconstruction of 16 bit integer
  int16_t spd_int = (msg.buf[2] << 8) | msg.buf[3]; // speed: using | operation to merge the high and low bits of data. Reconstruction of 16 bit integer

  // convert data back into proper units
  float position = pos_int * 0.1f; // conversion to degrees
  float speed = spd_int*10.0f; // conversion to ERPM

  // printout of data
  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Speed: ");
  Serial.println(speed);

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  delay(1000); // small delay on each loop
}