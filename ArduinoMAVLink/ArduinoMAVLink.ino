// Arduino MAVLink test code.

//#include <FastSerial.h>
#include "C:/Users/luis.Valtierra/Documents/ardupilot/libraries/mavlink/include/mavlink.h"         // Mavlink interface

// FastSerialPort0(Serial);

void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
  // Define the system type (see mavlink_types.h for list of possible types)
  int compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  uint8_t system_type = MAV_QUADROTOR;
  uint8_t autopilot_type = MAV_AUTOPILOT_PIXHAWK;
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message
  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
  mavlink_msg_heartbeat_pack(1, compid, &msg, system_type, autopilot_type);
  // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  /* The default UART header for your MCU */
  // Send the message (.write sends as bytes)
  delay(1000);
  Serial1.write(buf, len);
  receve_msg();
    memset(buf, 0, MAVLINK_MAX_PACKET_LEN);
}

    void receve_msg()
  {
    uint8_t recsize = Serial1.available();  //receive data over serial
    while (Serial1.available() > 0)
    {
      mavlink_message_t msg;
      mavlink_status_t status;
      uint8_t rec =  Serial1.read();//Show bytes send from the pixhawk
      //Serial.println(rec);
      //try to get a new message
      if (mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status ) == false )
      {
        Serial.println("Received Packet :");
        Serial.println("Sys: ");
        Serial.println(msg.sysid);
        Serial.println("Comp: ");
        Serial.println(msg.compid);
        Serial.println("Len: ");
        Serial.println(msg.len);
        Serial.println("Msg Id: ");
        Serial.println(msg.msgid);
      }

      Serial.println(" ");
      // And get the next one

    }
  }





