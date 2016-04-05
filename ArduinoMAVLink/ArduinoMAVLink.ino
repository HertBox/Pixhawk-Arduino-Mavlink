
#include "C:/Users/luis.Valtierra/Documents/ardupilot/libraries/mavlink/include/mavlink.h"         // Mavlink interface

uint8_t system_id = 255; 
uint8_t component_id = 200;

// Message #0  HEARTHBEAT 
uint8_t    ap_type = MAV_GENERIC;
uint8_t    ap_autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 1;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
 // Define the system type (see mavlink_types.h for list of possible types) 
 int system_type = MAV_QUADROTOR;
  int autopilot_type = MAV_AUTOPILOT_GENERIC;
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type);
  
  // Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  delay(1000);
  Serial1.write(buf, len);
  /* The default UART header for your MCU */
  // Send the message (.write sends as bytes)
  
  receve_msg();
}

void receve_msg()
{
  //receive data over serial
  
  while (Serial1.available() > 0)
  {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t rec =  Serial1.read();//Show bytes send from the pixhawk
    //try to get a new message
    mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status );
    
      Serial.println("Received Packet :");
      Serial.println("Sys: ");
      Serial.println(msg.sysid);
      Serial.println("Comp: ");
      Serial.println(msg.compid);
      Serial.println("Len: ");
      Serial.println(msg.len);
      Serial.println("Msg Id: ");
      Serial.println(msg.msgid);
      Serial.println("Seq: ");
      Serial.println(msg.seq);
      Serial.println("payload: ");
      Serial.println(msg.payload[MAVLINK_MAX_PAYLOAD_LEN]);
      Serial.println("cka: ");
      Serial.println(msg.ck_a);
      Serial.println("ckb: ");
      Serial.println(msg.ck_b);
      
    // And get the next one

  }
}





