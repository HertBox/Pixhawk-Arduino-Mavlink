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
	int system_type = MAV_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Pack the message
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	delay(1000);
	Serial1.write(buf, len);
	comm_receive();
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	while(Serial1.available() > 0) {
     
		uint8_t c = Serial1.read();
   Serial.println(c);
			//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			Serial.println("Enter"); 
			// Handle message
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_SET_MODE: {
			       Serial.println("set mode");
			        	// set mode
             }
			        break;
			        case MAVLINK_MSG_ID_ACTION:
           		Serial.println("ID action");
           				// EXECUTE ACTION
				break;
				default:
					//Do nothing
				break;
			}
		} 
		// And get the next one
	}
}


