#include <checksum.h>
#include <mavlink_types.h>
#include <mavlink.h>
#include <protocol.h>
#include <math.h>

//#include "C:/Users/luis.Valtierra/Documents/ardupilot/libraries/mavlink/include/mavlink.h"         // Mavlink interface
//#include "C:/Users/luis.Valtierra/Documents/ardupilot/libraries/mavlink/mavlink/v1.0/mavlink_types.h"

uint8_t system_id = 1;
uint8_t component_id = 1;
uint8_t type = 6; //GCS
uint8_t autopilot = 0; //generic




uint8_t received_sysid = 1;
uint8_t received_compid = 1;
uint8_t GCS_UNITS = 0;
int a = 0;
void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
  // Define the system type (see mavlink_types.h for list of possible types)
  receve_msg();
  if (a == 0)
  {
    present_msg();
    a = 1;
  }
  else {
    a = 1;
    start_feeds();
  }
}
void present_msg() {
  mavlink_message_t msg;
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg,  type, autopilot, 0, 0, 4);
  send_message(&msg);
  delay(10);
}


void start_feeds() //request data
{
  mavlink_message_t msg;
  mavlink_msg_request_data_stream_pack(system_id, component_id, &msg,
                                       received_sysid, received_compid, 30, 20, 1);
  send_message(&msg);
  delay(10);

}

void send_message(mavlink_message_t* msg) //send data bit by bit
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  delay(1000);
  Serial1.write(buf, len);// Send the message (.write sends as bytes)
}


//void stop_feeds() //stop request
//{
//  mavlink_message_t msg1;
//
//  send_message(&msg1);
//  delay(500);
//}

void receve_msg()
{ //receive data over serial
  while (Serial1.available() > 0)
  {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t rec =  Serial1.read();//Show bytes send from the pixhawk
    Serial.println(rec);
    mavlink_parse_char(Serial1, rec, &msg, &status);

    gcs_handleMessage(&msg);


  }
  Serial.println("sali");

}

void gcs_handleMessage(mavlink_message_t* msg) //read varaible
{
  //  Serial.println();
  //  Serial.print("Message ID: ");
  //  Serial.println(msg->msgid);
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      {
        mavlink_heartbeat_t packet;
        mavlink_msg_heartbeat_decode(msg, &packet);
        uint8_t beat = 1;
        if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
          uint8_t received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
          uint8_t received_compid = (*msg).compid;
          uint8_t bmode = packet.base_mode;
          uint8_t cmode = packet.custom_mode;
          Serial.println("Im in heartbeat");

        }
        break;
      }
    case MAVLINK_MSG_ID_ATTITUDE:
      {
        // decode
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(msg, &packet);
        uint8_t pitch = packet.pitch;
        uint8_t yaw = packet.yaw;
        uint8_t roll = packet.roll;
        Serial.println("Im im  Attitude");
        break;
      }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        // decode
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(msg, &packet);
        uint8_t  gpsfix = packet.fix_type;
        uint8_t  mav_utime = packet.time_usec;
        uint8_t  numSats = packet.satellites_visible;
        uint8_t  cog = packet.cog;
        break;
      }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        uint8_t  latitude = packet.lat;
        uint8_t  longitude = packet.lon;
        if (GCS_UNITS == 0)uint8_t altitude = packet.alt / 1000;
        else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) ) uint8_t altitude = (packet.alt / 1000) * 3.28084;
        break;
      }
    case MAVLINK_MSG_ID_GPS_STATUS:
      {
        mavlink_gps_status_t packet;
        mavlink_msg_gps_status_decode(msg, &packet);
        break;
      }
    case MAVLINK_MSG_ID_VFR_HUD:
      {
        mavlink_vfr_hud_t packet;
        mavlink_msg_vfr_hud_decode(msg, &packet);
        uint8_t  heading = packet.heading;
        if (GCS_UNITS == 0) uint8_t ias = packet.airspeed * 3.6;
        else if (GCS_UNITS == 1)uint8_t ias = packet.airspeed * 2.24;
        else if (GCS_UNITS == 2)uint8_t ias = packet.airspeed * 1.94;
        if (GCS_UNITS == 0)uint8_t grs = packet.groundspeed * 3.6;
        else if (GCS_UNITS == 1)uint8_t grs = packet.groundspeed * 2.24;
        else if (GCS_UNITS == 2)uint8_t grs = packet.groundspeed * 1.94;
        if (GCS_UNITS == 0)uint8_t vsi = packet.climb;
        else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) )uint8_t vsi = packet.climb * 3.28084;
        break;
      }
    case MAVLINK_MSG_ID_RAW_PRESSURE:
      {
        // decode
        mavlink_raw_pressure_t packet;
        mavlink_msg_raw_pressure_decode(msg, &packet);
        break;
      }
    case MAVLINK_MSG_ID_SYS_STATUS:
      {

        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(msg, &packet);
        uint8_t  vbat = packet.voltage_battery;
        break;
      }
  }

}



//void heartbeat_msg()//MAVLINK_MSG_ID_HEARTBEAT 0
//{
//  uint8_t system_type = MAV_TYPE_QUADROTOR;
//  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
//  uint8_t base_mode =MAV_MODE_FLAG_ENUM_END;
//  uint8_t custom_mode = 0 ;
//  uint8_t system_status = MAV_STATE_STANDBY ;
//
//  mavlink_message_t msg;// Initialize the required buffers
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
//  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type, base_mode, custom_mode, system_status);// Pack the message
//  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);// Copy the message to send buffer
//  delay(1000);
//  Serial1.write(buf, len);// Send the message (.write sends as bytes)
//}




