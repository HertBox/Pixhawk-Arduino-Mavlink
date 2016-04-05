#include "C:/Users/luis.Valtierra/Documents/ardupilot/libraries/mavlink/include/mavlink.h"


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
  // put your main code here, to run repeatedly:
  receve_msg();
  start_feeds();


}

void start_feeds() //request data
{
  mavlink_message_t msg;
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_SENSORS_RATE, MAV_DATA_STREAM_RAW_SENSORS_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA1_RATE, MAV_DATA_STREAM_EXTRA1_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTRA2_RATE, MAV_DATA_STREAM_EXTRA2_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTENDED_STATUS_RATE, MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_CONTROLLER, MAV_DATA_STREAM_RAW_CONTROLLER_RATE, MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, MAV_DATA_STREAM_POSITION_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RC_CHANNELS, 0, 0);
  send_message(&msg);
}

void send_message(mavlink_message_t* msg) //send data bit by bit
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  for (uint16_t i = 0; i < len; i++)
  {
    Serial1.write(buf[i]);
  }
}

void stop_feeds() //stop request
{
  mavlink_message_t msg1;
  mavlink_msg_request_data_stream_pack(127, 0, &msg1, received_sysid, received_compid, MAV_DATA_STREAM_ALL, 0, 0);
  send_message(&msg1);
  delay(500);
}


void receve_msg()
{ //receive data over serial
  while (Serial1.available() > 0)
  {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t rec =  Serial1.read();//Show bytes send from the pixhawk
    mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status);
    gcs_handleMessage(&msg);

  }
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
        }
        break;
      }
    case MAVLINK_MSG_ID_ATTITUDE:
      {
        // decode
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(msg, &packet);
        pitch = toDeg(packet.pitch);
        yaw = toDeg(packet.yaw);
        roll = toDeg(packet.roll);
        break;
      }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        // decode
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(msg, &packet);
        gpsfix = packet.fix_type;
        mav_utime = packet.time_usec;
        numSats = packet.satellites_visible;
        cog = packet.cog;
        break;
      }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        latitude = packet.lat;
        longitude = packet.lon;
        if (GCS_UNITS == 0) altitude = packet.alt / 1000;
        else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) ) altitude = (packet.alt / 1000) * 3.28084;
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
        heading = packet.heading;
        if (GCS_UNITS == 0) ias = packet.airspeed * 3.6;
        else if (GCS_UNITS == 1) ias = packet.airspeed * 2.24;
        else if (GCS_UNITS == 2) ias = packet.airspeed * 1.94;
        if (GCS_UNITS == 0) grs = packet.groundspeed * 3.6;
        else if (GCS_UNITS == 1) grs = packet.groundspeed * 2.24;
        else if (GCS_UNITS == 2) grs = packet.groundspeed * 1.94;
        if (GCS_UNITS == 0) vsi = packet.climb;
        else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) ) vsi = packet.climb * 3.28084;
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
        vbat = packet.voltage_battery;
        break;
      }
  }

}





