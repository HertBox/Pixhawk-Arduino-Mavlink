/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	 Generated on Sunday, October 31 2010, 11:49 UTC
 */
#ifndef PIXHAWK_H
#define PIXHAWK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "../protocol.h"

#define MAVLINK_ENABLED_PIXHAWK


#include "../common/common.h"
// ENUM DEFINITIONS

/** @brief Slugs parameter interface subsets */
enum SLUGS_PID_INDX_IDS
{
	PID_YAW_DAMPER=2,
	PID_PITCH=3, /* With comment: PID Pitch parameter*/
	PID_ALT_HOLD=50
};


// MESSAGE DEFINITIONS

#include "./mavlink_msg_attitude_control.h"
#include "./mavlink_msg_debug_vect.h"
#include "./mavlink_msg_set_cam_shutter.h"
#include "./mavlink_msg_image_triggered.h"
#include "./mavlink_msg_image_trigger_control.h"
#include "./mavlink_msg_image_available.h"
#include "./mavlink_msg_vision_position_estimate.h"
#include "./mavlink_msg_vicon_position_estimate.h"
#include "./mavlink_msg_position_control_setpoint_set.h"
#include "./mavlink_msg_position_control_offset_set.h"
#include "./mavlink_msg_position_control_setpoint.h"
#include "./mavlink_msg_marker.h"
#include "./mavlink_msg_raw_aux.h"
#include "./mavlink_msg_aux_status.h"
#include "./mavlink_msg_control_status.h"
#include "./mavlink_msg_watchdog_heartbeat.h"
#include "./mavlink_msg_watchdog_process_info.h"
#include "./mavlink_msg_watchdog_process_status.h"
#include "./mavlink_msg_watchdog_command.h"
#include "./mavlink_msg_pattern_detected.h"
#include "./mavlink_msg_point_of_interest.h"
#include "./mavlink_msg_point_of_interest_connection.h"
#ifdef __cplusplus
}
#endif
#endif
