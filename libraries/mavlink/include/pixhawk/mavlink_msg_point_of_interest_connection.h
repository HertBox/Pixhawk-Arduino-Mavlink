// MESSAGE POINT_OF_INTEREST_CONNECTION PACKING

#define MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION 162

typedef struct __mavlink_point_of_interest_connection_t 
{
	uint8_t type; ///< 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	uint8_t color; ///< 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	uint8_t coordinate_system; ///< 0: global, 1:local
	uint16_t timeout; ///< 0: no timeout, >1: timeout in seconds
	float x1; ///< X1 Position
	float y1; ///< Y1 Position
	float z1; ///< Z1 Position
	float x2; ///< X2 Position
	float y2; ///< Y2 Position
	float z2; ///< Z2 Position
	int8_t name[25]; ///< POI connection name

} mavlink_point_of_interest_connection_t;

#define MAVLINK_MSG_POINT_OF_INTEREST_CONNECTION_FIELD_NAME_LEN 25


/**
 * @brief Send a point_of_interest_connection message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x1 X1 Position
 * @param y1 Y1 Position
 * @param z1 Z1 Position
 * @param x2 X2 Position
 * @param y2 Y2 Position
 * @param z2 Z2 Position
 * @param name POI connection name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x1, float y1, float z1, float x2, float y2, float z2, const int8_t* name)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION;

	i += put_uint8_t_by_index(type, i, msg->payload); //0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	i += put_uint8_t_by_index(color, i, msg->payload); //0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	i += put_uint8_t_by_index(coordinate_system, i, msg->payload); //0: global, 1:local
	i += put_uint16_t_by_index(timeout, i, msg->payload); //0: no timeout, >1: timeout in seconds
	i += put_float_by_index(x1, i, msg->payload); //X1 Position
	i += put_float_by_index(y1, i, msg->payload); //Y1 Position
	i += put_float_by_index(z1, i, msg->payload); //Z1 Position
	i += put_float_by_index(x2, i, msg->payload); //X2 Position
	i += put_float_by_index(y2, i, msg->payload); //Y2 Position
	i += put_float_by_index(z2, i, msg->payload); //Z2 Position
	i += put_array_by_index(name, 25, i, msg->payload); //POI connection name

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_point_of_interest_connection_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_connection_t* point_of_interest_connection)
{
	return mavlink_msg_point_of_interest_connection_pack(system_id, component_id, msg, point_of_interest_connection->type, point_of_interest_connection->color, point_of_interest_connection->coordinate_system, point_of_interest_connection->timeout, point_of_interest_connection->x1, point_of_interest_connection->y1, point_of_interest_connection->z1, point_of_interest_connection->x2, point_of_interest_connection->y2, point_of_interest_connection->z2, point_of_interest_connection->name);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_connection_send(mavlink_channel_t chan, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x1, float y1, float z1, float x2, float y2, float z2, const int8_t* name)
{
	mavlink_message_t msg;
	mavlink_msg_point_of_interest_connection_pack(mavlink_system.sysid, mavlink_system.compid, &msg, type, color, coordinate_system, timeout, x1, y1, z1, x2, y2, z2, name);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE POINT_OF_INTEREST_CONNECTION UNPACKING

/**
 * @brief Get field type from point_of_interest_connection message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_type(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field color from point_of_interest_connection message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_color(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field coordinate_system from point_of_interest_connection message
 *
 * @return 0: global, 1:local
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_coordinate_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field timeout from point_of_interest_connection message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_get_timeout(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field x1 from point_of_interest_connection message
 *
 * @return X1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_x1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y1 from point_of_interest_connection message
 *
 * @return Y1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_y1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z1 from point_of_interest_connection message
 *
 * @return Z1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_z1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field x2 from point_of_interest_connection message
 *
 * @return X2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_x2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field y2 from point_of_interest_connection message
 *
 * @return Y2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_y2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z2 from point_of_interest_connection message
 *
 * @return Z2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_z2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field name from point_of_interest_connection message
 *
 * @return POI connection name
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_get_name(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float), 25);
	return 25;
}

static inline void mavlink_msg_point_of_interest_connection_decode(const mavlink_message_t* msg, mavlink_point_of_interest_connection_t* point_of_interest_connection)
{
	point_of_interest_connection->type = mavlink_msg_point_of_interest_connection_get_type(msg);
	point_of_interest_connection->color = mavlink_msg_point_of_interest_connection_get_color(msg);
	point_of_interest_connection->coordinate_system = mavlink_msg_point_of_interest_connection_get_coordinate_system(msg);
	point_of_interest_connection->timeout = mavlink_msg_point_of_interest_connection_get_timeout(msg);
	point_of_interest_connection->x1 = mavlink_msg_point_of_interest_connection_get_x1(msg);
	point_of_interest_connection->y1 = mavlink_msg_point_of_interest_connection_get_y1(msg);
	point_of_interest_connection->z1 = mavlink_msg_point_of_interest_connection_get_z1(msg);
	point_of_interest_connection->x2 = mavlink_msg_point_of_interest_connection_get_x2(msg);
	point_of_interest_connection->y2 = mavlink_msg_point_of_interest_connection_get_y2(msg);
	point_of_interest_connection->z2 = mavlink_msg_point_of_interest_connection_get_z2(msg);
	mavlink_msg_point_of_interest_connection_get_name(msg, point_of_interest_connection->name);
}
