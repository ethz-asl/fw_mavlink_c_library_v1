// MESSAGE WIND_SENSOR PACKING

#define MAVLINK_MSG_ID_WIND_SENSOR 213

MAVPACKED(
typedef struct __mavlink_wind_sensor_t {
 float direction; /*<  Wind Direction*/
 float avg_mag; /*<  Average wind measured*/
 float max_mag; /*<  Maximum wind measured*/
 float std_dev; /*<  Standard Deviation of measured wind magnitude*/
}) mavlink_wind_sensor_t;

#define MAVLINK_MSG_ID_WIND_SENSOR_LEN 16
#define MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN 16
#define MAVLINK_MSG_ID_213_LEN 16
#define MAVLINK_MSG_ID_213_MIN_LEN 16

#define MAVLINK_MSG_ID_WIND_SENSOR_CRC 173
#define MAVLINK_MSG_ID_213_CRC 173



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WIND_SENSOR { \
	213, \
	"WIND_SENSOR", \
	4, \
	{  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_sensor_t, direction) }, \
         { "avg_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_sensor_t, avg_mag) }, \
         { "max_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_sensor_t, max_mag) }, \
         { "std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wind_sensor_t, std_dev) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WIND_SENSOR { \
	"WIND_SENSOR", \
	4, \
	{  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_sensor_t, direction) }, \
         { "avg_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_sensor_t, avg_mag) }, \
         { "max_mag", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_sensor_t, max_mag) }, \
         { "std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wind_sensor_t, std_dev) }, \
         } \
}
#endif

/**
 * @brief Pack a wind_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param direction  Wind Direction
 * @param avg_mag  Average wind measured
 * @param max_mag  Maximum wind measured
 * @param std_dev  Standard Deviation of measured wind magnitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float direction, float avg_mag, float max_mag, float std_dev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_SENSOR_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, avg_mag);
	_mav_put_float(buf, 8, max_mag);
	_mav_put_float(buf, 12, std_dev);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_SENSOR_LEN);
#else
	mavlink_wind_sensor_t packet;
	packet.direction = direction;
	packet.avg_mag = avg_mag;
	packet.max_mag = max_mag;
	packet.std_dev = std_dev;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
}

/**
 * @brief Pack a wind_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param direction  Wind Direction
 * @param avg_mag  Average wind measured
 * @param max_mag  Maximum wind measured
 * @param std_dev  Standard Deviation of measured wind magnitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float direction,float avg_mag,float max_mag,float std_dev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_SENSOR_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, avg_mag);
	_mav_put_float(buf, 8, max_mag);
	_mav_put_float(buf, 12, std_dev);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_SENSOR_LEN);
#else
	mavlink_wind_sensor_t packet;
	packet.direction = direction;
	packet.avg_mag = avg_mag;
	packet.max_mag = max_mag;
	packet.std_dev = std_dev;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
}

/**
 * @brief Encode a wind_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wind_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_sensor_t* wind_sensor)
{
	return mavlink_msg_wind_sensor_pack(system_id, component_id, msg, wind_sensor->direction, wind_sensor->avg_mag, wind_sensor->max_mag, wind_sensor->std_dev);
}

/**
 * @brief Encode a wind_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wind_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wind_sensor_t* wind_sensor)
{
	return mavlink_msg_wind_sensor_pack_chan(system_id, component_id, chan, msg, wind_sensor->direction, wind_sensor->avg_mag, wind_sensor->max_mag, wind_sensor->std_dev);
}

/**
 * @brief Send a wind_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param direction  Wind Direction
 * @param avg_mag  Average wind measured
 * @param max_mag  Maximum wind measured
 * @param std_dev  Standard Deviation of measured wind magnitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wind_sensor_send(mavlink_channel_t chan, float direction, float avg_mag, float max_mag, float std_dev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_SENSOR_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, avg_mag);
	_mav_put_float(buf, 8, max_mag);
	_mav_put_float(buf, 12, std_dev);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_SENSOR, buf, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
#else
	mavlink_wind_sensor_t packet;
	packet.direction = direction;
	packet.avg_mag = avg_mag;
	packet.max_mag = max_mag;
	packet.std_dev = std_dev;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
#endif
}

/**
 * @brief Send a wind_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wind_sensor_send_struct(mavlink_channel_t chan, const mavlink_wind_sensor_t* wind_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wind_sensor_send(chan, wind_sensor->direction, wind_sensor->avg_mag, wind_sensor->max_mag, wind_sensor->std_dev);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_SENSOR, (const char *)wind_sensor, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_WIND_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wind_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float direction, float avg_mag, float max_mag, float std_dev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, avg_mag);
	_mav_put_float(buf, 8, max_mag);
	_mav_put_float(buf, 12, std_dev);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_SENSOR, buf, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
#else
	mavlink_wind_sensor_t *packet = (mavlink_wind_sensor_t *)msgbuf;
	packet->direction = direction;
	packet->avg_mag = avg_mag;
	packet->max_mag = max_mag;
	packet->std_dev = std_dev;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_SENSOR, (const char *)packet, MAVLINK_MSG_ID_WIND_SENSOR_MIN_LEN, MAVLINK_MSG_ID_WIND_SENSOR_LEN, MAVLINK_MSG_ID_WIND_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE WIND_SENSOR UNPACKING


/**
 * @brief Get field direction from wind_sensor message
 *
 * @return  Wind Direction
 */
static inline float mavlink_msg_wind_sensor_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field avg_mag from wind_sensor message
 *
 * @return  Average wind measured
 */
static inline float mavlink_msg_wind_sensor_get_avg_mag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field max_mag from wind_sensor message
 *
 * @return  Maximum wind measured
 */
static inline float mavlink_msg_wind_sensor_get_max_mag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field std_dev from wind_sensor message
 *
 * @return  Standard Deviation of measured wind magnitude
 */
static inline float mavlink_msg_wind_sensor_get_std_dev(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a wind_sensor message into a struct
 *
 * @param msg The message to decode
 * @param wind_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_wind_sensor_decode(const mavlink_message_t* msg, mavlink_wind_sensor_t* wind_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	wind_sensor->direction = mavlink_msg_wind_sensor_get_direction(msg);
	wind_sensor->avg_mag = mavlink_msg_wind_sensor_get_avg_mag(msg);
	wind_sensor->max_mag = mavlink_msg_wind_sensor_get_max_mag(msg);
	wind_sensor->std_dev = mavlink_msg_wind_sensor_get_std_dev(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WIND_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_WIND_SENSOR_LEN;
        memset(wind_sensor, 0, MAVLINK_MSG_ID_WIND_SENSOR_LEN);
	memcpy(wind_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
