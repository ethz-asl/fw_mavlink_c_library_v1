#pragma once
// MESSAGE SENS_4H_VECTOR_PROBE PACKING

#define MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE 214

MAVPACKED(
typedef struct __mavlink_sens_4h_vector_probe_t {
 uint64_t timestamp; /*< Timestamp*/
 float raw_pressure1; /*< Raw pressure on port 1*/
 float raw_pressure2; /*< Raw pressure on port 2*/
 float raw_pressure3; /*< Raw pressure on port 3*/
 float raw_pressure4; /*< Raw pressure on port 4*/
 float filtered_pressure1; /*< filtered pressure on port 1*/
 float filtered_pressure2; /*< filtered pressure on port 2*/
 float filtered_pressure3; /*< filtered pressure on port 3*/
 float filtered_pressure4; /*< filtered pressure on port 4*/
 float alpha; /*< Angle of attack*/
 float beta; /*< Sideslip angle*/
 float u; /*< airspeed*/
 float pt; /*< total pressure*/
}) mavlink_sens_4h_vector_probe_t;

#define MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN 56
#define MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN 56
#define MAVLINK_MSG_ID_214_LEN 56
#define MAVLINK_MSG_ID_214_MIN_LEN 56

#define MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC 87
#define MAVLINK_MSG_ID_214_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_4H_VECTOR_PROBE { \
    214, \
    "SENS_4H_VECTOR_PROBE", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_4h_vector_probe_t, timestamp) }, \
         { "raw_pressure1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure1) }, \
         { "raw_pressure2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure2) }, \
         { "raw_pressure3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure3) }, \
         { "raw_pressure4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure4) }, \
         { "filtered_pressure1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure1) }, \
         { "filtered_pressure2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure2) }, \
         { "filtered_pressure3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure3) }, \
         { "filtered_pressure4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure4) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sens_4h_vector_probe_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sens_4h_vector_probe_t, beta) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sens_4h_vector_probe_t, u) }, \
         { "pt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sens_4h_vector_probe_t, pt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_4H_VECTOR_PROBE { \
    "SENS_4H_VECTOR_PROBE", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_4h_vector_probe_t, timestamp) }, \
         { "raw_pressure1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure1) }, \
         { "raw_pressure2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure2) }, \
         { "raw_pressure3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure3) }, \
         { "raw_pressure4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_4h_vector_probe_t, raw_pressure4) }, \
         { "filtered_pressure1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure1) }, \
         { "filtered_pressure2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure2) }, \
         { "filtered_pressure3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure3) }, \
         { "filtered_pressure4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sens_4h_vector_probe_t, filtered_pressure4) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sens_4h_vector_probe_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sens_4h_vector_probe_t, beta) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sens_4h_vector_probe_t, u) }, \
         { "pt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_sens_4h_vector_probe_t, pt) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_4h_vector_probe message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param raw_pressure1 Raw pressure on port 1
 * @param raw_pressure2 Raw pressure on port 2
 * @param raw_pressure3 Raw pressure on port 3
 * @param raw_pressure4 Raw pressure on port 4
 * @param filtered_pressure1 filtered pressure on port 1
 * @param filtered_pressure2 filtered pressure on port 2
 * @param filtered_pressure3 filtered pressure on port 3
 * @param filtered_pressure4 filtered pressure on port 4
 * @param alpha Angle of attack
 * @param beta Sideslip angle
 * @param u airspeed
 * @param pt total pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_4h_vector_probe_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float raw_pressure1, float raw_pressure2, float raw_pressure3, float raw_pressure4, float filtered_pressure1, float filtered_pressure2, float filtered_pressure3, float filtered_pressure4, float alpha, float beta, float u, float pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, raw_pressure1);
    _mav_put_float(buf, 12, raw_pressure2);
    _mav_put_float(buf, 16, raw_pressure3);
    _mav_put_float(buf, 20, raw_pressure4);
    _mav_put_float(buf, 24, filtered_pressure1);
    _mav_put_float(buf, 28, filtered_pressure2);
    _mav_put_float(buf, 32, filtered_pressure3);
    _mav_put_float(buf, 36, filtered_pressure4);
    _mav_put_float(buf, 40, alpha);
    _mav_put_float(buf, 44, beta);
    _mav_put_float(buf, 48, u);
    _mav_put_float(buf, 52, pt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN);
#else
    mavlink_sens_4h_vector_probe_t packet;
    packet.timestamp = timestamp;
    packet.raw_pressure1 = raw_pressure1;
    packet.raw_pressure2 = raw_pressure2;
    packet.raw_pressure3 = raw_pressure3;
    packet.raw_pressure4 = raw_pressure4;
    packet.filtered_pressure1 = filtered_pressure1;
    packet.filtered_pressure2 = filtered_pressure2;
    packet.filtered_pressure3 = filtered_pressure3;
    packet.filtered_pressure4 = filtered_pressure4;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.u = u;
    packet.pt = pt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
}

/**
 * @brief Pack a sens_4h_vector_probe message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param raw_pressure1 Raw pressure on port 1
 * @param raw_pressure2 Raw pressure on port 2
 * @param raw_pressure3 Raw pressure on port 3
 * @param raw_pressure4 Raw pressure on port 4
 * @param filtered_pressure1 filtered pressure on port 1
 * @param filtered_pressure2 filtered pressure on port 2
 * @param filtered_pressure3 filtered pressure on port 3
 * @param filtered_pressure4 filtered pressure on port 4
 * @param alpha Angle of attack
 * @param beta Sideslip angle
 * @param u airspeed
 * @param pt total pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_4h_vector_probe_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float raw_pressure1,float raw_pressure2,float raw_pressure3,float raw_pressure4,float filtered_pressure1,float filtered_pressure2,float filtered_pressure3,float filtered_pressure4,float alpha,float beta,float u,float pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, raw_pressure1);
    _mav_put_float(buf, 12, raw_pressure2);
    _mav_put_float(buf, 16, raw_pressure3);
    _mav_put_float(buf, 20, raw_pressure4);
    _mav_put_float(buf, 24, filtered_pressure1);
    _mav_put_float(buf, 28, filtered_pressure2);
    _mav_put_float(buf, 32, filtered_pressure3);
    _mav_put_float(buf, 36, filtered_pressure4);
    _mav_put_float(buf, 40, alpha);
    _mav_put_float(buf, 44, beta);
    _mav_put_float(buf, 48, u);
    _mav_put_float(buf, 52, pt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN);
#else
    mavlink_sens_4h_vector_probe_t packet;
    packet.timestamp = timestamp;
    packet.raw_pressure1 = raw_pressure1;
    packet.raw_pressure2 = raw_pressure2;
    packet.raw_pressure3 = raw_pressure3;
    packet.raw_pressure4 = raw_pressure4;
    packet.filtered_pressure1 = filtered_pressure1;
    packet.filtered_pressure2 = filtered_pressure2;
    packet.filtered_pressure3 = filtered_pressure3;
    packet.filtered_pressure4 = filtered_pressure4;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.u = u;
    packet.pt = pt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
}

/**
 * @brief Encode a sens_4h_vector_probe struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_4h_vector_probe C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_4h_vector_probe_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_4h_vector_probe_t* sens_4h_vector_probe)
{
    return mavlink_msg_sens_4h_vector_probe_pack(system_id, component_id, msg, sens_4h_vector_probe->timestamp, sens_4h_vector_probe->raw_pressure1, sens_4h_vector_probe->raw_pressure2, sens_4h_vector_probe->raw_pressure3, sens_4h_vector_probe->raw_pressure4, sens_4h_vector_probe->filtered_pressure1, sens_4h_vector_probe->filtered_pressure2, sens_4h_vector_probe->filtered_pressure3, sens_4h_vector_probe->filtered_pressure4, sens_4h_vector_probe->alpha, sens_4h_vector_probe->beta, sens_4h_vector_probe->u, sens_4h_vector_probe->pt);
}

/**
 * @brief Encode a sens_4h_vector_probe struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_4h_vector_probe C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_4h_vector_probe_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_4h_vector_probe_t* sens_4h_vector_probe)
{
    return mavlink_msg_sens_4h_vector_probe_pack_chan(system_id, component_id, chan, msg, sens_4h_vector_probe->timestamp, sens_4h_vector_probe->raw_pressure1, sens_4h_vector_probe->raw_pressure2, sens_4h_vector_probe->raw_pressure3, sens_4h_vector_probe->raw_pressure4, sens_4h_vector_probe->filtered_pressure1, sens_4h_vector_probe->filtered_pressure2, sens_4h_vector_probe->filtered_pressure3, sens_4h_vector_probe->filtered_pressure4, sens_4h_vector_probe->alpha, sens_4h_vector_probe->beta, sens_4h_vector_probe->u, sens_4h_vector_probe->pt);
}

/**
 * @brief Send a sens_4h_vector_probe message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param raw_pressure1 Raw pressure on port 1
 * @param raw_pressure2 Raw pressure on port 2
 * @param raw_pressure3 Raw pressure on port 3
 * @param raw_pressure4 Raw pressure on port 4
 * @param filtered_pressure1 filtered pressure on port 1
 * @param filtered_pressure2 filtered pressure on port 2
 * @param filtered_pressure3 filtered pressure on port 3
 * @param filtered_pressure4 filtered pressure on port 4
 * @param alpha Angle of attack
 * @param beta Sideslip angle
 * @param u airspeed
 * @param pt total pressure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_4h_vector_probe_send(mavlink_channel_t chan, uint64_t timestamp, float raw_pressure1, float raw_pressure2, float raw_pressure3, float raw_pressure4, float filtered_pressure1, float filtered_pressure2, float filtered_pressure3, float filtered_pressure4, float alpha, float beta, float u, float pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, raw_pressure1);
    _mav_put_float(buf, 12, raw_pressure2);
    _mav_put_float(buf, 16, raw_pressure3);
    _mav_put_float(buf, 20, raw_pressure4);
    _mav_put_float(buf, 24, filtered_pressure1);
    _mav_put_float(buf, 28, filtered_pressure2);
    _mav_put_float(buf, 32, filtered_pressure3);
    _mav_put_float(buf, 36, filtered_pressure4);
    _mav_put_float(buf, 40, alpha);
    _mav_put_float(buf, 44, beta);
    _mav_put_float(buf, 48, u);
    _mav_put_float(buf, 52, pt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE, buf, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
#else
    mavlink_sens_4h_vector_probe_t packet;
    packet.timestamp = timestamp;
    packet.raw_pressure1 = raw_pressure1;
    packet.raw_pressure2 = raw_pressure2;
    packet.raw_pressure3 = raw_pressure3;
    packet.raw_pressure4 = raw_pressure4;
    packet.filtered_pressure1 = filtered_pressure1;
    packet.filtered_pressure2 = filtered_pressure2;
    packet.filtered_pressure3 = filtered_pressure3;
    packet.filtered_pressure4 = filtered_pressure4;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.u = u;
    packet.pt = pt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE, (const char *)&packet, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
#endif
}

/**
 * @brief Send a sens_4h_vector_probe message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_4h_vector_probe_send_struct(mavlink_channel_t chan, const mavlink_sens_4h_vector_probe_t* sens_4h_vector_probe)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_4h_vector_probe_send(chan, sens_4h_vector_probe->timestamp, sens_4h_vector_probe->raw_pressure1, sens_4h_vector_probe->raw_pressure2, sens_4h_vector_probe->raw_pressure3, sens_4h_vector_probe->raw_pressure4, sens_4h_vector_probe->filtered_pressure1, sens_4h_vector_probe->filtered_pressure2, sens_4h_vector_probe->filtered_pressure3, sens_4h_vector_probe->filtered_pressure4, sens_4h_vector_probe->alpha, sens_4h_vector_probe->beta, sens_4h_vector_probe->u, sens_4h_vector_probe->pt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE, (const char *)sens_4h_vector_probe, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_4h_vector_probe_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float raw_pressure1, float raw_pressure2, float raw_pressure3, float raw_pressure4, float filtered_pressure1, float filtered_pressure2, float filtered_pressure3, float filtered_pressure4, float alpha, float beta, float u, float pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, raw_pressure1);
    _mav_put_float(buf, 12, raw_pressure2);
    _mav_put_float(buf, 16, raw_pressure3);
    _mav_put_float(buf, 20, raw_pressure4);
    _mav_put_float(buf, 24, filtered_pressure1);
    _mav_put_float(buf, 28, filtered_pressure2);
    _mav_put_float(buf, 32, filtered_pressure3);
    _mav_put_float(buf, 36, filtered_pressure4);
    _mav_put_float(buf, 40, alpha);
    _mav_put_float(buf, 44, beta);
    _mav_put_float(buf, 48, u);
    _mav_put_float(buf, 52, pt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE, buf, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
#else
    mavlink_sens_4h_vector_probe_t *packet = (mavlink_sens_4h_vector_probe_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->raw_pressure1 = raw_pressure1;
    packet->raw_pressure2 = raw_pressure2;
    packet->raw_pressure3 = raw_pressure3;
    packet->raw_pressure4 = raw_pressure4;
    packet->filtered_pressure1 = filtered_pressure1;
    packet->filtered_pressure2 = filtered_pressure2;
    packet->filtered_pressure3 = filtered_pressure3;
    packet->filtered_pressure4 = filtered_pressure4;
    packet->alpha = alpha;
    packet->beta = beta;
    packet->u = u;
    packet->pt = pt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE, (const char *)packet, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_MIN_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_4H_VECTOR_PROBE UNPACKING


/**
 * @brief Get field timestamp from sens_4h_vector_probe message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_sens_4h_vector_probe_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field raw_pressure1 from sens_4h_vector_probe message
 *
 * @return Raw pressure on port 1
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_raw_pressure1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field raw_pressure2 from sens_4h_vector_probe message
 *
 * @return Raw pressure on port 2
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_raw_pressure2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field raw_pressure3 from sens_4h_vector_probe message
 *
 * @return Raw pressure on port 3
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_raw_pressure3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field raw_pressure4 from sens_4h_vector_probe message
 *
 * @return Raw pressure on port 4
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_raw_pressure4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field filtered_pressure1 from sens_4h_vector_probe message
 *
 * @return filtered pressure on port 1
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_filtered_pressure1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field filtered_pressure2 from sens_4h_vector_probe message
 *
 * @return filtered pressure on port 2
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_filtered_pressure2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field filtered_pressure3 from sens_4h_vector_probe message
 *
 * @return filtered pressure on port 3
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_filtered_pressure3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field filtered_pressure4 from sens_4h_vector_probe message
 *
 * @return filtered pressure on port 4
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_filtered_pressure4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field alpha from sens_4h_vector_probe message
 *
 * @return Angle of attack
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_alpha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field beta from sens_4h_vector_probe message
 *
 * @return Sideslip angle
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_beta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field u from sens_4h_vector_probe message
 *
 * @return airspeed
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_u(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pt from sens_4h_vector_probe message
 *
 * @return total pressure
 */
static inline float mavlink_msg_sens_4h_vector_probe_get_pt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a sens_4h_vector_probe message into a struct
 *
 * @param msg The message to decode
 * @param sens_4h_vector_probe C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_4h_vector_probe_decode(const mavlink_message_t* msg, mavlink_sens_4h_vector_probe_t* sens_4h_vector_probe)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_4h_vector_probe->timestamp = mavlink_msg_sens_4h_vector_probe_get_timestamp(msg);
    sens_4h_vector_probe->raw_pressure1 = mavlink_msg_sens_4h_vector_probe_get_raw_pressure1(msg);
    sens_4h_vector_probe->raw_pressure2 = mavlink_msg_sens_4h_vector_probe_get_raw_pressure2(msg);
    sens_4h_vector_probe->raw_pressure3 = mavlink_msg_sens_4h_vector_probe_get_raw_pressure3(msg);
    sens_4h_vector_probe->raw_pressure4 = mavlink_msg_sens_4h_vector_probe_get_raw_pressure4(msg);
    sens_4h_vector_probe->filtered_pressure1 = mavlink_msg_sens_4h_vector_probe_get_filtered_pressure1(msg);
    sens_4h_vector_probe->filtered_pressure2 = mavlink_msg_sens_4h_vector_probe_get_filtered_pressure2(msg);
    sens_4h_vector_probe->filtered_pressure3 = mavlink_msg_sens_4h_vector_probe_get_filtered_pressure3(msg);
    sens_4h_vector_probe->filtered_pressure4 = mavlink_msg_sens_4h_vector_probe_get_filtered_pressure4(msg);
    sens_4h_vector_probe->alpha = mavlink_msg_sens_4h_vector_probe_get_alpha(msg);
    sens_4h_vector_probe->beta = mavlink_msg_sens_4h_vector_probe_get_beta(msg);
    sens_4h_vector_probe->u = mavlink_msg_sens_4h_vector_probe_get_u(msg);
    sens_4h_vector_probe->pt = mavlink_msg_sens_4h_vector_probe_get_pt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN? msg->len : MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN;
        memset(sens_4h_vector_probe, 0, MAVLINK_MSG_ID_SENS_4H_VECTOR_PROBE_LEN);
    memcpy(sens_4h_vector_probe, _MAV_PAYLOAD(msg), len);
#endif
}
