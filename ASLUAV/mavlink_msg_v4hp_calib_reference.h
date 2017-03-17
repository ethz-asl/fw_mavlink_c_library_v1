#pragma once
// MESSAGE V4HP_CALIB_REFERENCE PACKING

#define MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE 215

MAVPACKED(
typedef struct __mavlink_v4hp_calib_reference_t {
 uint64_t timestamp; /*< Timestamp*/
 int16_t alpha; /*< alpha (degrees*10)*/
 int16_t beta; /*< beta (degrees*10)*/
}) mavlink_v4hp_calib_reference_t;

#define MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN 12
#define MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN 12
#define MAVLINK_MSG_ID_215_LEN 12
#define MAVLINK_MSG_ID_215_MIN_LEN 12

#define MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC 167
#define MAVLINK_MSG_ID_215_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_V4HP_CALIB_REFERENCE { \
    215, \
    "V4HP_CALIB_REFERENCE", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_v4hp_calib_reference_t, timestamp) }, \
         { "alpha", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_v4hp_calib_reference_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_v4hp_calib_reference_t, beta) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_V4HP_CALIB_REFERENCE { \
    "V4HP_CALIB_REFERENCE", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_v4hp_calib_reference_t, timestamp) }, \
         { "alpha", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_v4hp_calib_reference_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_v4hp_calib_reference_t, beta) }, \
         } \
}
#endif

/**
 * @brief Pack a v4hp_calib_reference message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param alpha alpha (degrees*10)
 * @param beta beta (degrees*10)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_v4hp_calib_reference_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, int16_t alpha, int16_t beta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int16_t(buf, 8, alpha);
    _mav_put_int16_t(buf, 10, beta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN);
#else
    mavlink_v4hp_calib_reference_t packet;
    packet.timestamp = timestamp;
    packet.alpha = alpha;
    packet.beta = beta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
}

/**
 * @brief Pack a v4hp_calib_reference message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param alpha alpha (degrees*10)
 * @param beta beta (degrees*10)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_v4hp_calib_reference_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,int16_t alpha,int16_t beta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int16_t(buf, 8, alpha);
    _mav_put_int16_t(buf, 10, beta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN);
#else
    mavlink_v4hp_calib_reference_t packet;
    packet.timestamp = timestamp;
    packet.alpha = alpha;
    packet.beta = beta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
}

/**
 * @brief Encode a v4hp_calib_reference struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param v4hp_calib_reference C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_v4hp_calib_reference_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_v4hp_calib_reference_t* v4hp_calib_reference)
{
    return mavlink_msg_v4hp_calib_reference_pack(system_id, component_id, msg, v4hp_calib_reference->timestamp, v4hp_calib_reference->alpha, v4hp_calib_reference->beta);
}

/**
 * @brief Encode a v4hp_calib_reference struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v4hp_calib_reference C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_v4hp_calib_reference_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_v4hp_calib_reference_t* v4hp_calib_reference)
{
    return mavlink_msg_v4hp_calib_reference_pack_chan(system_id, component_id, chan, msg, v4hp_calib_reference->timestamp, v4hp_calib_reference->alpha, v4hp_calib_reference->beta);
}

/**
 * @brief Send a v4hp_calib_reference message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param alpha alpha (degrees*10)
 * @param beta beta (degrees*10)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_v4hp_calib_reference_send(mavlink_channel_t chan, uint64_t timestamp, int16_t alpha, int16_t beta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int16_t(buf, 8, alpha);
    _mav_put_int16_t(buf, 10, beta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE, buf, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
#else
    mavlink_v4hp_calib_reference_t packet;
    packet.timestamp = timestamp;
    packet.alpha = alpha;
    packet.beta = beta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE, (const char *)&packet, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
#endif
}

/**
 * @brief Send a v4hp_calib_reference message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_v4hp_calib_reference_send_struct(mavlink_channel_t chan, const mavlink_v4hp_calib_reference_t* v4hp_calib_reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_v4hp_calib_reference_send(chan, v4hp_calib_reference->timestamp, v4hp_calib_reference->alpha, v4hp_calib_reference->beta);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE, (const char *)v4hp_calib_reference, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_v4hp_calib_reference_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, int16_t alpha, int16_t beta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int16_t(buf, 8, alpha);
    _mav_put_int16_t(buf, 10, beta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE, buf, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
#else
    mavlink_v4hp_calib_reference_t *packet = (mavlink_v4hp_calib_reference_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->alpha = alpha;
    packet->beta = beta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE, (const char *)packet, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_MIN_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_CRC);
#endif
}
#endif

#endif

// MESSAGE V4HP_CALIB_REFERENCE UNPACKING


/**
 * @brief Get field timestamp from v4hp_calib_reference message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_v4hp_calib_reference_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field alpha from v4hp_calib_reference message
 *
 * @return alpha (degrees*10)
 */
static inline int16_t mavlink_msg_v4hp_calib_reference_get_alpha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field beta from v4hp_calib_reference message
 *
 * @return beta (degrees*10)
 */
static inline int16_t mavlink_msg_v4hp_calib_reference_get_beta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a v4hp_calib_reference message into a struct
 *
 * @param msg The message to decode
 * @param v4hp_calib_reference C-struct to decode the message contents into
 */
static inline void mavlink_msg_v4hp_calib_reference_decode(const mavlink_message_t* msg, mavlink_v4hp_calib_reference_t* v4hp_calib_reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    v4hp_calib_reference->timestamp = mavlink_msg_v4hp_calib_reference_get_timestamp(msg);
    v4hp_calib_reference->alpha = mavlink_msg_v4hp_calib_reference_get_alpha(msg);
    v4hp_calib_reference->beta = mavlink_msg_v4hp_calib_reference_get_beta(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN? msg->len : MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN;
        memset(v4hp_calib_reference, 0, MAVLINK_MSG_ID_V4HP_CALIB_REFERENCE_LEN);
    memcpy(v4hp_calib_reference, _MAV_PAYLOAD(msg), len);
#endif
}
