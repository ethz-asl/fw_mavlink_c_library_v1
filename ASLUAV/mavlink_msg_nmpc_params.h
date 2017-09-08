#pragma once
// MESSAGE NMPC_PARAMS PACKING

#define MAVLINK_MSG_ID_NMPC_PARAMS 214

MAVPACKED(
typedef struct __mavlink_nmpc_params_t {
 float R_acpt; /*< Track switch acceptance radius [m]*/
 float ceta_acpt; /*< Cosine of track switch acceptance angle [~]*/
 float alpha_p_co; /*< Angle of attack upper cutoff [rad]*/
 float alpha_m_co; /*< Angle of attack lower cutoff [rad]*/
 float alpha_delta_co; /*< Angle of attack cutoff transition length [rad]*/
 float T_b_lat; /*< Lateral-directional track error boundary constant [s]*/
 float T_b_lon; /*< Longitudinal track error boundary constant [s]*/
 float Qdiag[11]; /*< Objective weights*/
}) mavlink_nmpc_params_t;

#define MAVLINK_MSG_ID_NMPC_PARAMS_LEN 72
#define MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN 72
#define MAVLINK_MSG_ID_214_LEN 72
#define MAVLINK_MSG_ID_214_MIN_LEN 72

#define MAVLINK_MSG_ID_NMPC_PARAMS_CRC 177
#define MAVLINK_MSG_ID_214_CRC 177

#define MAVLINK_MSG_NMPC_PARAMS_FIELD_QDIAG_LEN 11

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NMPC_PARAMS { \
    214, \
    "NMPC_PARAMS", \
    8, \
    {  { "R_acpt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nmpc_params_t, R_acpt) }, \
         { "ceta_acpt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nmpc_params_t, ceta_acpt) }, \
         { "alpha_p_co", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nmpc_params_t, alpha_p_co) }, \
         { "alpha_m_co", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nmpc_params_t, alpha_m_co) }, \
         { "alpha_delta_co", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nmpc_params_t, alpha_delta_co) }, \
         { "T_b_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nmpc_params_t, T_b_lat) }, \
         { "T_b_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nmpc_params_t, T_b_lon) }, \
         { "Qdiag", NULL, MAVLINK_TYPE_FLOAT, 11, 28, offsetof(mavlink_nmpc_params_t, Qdiag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NMPC_PARAMS { \
    "NMPC_PARAMS", \
    8, \
    {  { "R_acpt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nmpc_params_t, R_acpt) }, \
         { "ceta_acpt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nmpc_params_t, ceta_acpt) }, \
         { "alpha_p_co", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nmpc_params_t, alpha_p_co) }, \
         { "alpha_m_co", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nmpc_params_t, alpha_m_co) }, \
         { "alpha_delta_co", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nmpc_params_t, alpha_delta_co) }, \
         { "T_b_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nmpc_params_t, T_b_lat) }, \
         { "T_b_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nmpc_params_t, T_b_lon) }, \
         { "Qdiag", NULL, MAVLINK_TYPE_FLOAT, 11, 28, offsetof(mavlink_nmpc_params_t, Qdiag) }, \
         } \
}
#endif

/**
 * @brief Pack a nmpc_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param R_acpt Track switch acceptance radius [m]
 * @param ceta_acpt Cosine of track switch acceptance angle [~]
 * @param alpha_p_co Angle of attack upper cutoff [rad]
 * @param alpha_m_co Angle of attack lower cutoff [rad]
 * @param alpha_delta_co Angle of attack cutoff transition length [rad]
 * @param T_b_lat Lateral-directional track error boundary constant [s]
 * @param T_b_lon Longitudinal track error boundary constant [s]
 * @param Qdiag Objective weights
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nmpc_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float R_acpt, float ceta_acpt, float alpha_p_co, float alpha_m_co, float alpha_delta_co, float T_b_lat, float T_b_lon, const float *Qdiag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NMPC_PARAMS_LEN];
    _mav_put_float(buf, 0, R_acpt);
    _mav_put_float(buf, 4, ceta_acpt);
    _mav_put_float(buf, 8, alpha_p_co);
    _mav_put_float(buf, 12, alpha_m_co);
    _mav_put_float(buf, 16, alpha_delta_co);
    _mav_put_float(buf, 20, T_b_lat);
    _mav_put_float(buf, 24, T_b_lon);
    _mav_put_float_array(buf, 28, Qdiag, 11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NMPC_PARAMS_LEN);
#else
    mavlink_nmpc_params_t packet;
    packet.R_acpt = R_acpt;
    packet.ceta_acpt = ceta_acpt;
    packet.alpha_p_co = alpha_p_co;
    packet.alpha_m_co = alpha_m_co;
    packet.alpha_delta_co = alpha_delta_co;
    packet.T_b_lat = T_b_lat;
    packet.T_b_lon = T_b_lon;
    mav_array_memcpy(packet.Qdiag, Qdiag, sizeof(float)*11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NMPC_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NMPC_PARAMS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
}

/**
 * @brief Pack a nmpc_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param R_acpt Track switch acceptance radius [m]
 * @param ceta_acpt Cosine of track switch acceptance angle [~]
 * @param alpha_p_co Angle of attack upper cutoff [rad]
 * @param alpha_m_co Angle of attack lower cutoff [rad]
 * @param alpha_delta_co Angle of attack cutoff transition length [rad]
 * @param T_b_lat Lateral-directional track error boundary constant [s]
 * @param T_b_lon Longitudinal track error boundary constant [s]
 * @param Qdiag Objective weights
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nmpc_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float R_acpt,float ceta_acpt,float alpha_p_co,float alpha_m_co,float alpha_delta_co,float T_b_lat,float T_b_lon,const float *Qdiag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NMPC_PARAMS_LEN];
    _mav_put_float(buf, 0, R_acpt);
    _mav_put_float(buf, 4, ceta_acpt);
    _mav_put_float(buf, 8, alpha_p_co);
    _mav_put_float(buf, 12, alpha_m_co);
    _mav_put_float(buf, 16, alpha_delta_co);
    _mav_put_float(buf, 20, T_b_lat);
    _mav_put_float(buf, 24, T_b_lon);
    _mav_put_float_array(buf, 28, Qdiag, 11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NMPC_PARAMS_LEN);
#else
    mavlink_nmpc_params_t packet;
    packet.R_acpt = R_acpt;
    packet.ceta_acpt = ceta_acpt;
    packet.alpha_p_co = alpha_p_co;
    packet.alpha_m_co = alpha_m_co;
    packet.alpha_delta_co = alpha_delta_co;
    packet.T_b_lat = T_b_lat;
    packet.T_b_lon = T_b_lon;
    mav_array_memcpy(packet.Qdiag, Qdiag, sizeof(float)*11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NMPC_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NMPC_PARAMS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
}

/**
 * @brief Encode a nmpc_params struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nmpc_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nmpc_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nmpc_params_t* nmpc_params)
{
    return mavlink_msg_nmpc_params_pack(system_id, component_id, msg, nmpc_params->R_acpt, nmpc_params->ceta_acpt, nmpc_params->alpha_p_co, nmpc_params->alpha_m_co, nmpc_params->alpha_delta_co, nmpc_params->T_b_lat, nmpc_params->T_b_lon, nmpc_params->Qdiag);
}

/**
 * @brief Encode a nmpc_params struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nmpc_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nmpc_params_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nmpc_params_t* nmpc_params)
{
    return mavlink_msg_nmpc_params_pack_chan(system_id, component_id, chan, msg, nmpc_params->R_acpt, nmpc_params->ceta_acpt, nmpc_params->alpha_p_co, nmpc_params->alpha_m_co, nmpc_params->alpha_delta_co, nmpc_params->T_b_lat, nmpc_params->T_b_lon, nmpc_params->Qdiag);
}

/**
 * @brief Send a nmpc_params message
 * @param chan MAVLink channel to send the message
 *
 * @param R_acpt Track switch acceptance radius [m]
 * @param ceta_acpt Cosine of track switch acceptance angle [~]
 * @param alpha_p_co Angle of attack upper cutoff [rad]
 * @param alpha_m_co Angle of attack lower cutoff [rad]
 * @param alpha_delta_co Angle of attack cutoff transition length [rad]
 * @param T_b_lat Lateral-directional track error boundary constant [s]
 * @param T_b_lon Longitudinal track error boundary constant [s]
 * @param Qdiag Objective weights
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nmpc_params_send(mavlink_channel_t chan, float R_acpt, float ceta_acpt, float alpha_p_co, float alpha_m_co, float alpha_delta_co, float T_b_lat, float T_b_lon, const float *Qdiag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NMPC_PARAMS_LEN];
    _mav_put_float(buf, 0, R_acpt);
    _mav_put_float(buf, 4, ceta_acpt);
    _mav_put_float(buf, 8, alpha_p_co);
    _mav_put_float(buf, 12, alpha_m_co);
    _mav_put_float(buf, 16, alpha_delta_co);
    _mav_put_float(buf, 20, T_b_lat);
    _mav_put_float(buf, 24, T_b_lon);
    _mav_put_float_array(buf, 28, Qdiag, 11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NMPC_PARAMS, buf, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
#else
    mavlink_nmpc_params_t packet;
    packet.R_acpt = R_acpt;
    packet.ceta_acpt = ceta_acpt;
    packet.alpha_p_co = alpha_p_co;
    packet.alpha_m_co = alpha_m_co;
    packet.alpha_delta_co = alpha_delta_co;
    packet.T_b_lat = T_b_lat;
    packet.T_b_lon = T_b_lon;
    mav_array_memcpy(packet.Qdiag, Qdiag, sizeof(float)*11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NMPC_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
#endif
}

/**
 * @brief Send a nmpc_params message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nmpc_params_send_struct(mavlink_channel_t chan, const mavlink_nmpc_params_t* nmpc_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nmpc_params_send(chan, nmpc_params->R_acpt, nmpc_params->ceta_acpt, nmpc_params->alpha_p_co, nmpc_params->alpha_m_co, nmpc_params->alpha_delta_co, nmpc_params->T_b_lat, nmpc_params->T_b_lon, nmpc_params->Qdiag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NMPC_PARAMS, (const char *)nmpc_params, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NMPC_PARAMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nmpc_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float R_acpt, float ceta_acpt, float alpha_p_co, float alpha_m_co, float alpha_delta_co, float T_b_lat, float T_b_lon, const float *Qdiag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, R_acpt);
    _mav_put_float(buf, 4, ceta_acpt);
    _mav_put_float(buf, 8, alpha_p_co);
    _mav_put_float(buf, 12, alpha_m_co);
    _mav_put_float(buf, 16, alpha_delta_co);
    _mav_put_float(buf, 20, T_b_lat);
    _mav_put_float(buf, 24, T_b_lon);
    _mav_put_float_array(buf, 28, Qdiag, 11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NMPC_PARAMS, buf, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
#else
    mavlink_nmpc_params_t *packet = (mavlink_nmpc_params_t *)msgbuf;
    packet->R_acpt = R_acpt;
    packet->ceta_acpt = ceta_acpt;
    packet->alpha_p_co = alpha_p_co;
    packet->alpha_m_co = alpha_m_co;
    packet->alpha_delta_co = alpha_delta_co;
    packet->T_b_lat = T_b_lat;
    packet->T_b_lon = T_b_lon;
    mav_array_memcpy(packet->Qdiag, Qdiag, sizeof(float)*11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NMPC_PARAMS, (const char *)packet, MAVLINK_MSG_ID_NMPC_PARAMS_MIN_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_LEN, MAVLINK_MSG_ID_NMPC_PARAMS_CRC);
#endif
}
#endif

#endif

// MESSAGE NMPC_PARAMS UNPACKING


/**
 * @brief Get field R_acpt from nmpc_params message
 *
 * @return Track switch acceptance radius [m]
 */
static inline float mavlink_msg_nmpc_params_get_R_acpt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ceta_acpt from nmpc_params message
 *
 * @return Cosine of track switch acceptance angle [~]
 */
static inline float mavlink_msg_nmpc_params_get_ceta_acpt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field alpha_p_co from nmpc_params message
 *
 * @return Angle of attack upper cutoff [rad]
 */
static inline float mavlink_msg_nmpc_params_get_alpha_p_co(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field alpha_m_co from nmpc_params message
 *
 * @return Angle of attack lower cutoff [rad]
 */
static inline float mavlink_msg_nmpc_params_get_alpha_m_co(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field alpha_delta_co from nmpc_params message
 *
 * @return Angle of attack cutoff transition length [rad]
 */
static inline float mavlink_msg_nmpc_params_get_alpha_delta_co(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field T_b_lat from nmpc_params message
 *
 * @return Lateral-directional track error boundary constant [s]
 */
static inline float mavlink_msg_nmpc_params_get_T_b_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field T_b_lon from nmpc_params message
 *
 * @return Longitudinal track error boundary constant [s]
 */
static inline float mavlink_msg_nmpc_params_get_T_b_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field Qdiag from nmpc_params message
 *
 * @return Objective weights
 */
static inline uint16_t mavlink_msg_nmpc_params_get_Qdiag(const mavlink_message_t* msg, float *Qdiag)
{
    return _MAV_RETURN_float_array(msg, Qdiag, 11,  28);
}

/**
 * @brief Decode a nmpc_params message into a struct
 *
 * @param msg The message to decode
 * @param nmpc_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_nmpc_params_decode(const mavlink_message_t* msg, mavlink_nmpc_params_t* nmpc_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nmpc_params->R_acpt = mavlink_msg_nmpc_params_get_R_acpt(msg);
    nmpc_params->ceta_acpt = mavlink_msg_nmpc_params_get_ceta_acpt(msg);
    nmpc_params->alpha_p_co = mavlink_msg_nmpc_params_get_alpha_p_co(msg);
    nmpc_params->alpha_m_co = mavlink_msg_nmpc_params_get_alpha_m_co(msg);
    nmpc_params->alpha_delta_co = mavlink_msg_nmpc_params_get_alpha_delta_co(msg);
    nmpc_params->T_b_lat = mavlink_msg_nmpc_params_get_T_b_lat(msg);
    nmpc_params->T_b_lon = mavlink_msg_nmpc_params_get_T_b_lon(msg);
    mavlink_msg_nmpc_params_get_Qdiag(msg, nmpc_params->Qdiag);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NMPC_PARAMS_LEN? msg->len : MAVLINK_MSG_ID_NMPC_PARAMS_LEN;
        memset(nmpc_params, 0, MAVLINK_MSG_ID_NMPC_PARAMS_LEN);
    memcpy(nmpc_params, _MAV_PAYLOAD(msg), len);
#endif
}
