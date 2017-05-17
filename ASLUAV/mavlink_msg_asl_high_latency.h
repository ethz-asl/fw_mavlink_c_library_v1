#pragma once
// MESSAGE ASL_HIGH_LATENCY PACKING

#define MAVLINK_MSG_ID_ASL_HIGH_LATENCY 213

MAVPACKED(
typedef struct __mavlink_asl_high_latency_t {
 uint32_t time_boot_ms; /*< Timestamp (since system boot) [ms]*/
 int32_t latitude; /*< Latitude [deg * 1E7]*/
 int32_t longitude; /*< Longitude [deg * 1E7]*/
 int16_t altitude_amsl; /*< Altitude above mean sea level [m]*/
 int16_t altitude_sp; /*< Altitude setpoint above mean sea level [m]*/
 uint16_t wp_num; /*< current waypoint number*/
 uint8_t base_mode; /*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
 int8_t roll; /*< Roll angle [deg]*/
 uint8_t heading; /*< Heading [deg / 2]*/
 uint8_t throttle; /*< Current throttle [1/100]*/
 uint8_t airspeed; /*< Airspeed [m/s * 10]*/
 uint8_t airspeed_sp; /*< Airspeed setpoint [m/s * 10]*/
 uint8_t windspeed; /*< Windspeed [m/s * 10]*/
 uint8_t groundspeed; /*< Groundspeed [m/s * 10]*/
 uint8_t gps_nsat; /*< Number of satellites visible. If unknown, set to 255*/
 uint8_t gps_fix_type; /*< See the GPS_FIX_TYPE enum.*/
 int8_t temperature_air; /*< Air temperature (degrees C) from airspeed sensor*/
 uint8_t failsafe; /*< failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence))*/
 uint8_t v_avg_mppt0; /*< Average voltage of MPPT 0 (solar panel side) since last message [V * 10]*/
 uint8_t v_avg_mppt1; /*< Average voltage of MPPT 1 (solar panel side) since last msg [V * 10]*/
 uint8_t v_avg_mppt2; /*< Average voltage of MPPT 2 (solar panel side) since last msg [V * 10]*/
 uint8_t state_batmon0; /*< Status bits of batmon 0*/
 uint8_t state_batmon1; /*< Status bits of batmon 1*/
 uint8_t state_batmon2; /*< Status bits of batmon 2*/
 int8_t p_avg_bat; /*< Average power of all batteries since last message ( <0: CHG, >0: DCHG) [W / 4]*/
 uint8_t v_avg_bat0; /*< Averag voltage of battery 0 since last msg [V * 10]*/
 uint8_t v_avg_bat1; /*< Averag voltage of battery 1 since last msg [V * 10]*/
 uint8_t v_avg_bat2; /*< Averag voltage of battery 2 since last msg [V * 10]*/
 uint8_t status_pwrbrd; /*< Power board status register, as in SENS_POWER_BOARD*/
 uint8_t p_out; /*< Average power consumed by all components (Motors, Avionics, OBC, ...) since last msg [W / 2]*/
}) mavlink_asl_high_latency_t;

#define MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN 42
#define MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN 42
#define MAVLINK_MSG_ID_213_LEN 42
#define MAVLINK_MSG_ID_213_MIN_LEN 42

#define MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC 159
#define MAVLINK_MSG_ID_213_CRC 159



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASL_HIGH_LATENCY { \
    213, \
    "ASL_HIGH_LATENCY", \
    30, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_asl_high_latency_t, time_boot_ms) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_asl_high_latency_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_asl_high_latency_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_asl_high_latency_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_asl_high_latency_t, altitude_sp) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_asl_high_latency_t, wp_num) }, \
         { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_asl_high_latency_t, base_mode) }, \
         { "roll", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_asl_high_latency_t, roll) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_asl_high_latency_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_asl_high_latency_t, throttle) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_asl_high_latency_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_asl_high_latency_t, airspeed_sp) }, \
         { "windspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_asl_high_latency_t, windspeed) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_asl_high_latency_t, groundspeed) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_asl_high_latency_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_asl_high_latency_t, gps_fix_type) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_asl_high_latency_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_asl_high_latency_t, failsafe) }, \
         { "v_avg_mppt0", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_asl_high_latency_t, v_avg_mppt0) }, \
         { "v_avg_mppt1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_asl_high_latency_t, v_avg_mppt1) }, \
         { "v_avg_mppt2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asl_high_latency_t, v_avg_mppt2) }, \
         { "state_batmon0", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_asl_high_latency_t, state_batmon0) }, \
         { "state_batmon1", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_asl_high_latency_t, state_batmon1) }, \
         { "state_batmon2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_asl_high_latency_t, state_batmon2) }, \
         { "p_avg_bat", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_asl_high_latency_t, p_avg_bat) }, \
         { "v_avg_bat0", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_asl_high_latency_t, v_avg_bat0) }, \
         { "v_avg_bat1", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_asl_high_latency_t, v_avg_bat1) }, \
         { "v_avg_bat2", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_asl_high_latency_t, v_avg_bat2) }, \
         { "status_pwrbrd", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_asl_high_latency_t, status_pwrbrd) }, \
         { "p_out", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_asl_high_latency_t, p_out) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASL_HIGH_LATENCY { \
    "ASL_HIGH_LATENCY", \
    30, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_asl_high_latency_t, time_boot_ms) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_asl_high_latency_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_asl_high_latency_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_asl_high_latency_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_asl_high_latency_t, altitude_sp) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_asl_high_latency_t, wp_num) }, \
         { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_asl_high_latency_t, base_mode) }, \
         { "roll", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_asl_high_latency_t, roll) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_asl_high_latency_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_asl_high_latency_t, throttle) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_asl_high_latency_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_asl_high_latency_t, airspeed_sp) }, \
         { "windspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_asl_high_latency_t, windspeed) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_asl_high_latency_t, groundspeed) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_asl_high_latency_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_asl_high_latency_t, gps_fix_type) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_asl_high_latency_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_asl_high_latency_t, failsafe) }, \
         { "v_avg_mppt0", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_asl_high_latency_t, v_avg_mppt0) }, \
         { "v_avg_mppt1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_asl_high_latency_t, v_avg_mppt1) }, \
         { "v_avg_mppt2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asl_high_latency_t, v_avg_mppt2) }, \
         { "state_batmon0", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_asl_high_latency_t, state_batmon0) }, \
         { "state_batmon1", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_asl_high_latency_t, state_batmon1) }, \
         { "state_batmon2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_asl_high_latency_t, state_batmon2) }, \
         { "p_avg_bat", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_asl_high_latency_t, p_avg_bat) }, \
         { "v_avg_bat0", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_asl_high_latency_t, v_avg_bat0) }, \
         { "v_avg_bat1", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_asl_high_latency_t, v_avg_bat1) }, \
         { "v_avg_bat2", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_asl_high_latency_t, v_avg_bat2) }, \
         { "status_pwrbrd", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_asl_high_latency_t, status_pwrbrd) }, \
         { "p_out", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_asl_high_latency_t, p_out) }, \
         } \
}
#endif

/**
 * @brief Pack a asl_high_latency message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (since system boot) [ms]
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param roll Roll angle [deg]
 * @param heading Heading [deg / 2]
 * @param throttle Current throttle [1/100]
 * @param latitude Latitude [deg * 1E7]
 * @param longitude Longitude [deg * 1E7]
 * @param altitude_amsl Altitude above mean sea level [m]
 * @param altitude_sp Altitude setpoint above mean sea level [m]
 * @param airspeed Airspeed [m/s * 10]
 * @param airspeed_sp Airspeed setpoint [m/s * 10]
 * @param windspeed Windspeed [m/s * 10]
 * @param groundspeed Groundspeed [m/s * 10]
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence))
 * @param wp_num current waypoint number
 * @param v_avg_mppt0 Average voltage of MPPT 0 (solar panel side) since last message [V * 10]
 * @param v_avg_mppt1 Average voltage of MPPT 1 (solar panel side) since last msg [V * 10]
 * @param v_avg_mppt2 Average voltage of MPPT 2 (solar panel side) since last msg [V * 10]
 * @param state_batmon0 Status bits of batmon 0
 * @param state_batmon1 Status bits of batmon 1
 * @param state_batmon2 Status bits of batmon 2
 * @param p_avg_bat Average power of all batteries since last message ( <0: CHG, >0: DCHG) [W / 4]
 * @param v_avg_bat0 Averag voltage of battery 0 since last msg [V * 10]
 * @param v_avg_bat1 Averag voltage of battery 1 since last msg [V * 10]
 * @param v_avg_bat2 Averag voltage of battery 2 since last msg [V * 10]
 * @param status_pwrbrd Power board status register, as in SENS_POWER_BOARD
 * @param p_out Average power consumed by all components (Motors, Avionics, OBC, ...) since last msg [W / 2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asl_high_latency_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t base_mode, int8_t roll, uint8_t heading, uint8_t throttle, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t windspeed, uint8_t groundspeed, uint8_t gps_nsat, uint8_t gps_fix_type, int8_t temperature_air, uint8_t failsafe, uint16_t wp_num, uint8_t v_avg_mppt0, uint8_t v_avg_mppt1, uint8_t v_avg_mppt2, uint8_t state_batmon0, uint8_t state_batmon1, uint8_t state_batmon2, int8_t p_avg_bat, uint8_t v_avg_bat0, uint8_t v_avg_bat1, uint8_t v_avg_bat2, uint8_t status_pwrbrd, uint8_t p_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, altitude_amsl);
    _mav_put_int16_t(buf, 14, altitude_sp);
    _mav_put_uint16_t(buf, 16, wp_num);
    _mav_put_uint8_t(buf, 18, base_mode);
    _mav_put_int8_t(buf, 19, roll);
    _mav_put_uint8_t(buf, 20, heading);
    _mav_put_uint8_t(buf, 21, throttle);
    _mav_put_uint8_t(buf, 22, airspeed);
    _mav_put_uint8_t(buf, 23, airspeed_sp);
    _mav_put_uint8_t(buf, 24, windspeed);
    _mav_put_uint8_t(buf, 25, groundspeed);
    _mav_put_uint8_t(buf, 26, gps_nsat);
    _mav_put_uint8_t(buf, 27, gps_fix_type);
    _mav_put_int8_t(buf, 28, temperature_air);
    _mav_put_uint8_t(buf, 29, failsafe);
    _mav_put_uint8_t(buf, 30, v_avg_mppt0);
    _mav_put_uint8_t(buf, 31, v_avg_mppt1);
    _mav_put_uint8_t(buf, 32, v_avg_mppt2);
    _mav_put_uint8_t(buf, 33, state_batmon0);
    _mav_put_uint8_t(buf, 34, state_batmon1);
    _mav_put_uint8_t(buf, 35, state_batmon2);
    _mav_put_int8_t(buf, 36, p_avg_bat);
    _mav_put_uint8_t(buf, 37, v_avg_bat0);
    _mav_put_uint8_t(buf, 38, v_avg_bat1);
    _mav_put_uint8_t(buf, 39, v_avg_bat2);
    _mav_put_uint8_t(buf, 40, status_pwrbrd);
    _mav_put_uint8_t(buf, 41, p_out);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN);
#else
    mavlink_asl_high_latency_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_num = wp_num;
    packet.base_mode = base_mode;
    packet.roll = roll;
    packet.heading = heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.windspeed = windspeed;
    packet.groundspeed = groundspeed;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.v_avg_mppt0 = v_avg_mppt0;
    packet.v_avg_mppt1 = v_avg_mppt1;
    packet.v_avg_mppt2 = v_avg_mppt2;
    packet.state_batmon0 = state_batmon0;
    packet.state_batmon1 = state_batmon1;
    packet.state_batmon2 = state_batmon2;
    packet.p_avg_bat = p_avg_bat;
    packet.v_avg_bat0 = v_avg_bat0;
    packet.v_avg_bat1 = v_avg_bat1;
    packet.v_avg_bat2 = v_avg_bat2;
    packet.status_pwrbrd = status_pwrbrd;
    packet.p_out = p_out;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASL_HIGH_LATENCY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
}

/**
 * @brief Pack a asl_high_latency message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (since system boot) [ms]
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param roll Roll angle [deg]
 * @param heading Heading [deg / 2]
 * @param throttle Current throttle [1/100]
 * @param latitude Latitude [deg * 1E7]
 * @param longitude Longitude [deg * 1E7]
 * @param altitude_amsl Altitude above mean sea level [m]
 * @param altitude_sp Altitude setpoint above mean sea level [m]
 * @param airspeed Airspeed [m/s * 10]
 * @param airspeed_sp Airspeed setpoint [m/s * 10]
 * @param windspeed Windspeed [m/s * 10]
 * @param groundspeed Groundspeed [m/s * 10]
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence))
 * @param wp_num current waypoint number
 * @param v_avg_mppt0 Average voltage of MPPT 0 (solar panel side) since last message [V * 10]
 * @param v_avg_mppt1 Average voltage of MPPT 1 (solar panel side) since last msg [V * 10]
 * @param v_avg_mppt2 Average voltage of MPPT 2 (solar panel side) since last msg [V * 10]
 * @param state_batmon0 Status bits of batmon 0
 * @param state_batmon1 Status bits of batmon 1
 * @param state_batmon2 Status bits of batmon 2
 * @param p_avg_bat Average power of all batteries since last message ( <0: CHG, >0: DCHG) [W / 4]
 * @param v_avg_bat0 Averag voltage of battery 0 since last msg [V * 10]
 * @param v_avg_bat1 Averag voltage of battery 1 since last msg [V * 10]
 * @param v_avg_bat2 Averag voltage of battery 2 since last msg [V * 10]
 * @param status_pwrbrd Power board status register, as in SENS_POWER_BOARD
 * @param p_out Average power consumed by all components (Motors, Avionics, OBC, ...) since last msg [W / 2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asl_high_latency_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t base_mode,int8_t roll,uint8_t heading,uint8_t throttle,int32_t latitude,int32_t longitude,int16_t altitude_amsl,int16_t altitude_sp,uint8_t airspeed,uint8_t airspeed_sp,uint8_t windspeed,uint8_t groundspeed,uint8_t gps_nsat,uint8_t gps_fix_type,int8_t temperature_air,uint8_t failsafe,uint16_t wp_num,uint8_t v_avg_mppt0,uint8_t v_avg_mppt1,uint8_t v_avg_mppt2,uint8_t state_batmon0,uint8_t state_batmon1,uint8_t state_batmon2,int8_t p_avg_bat,uint8_t v_avg_bat0,uint8_t v_avg_bat1,uint8_t v_avg_bat2,uint8_t status_pwrbrd,uint8_t p_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, altitude_amsl);
    _mav_put_int16_t(buf, 14, altitude_sp);
    _mav_put_uint16_t(buf, 16, wp_num);
    _mav_put_uint8_t(buf, 18, base_mode);
    _mav_put_int8_t(buf, 19, roll);
    _mav_put_uint8_t(buf, 20, heading);
    _mav_put_uint8_t(buf, 21, throttle);
    _mav_put_uint8_t(buf, 22, airspeed);
    _mav_put_uint8_t(buf, 23, airspeed_sp);
    _mav_put_uint8_t(buf, 24, windspeed);
    _mav_put_uint8_t(buf, 25, groundspeed);
    _mav_put_uint8_t(buf, 26, gps_nsat);
    _mav_put_uint8_t(buf, 27, gps_fix_type);
    _mav_put_int8_t(buf, 28, temperature_air);
    _mav_put_uint8_t(buf, 29, failsafe);
    _mav_put_uint8_t(buf, 30, v_avg_mppt0);
    _mav_put_uint8_t(buf, 31, v_avg_mppt1);
    _mav_put_uint8_t(buf, 32, v_avg_mppt2);
    _mav_put_uint8_t(buf, 33, state_batmon0);
    _mav_put_uint8_t(buf, 34, state_batmon1);
    _mav_put_uint8_t(buf, 35, state_batmon2);
    _mav_put_int8_t(buf, 36, p_avg_bat);
    _mav_put_uint8_t(buf, 37, v_avg_bat0);
    _mav_put_uint8_t(buf, 38, v_avg_bat1);
    _mav_put_uint8_t(buf, 39, v_avg_bat2);
    _mav_put_uint8_t(buf, 40, status_pwrbrd);
    _mav_put_uint8_t(buf, 41, p_out);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN);
#else
    mavlink_asl_high_latency_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_num = wp_num;
    packet.base_mode = base_mode;
    packet.roll = roll;
    packet.heading = heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.windspeed = windspeed;
    packet.groundspeed = groundspeed;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.v_avg_mppt0 = v_avg_mppt0;
    packet.v_avg_mppt1 = v_avg_mppt1;
    packet.v_avg_mppt2 = v_avg_mppt2;
    packet.state_batmon0 = state_batmon0;
    packet.state_batmon1 = state_batmon1;
    packet.state_batmon2 = state_batmon2;
    packet.p_avg_bat = p_avg_bat;
    packet.v_avg_bat0 = v_avg_bat0;
    packet.v_avg_bat1 = v_avg_bat1;
    packet.v_avg_bat2 = v_avg_bat2;
    packet.status_pwrbrd = status_pwrbrd;
    packet.p_out = p_out;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASL_HIGH_LATENCY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
}

/**
 * @brief Encode a asl_high_latency struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param asl_high_latency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asl_high_latency_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_asl_high_latency_t* asl_high_latency)
{
    return mavlink_msg_asl_high_latency_pack(system_id, component_id, msg, asl_high_latency->time_boot_ms, asl_high_latency->base_mode, asl_high_latency->roll, asl_high_latency->heading, asl_high_latency->throttle, asl_high_latency->latitude, asl_high_latency->longitude, asl_high_latency->altitude_amsl, asl_high_latency->altitude_sp, asl_high_latency->airspeed, asl_high_latency->airspeed_sp, asl_high_latency->windspeed, asl_high_latency->groundspeed, asl_high_latency->gps_nsat, asl_high_latency->gps_fix_type, asl_high_latency->temperature_air, asl_high_latency->failsafe, asl_high_latency->wp_num, asl_high_latency->v_avg_mppt0, asl_high_latency->v_avg_mppt1, asl_high_latency->v_avg_mppt2, asl_high_latency->state_batmon0, asl_high_latency->state_batmon1, asl_high_latency->state_batmon2, asl_high_latency->p_avg_bat, asl_high_latency->v_avg_bat0, asl_high_latency->v_avg_bat1, asl_high_latency->v_avg_bat2, asl_high_latency->status_pwrbrd, asl_high_latency->p_out);
}

/**
 * @brief Encode a asl_high_latency struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param asl_high_latency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asl_high_latency_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_asl_high_latency_t* asl_high_latency)
{
    return mavlink_msg_asl_high_latency_pack_chan(system_id, component_id, chan, msg, asl_high_latency->time_boot_ms, asl_high_latency->base_mode, asl_high_latency->roll, asl_high_latency->heading, asl_high_latency->throttle, asl_high_latency->latitude, asl_high_latency->longitude, asl_high_latency->altitude_amsl, asl_high_latency->altitude_sp, asl_high_latency->airspeed, asl_high_latency->airspeed_sp, asl_high_latency->windspeed, asl_high_latency->groundspeed, asl_high_latency->gps_nsat, asl_high_latency->gps_fix_type, asl_high_latency->temperature_air, asl_high_latency->failsafe, asl_high_latency->wp_num, asl_high_latency->v_avg_mppt0, asl_high_latency->v_avg_mppt1, asl_high_latency->v_avg_mppt2, asl_high_latency->state_batmon0, asl_high_latency->state_batmon1, asl_high_latency->state_batmon2, asl_high_latency->p_avg_bat, asl_high_latency->v_avg_bat0, asl_high_latency->v_avg_bat1, asl_high_latency->v_avg_bat2, asl_high_latency->status_pwrbrd, asl_high_latency->p_out);
}

/**
 * @brief Send a asl_high_latency message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (since system boot) [ms]
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param roll Roll angle [deg]
 * @param heading Heading [deg / 2]
 * @param throttle Current throttle [1/100]
 * @param latitude Latitude [deg * 1E7]
 * @param longitude Longitude [deg * 1E7]
 * @param altitude_amsl Altitude above mean sea level [m]
 * @param altitude_sp Altitude setpoint above mean sea level [m]
 * @param airspeed Airspeed [m/s * 10]
 * @param airspeed_sp Airspeed setpoint [m/s * 10]
 * @param windspeed Windspeed [m/s * 10]
 * @param groundspeed Groundspeed [m/s * 10]
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence))
 * @param wp_num current waypoint number
 * @param v_avg_mppt0 Average voltage of MPPT 0 (solar panel side) since last message [V * 10]
 * @param v_avg_mppt1 Average voltage of MPPT 1 (solar panel side) since last msg [V * 10]
 * @param v_avg_mppt2 Average voltage of MPPT 2 (solar panel side) since last msg [V * 10]
 * @param state_batmon0 Status bits of batmon 0
 * @param state_batmon1 Status bits of batmon 1
 * @param state_batmon2 Status bits of batmon 2
 * @param p_avg_bat Average power of all batteries since last message ( <0: CHG, >0: DCHG) [W / 4]
 * @param v_avg_bat0 Averag voltage of battery 0 since last msg [V * 10]
 * @param v_avg_bat1 Averag voltage of battery 1 since last msg [V * 10]
 * @param v_avg_bat2 Averag voltage of battery 2 since last msg [V * 10]
 * @param status_pwrbrd Power board status register, as in SENS_POWER_BOARD
 * @param p_out Average power consumed by all components (Motors, Avionics, OBC, ...) since last msg [W / 2]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_asl_high_latency_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t base_mode, int8_t roll, uint8_t heading, uint8_t throttle, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t windspeed, uint8_t groundspeed, uint8_t gps_nsat, uint8_t gps_fix_type, int8_t temperature_air, uint8_t failsafe, uint16_t wp_num, uint8_t v_avg_mppt0, uint8_t v_avg_mppt1, uint8_t v_avg_mppt2, uint8_t state_batmon0, uint8_t state_batmon1, uint8_t state_batmon2, int8_t p_avg_bat, uint8_t v_avg_bat0, uint8_t v_avg_bat1, uint8_t v_avg_bat2, uint8_t status_pwrbrd, uint8_t p_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, altitude_amsl);
    _mav_put_int16_t(buf, 14, altitude_sp);
    _mav_put_uint16_t(buf, 16, wp_num);
    _mav_put_uint8_t(buf, 18, base_mode);
    _mav_put_int8_t(buf, 19, roll);
    _mav_put_uint8_t(buf, 20, heading);
    _mav_put_uint8_t(buf, 21, throttle);
    _mav_put_uint8_t(buf, 22, airspeed);
    _mav_put_uint8_t(buf, 23, airspeed_sp);
    _mav_put_uint8_t(buf, 24, windspeed);
    _mav_put_uint8_t(buf, 25, groundspeed);
    _mav_put_uint8_t(buf, 26, gps_nsat);
    _mav_put_uint8_t(buf, 27, gps_fix_type);
    _mav_put_int8_t(buf, 28, temperature_air);
    _mav_put_uint8_t(buf, 29, failsafe);
    _mav_put_uint8_t(buf, 30, v_avg_mppt0);
    _mav_put_uint8_t(buf, 31, v_avg_mppt1);
    _mav_put_uint8_t(buf, 32, v_avg_mppt2);
    _mav_put_uint8_t(buf, 33, state_batmon0);
    _mav_put_uint8_t(buf, 34, state_batmon1);
    _mav_put_uint8_t(buf, 35, state_batmon2);
    _mav_put_int8_t(buf, 36, p_avg_bat);
    _mav_put_uint8_t(buf, 37, v_avg_bat0);
    _mav_put_uint8_t(buf, 38, v_avg_bat1);
    _mav_put_uint8_t(buf, 39, v_avg_bat2);
    _mav_put_uint8_t(buf, 40, status_pwrbrd);
    _mav_put_uint8_t(buf, 41, p_out);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY, buf, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
#else
    mavlink_asl_high_latency_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_num = wp_num;
    packet.base_mode = base_mode;
    packet.roll = roll;
    packet.heading = heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.windspeed = windspeed;
    packet.groundspeed = groundspeed;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.v_avg_mppt0 = v_avg_mppt0;
    packet.v_avg_mppt1 = v_avg_mppt1;
    packet.v_avg_mppt2 = v_avg_mppt2;
    packet.state_batmon0 = state_batmon0;
    packet.state_batmon1 = state_batmon1;
    packet.state_batmon2 = state_batmon2;
    packet.p_avg_bat = p_avg_bat;
    packet.v_avg_bat0 = v_avg_bat0;
    packet.v_avg_bat1 = v_avg_bat1;
    packet.v_avg_bat2 = v_avg_bat2;
    packet.status_pwrbrd = status_pwrbrd;
    packet.p_out = p_out;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY, (const char *)&packet, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
#endif
}

/**
 * @brief Send a asl_high_latency message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_asl_high_latency_send_struct(mavlink_channel_t chan, const mavlink_asl_high_latency_t* asl_high_latency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_asl_high_latency_send(chan, asl_high_latency->time_boot_ms, asl_high_latency->base_mode, asl_high_latency->roll, asl_high_latency->heading, asl_high_latency->throttle, asl_high_latency->latitude, asl_high_latency->longitude, asl_high_latency->altitude_amsl, asl_high_latency->altitude_sp, asl_high_latency->airspeed, asl_high_latency->airspeed_sp, asl_high_latency->windspeed, asl_high_latency->groundspeed, asl_high_latency->gps_nsat, asl_high_latency->gps_fix_type, asl_high_latency->temperature_air, asl_high_latency->failsafe, asl_high_latency->wp_num, asl_high_latency->v_avg_mppt0, asl_high_latency->v_avg_mppt1, asl_high_latency->v_avg_mppt2, asl_high_latency->state_batmon0, asl_high_latency->state_batmon1, asl_high_latency->state_batmon2, asl_high_latency->p_avg_bat, asl_high_latency->v_avg_bat0, asl_high_latency->v_avg_bat1, asl_high_latency->v_avg_bat2, asl_high_latency->status_pwrbrd, asl_high_latency->p_out);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY, (const char *)asl_high_latency, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_asl_high_latency_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t base_mode, int8_t roll, uint8_t heading, uint8_t throttle, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t windspeed, uint8_t groundspeed, uint8_t gps_nsat, uint8_t gps_fix_type, int8_t temperature_air, uint8_t failsafe, uint16_t wp_num, uint8_t v_avg_mppt0, uint8_t v_avg_mppt1, uint8_t v_avg_mppt2, uint8_t state_batmon0, uint8_t state_batmon1, uint8_t state_batmon2, int8_t p_avg_bat, uint8_t v_avg_bat0, uint8_t v_avg_bat1, uint8_t v_avg_bat2, uint8_t status_pwrbrd, uint8_t p_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, altitude_amsl);
    _mav_put_int16_t(buf, 14, altitude_sp);
    _mav_put_uint16_t(buf, 16, wp_num);
    _mav_put_uint8_t(buf, 18, base_mode);
    _mav_put_int8_t(buf, 19, roll);
    _mav_put_uint8_t(buf, 20, heading);
    _mav_put_uint8_t(buf, 21, throttle);
    _mav_put_uint8_t(buf, 22, airspeed);
    _mav_put_uint8_t(buf, 23, airspeed_sp);
    _mav_put_uint8_t(buf, 24, windspeed);
    _mav_put_uint8_t(buf, 25, groundspeed);
    _mav_put_uint8_t(buf, 26, gps_nsat);
    _mav_put_uint8_t(buf, 27, gps_fix_type);
    _mav_put_int8_t(buf, 28, temperature_air);
    _mav_put_uint8_t(buf, 29, failsafe);
    _mav_put_uint8_t(buf, 30, v_avg_mppt0);
    _mav_put_uint8_t(buf, 31, v_avg_mppt1);
    _mav_put_uint8_t(buf, 32, v_avg_mppt2);
    _mav_put_uint8_t(buf, 33, state_batmon0);
    _mav_put_uint8_t(buf, 34, state_batmon1);
    _mav_put_uint8_t(buf, 35, state_batmon2);
    _mav_put_int8_t(buf, 36, p_avg_bat);
    _mav_put_uint8_t(buf, 37, v_avg_bat0);
    _mav_put_uint8_t(buf, 38, v_avg_bat1);
    _mav_put_uint8_t(buf, 39, v_avg_bat2);
    _mav_put_uint8_t(buf, 40, status_pwrbrd);
    _mav_put_uint8_t(buf, 41, p_out);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY, buf, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
#else
    mavlink_asl_high_latency_t *packet = (mavlink_asl_high_latency_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude_amsl = altitude_amsl;
    packet->altitude_sp = altitude_sp;
    packet->wp_num = wp_num;
    packet->base_mode = base_mode;
    packet->roll = roll;
    packet->heading = heading;
    packet->throttle = throttle;
    packet->airspeed = airspeed;
    packet->airspeed_sp = airspeed_sp;
    packet->windspeed = windspeed;
    packet->groundspeed = groundspeed;
    packet->gps_nsat = gps_nsat;
    packet->gps_fix_type = gps_fix_type;
    packet->temperature_air = temperature_air;
    packet->failsafe = failsafe;
    packet->v_avg_mppt0 = v_avg_mppt0;
    packet->v_avg_mppt1 = v_avg_mppt1;
    packet->v_avg_mppt2 = v_avg_mppt2;
    packet->state_batmon0 = state_batmon0;
    packet->state_batmon1 = state_batmon1;
    packet->state_batmon2 = state_batmon2;
    packet->p_avg_bat = p_avg_bat;
    packet->v_avg_bat0 = v_avg_bat0;
    packet->v_avg_bat1 = v_avg_bat1;
    packet->v_avg_bat2 = v_avg_bat2;
    packet->status_pwrbrd = status_pwrbrd;
    packet->p_out = p_out;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_HIGH_LATENCY, (const char *)packet, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_CRC);
#endif
}
#endif

#endif

// MESSAGE ASL_HIGH_LATENCY UNPACKING


/**
 * @brief Get field time_boot_ms from asl_high_latency message
 *
 * @return Timestamp (since system boot) [ms]
 */
static inline uint32_t mavlink_msg_asl_high_latency_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field base_mode from asl_high_latency message
 *
 * @return System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_base_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field roll from asl_high_latency message
 *
 * @return Roll angle [deg]
 */
static inline int8_t mavlink_msg_asl_high_latency_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  19);
}

/**
 * @brief Get field heading from asl_high_latency message
 *
 * @return Heading [deg / 2]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field throttle from asl_high_latency message
 *
 * @return Current throttle [1/100]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field latitude from asl_high_latency message
 *
 * @return Latitude [deg * 1E7]
 */
static inline int32_t mavlink_msg_asl_high_latency_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from asl_high_latency message
 *
 * @return Longitude [deg * 1E7]
 */
static inline int32_t mavlink_msg_asl_high_latency_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude_amsl from asl_high_latency message
 *
 * @return Altitude above mean sea level [m]
 */
static inline int16_t mavlink_msg_asl_high_latency_get_altitude_amsl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field altitude_sp from asl_high_latency message
 *
 * @return Altitude setpoint above mean sea level [m]
 */
static inline int16_t mavlink_msg_asl_high_latency_get_altitude_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field airspeed from asl_high_latency message
 *
 * @return Airspeed [m/s * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field airspeed_sp from asl_high_latency message
 *
 * @return Airspeed setpoint [m/s * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_airspeed_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field windspeed from asl_high_latency message
 *
 * @return Windspeed [m/s * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_windspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field groundspeed from asl_high_latency message
 *
 * @return Groundspeed [m/s * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field gps_nsat from asl_high_latency message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_gps_nsat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field gps_fix_type from asl_high_latency message
 *
 * @return See the GPS_FIX_TYPE enum.
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_gps_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field temperature_air from asl_high_latency message
 *
 * @return Air temperature (degrees C) from airspeed sensor
 */
static inline int8_t mavlink_msg_asl_high_latency_get_temperature_air(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  28);
}

/**
 * @brief Get field failsafe from asl_high_latency message
 *
 * @return failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence))
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_failsafe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field wp_num from asl_high_latency message
 *
 * @return current waypoint number
 */
static inline uint16_t mavlink_msg_asl_high_latency_get_wp_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field v_avg_mppt0 from asl_high_latency message
 *
 * @return Average voltage of MPPT 0 (solar panel side) since last message [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_mppt0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field v_avg_mppt1 from asl_high_latency message
 *
 * @return Average voltage of MPPT 1 (solar panel side) since last msg [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_mppt1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field v_avg_mppt2 from asl_high_latency message
 *
 * @return Average voltage of MPPT 2 (solar panel side) since last msg [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_mppt2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field state_batmon0 from asl_high_latency message
 *
 * @return Status bits of batmon 0
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_state_batmon0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field state_batmon1 from asl_high_latency message
 *
 * @return Status bits of batmon 1
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_state_batmon1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field state_batmon2 from asl_high_latency message
 *
 * @return Status bits of batmon 2
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_state_batmon2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field p_avg_bat from asl_high_latency message
 *
 * @return Average power of all batteries since last message ( <0: CHG, >0: DCHG) [W / 4]
 */
static inline int8_t mavlink_msg_asl_high_latency_get_p_avg_bat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field v_avg_bat0 from asl_high_latency message
 *
 * @return Averag voltage of battery 0 since last msg [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_bat0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field v_avg_bat1 from asl_high_latency message
 *
 * @return Averag voltage of battery 1 since last msg [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_bat1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field v_avg_bat2 from asl_high_latency message
 *
 * @return Averag voltage of battery 2 since last msg [V * 10]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_v_avg_bat2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field status_pwrbrd from asl_high_latency message
 *
 * @return Power board status register, as in SENS_POWER_BOARD
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_status_pwrbrd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field p_out from asl_high_latency message
 *
 * @return Average power consumed by all components (Motors, Avionics, OBC, ...) since last msg [W / 2]
 */
static inline uint8_t mavlink_msg_asl_high_latency_get_p_out(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Decode a asl_high_latency message into a struct
 *
 * @param msg The message to decode
 * @param asl_high_latency C-struct to decode the message contents into
 */
static inline void mavlink_msg_asl_high_latency_decode(const mavlink_message_t* msg, mavlink_asl_high_latency_t* asl_high_latency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    asl_high_latency->time_boot_ms = mavlink_msg_asl_high_latency_get_time_boot_ms(msg);
    asl_high_latency->latitude = mavlink_msg_asl_high_latency_get_latitude(msg);
    asl_high_latency->longitude = mavlink_msg_asl_high_latency_get_longitude(msg);
    asl_high_latency->altitude_amsl = mavlink_msg_asl_high_latency_get_altitude_amsl(msg);
    asl_high_latency->altitude_sp = mavlink_msg_asl_high_latency_get_altitude_sp(msg);
    asl_high_latency->wp_num = mavlink_msg_asl_high_latency_get_wp_num(msg);
    asl_high_latency->base_mode = mavlink_msg_asl_high_latency_get_base_mode(msg);
    asl_high_latency->roll = mavlink_msg_asl_high_latency_get_roll(msg);
    asl_high_latency->heading = mavlink_msg_asl_high_latency_get_heading(msg);
    asl_high_latency->throttle = mavlink_msg_asl_high_latency_get_throttle(msg);
    asl_high_latency->airspeed = mavlink_msg_asl_high_latency_get_airspeed(msg);
    asl_high_latency->airspeed_sp = mavlink_msg_asl_high_latency_get_airspeed_sp(msg);
    asl_high_latency->windspeed = mavlink_msg_asl_high_latency_get_windspeed(msg);
    asl_high_latency->groundspeed = mavlink_msg_asl_high_latency_get_groundspeed(msg);
    asl_high_latency->gps_nsat = mavlink_msg_asl_high_latency_get_gps_nsat(msg);
    asl_high_latency->gps_fix_type = mavlink_msg_asl_high_latency_get_gps_fix_type(msg);
    asl_high_latency->temperature_air = mavlink_msg_asl_high_latency_get_temperature_air(msg);
    asl_high_latency->failsafe = mavlink_msg_asl_high_latency_get_failsafe(msg);
    asl_high_latency->v_avg_mppt0 = mavlink_msg_asl_high_latency_get_v_avg_mppt0(msg);
    asl_high_latency->v_avg_mppt1 = mavlink_msg_asl_high_latency_get_v_avg_mppt1(msg);
    asl_high_latency->v_avg_mppt2 = mavlink_msg_asl_high_latency_get_v_avg_mppt2(msg);
    asl_high_latency->state_batmon0 = mavlink_msg_asl_high_latency_get_state_batmon0(msg);
    asl_high_latency->state_batmon1 = mavlink_msg_asl_high_latency_get_state_batmon1(msg);
    asl_high_latency->state_batmon2 = mavlink_msg_asl_high_latency_get_state_batmon2(msg);
    asl_high_latency->p_avg_bat = mavlink_msg_asl_high_latency_get_p_avg_bat(msg);
    asl_high_latency->v_avg_bat0 = mavlink_msg_asl_high_latency_get_v_avg_bat0(msg);
    asl_high_latency->v_avg_bat1 = mavlink_msg_asl_high_latency_get_v_avg_bat1(msg);
    asl_high_latency->v_avg_bat2 = mavlink_msg_asl_high_latency_get_v_avg_bat2(msg);
    asl_high_latency->status_pwrbrd = mavlink_msg_asl_high_latency_get_status_pwrbrd(msg);
    asl_high_latency->p_out = mavlink_msg_asl_high_latency_get_p_out(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN? msg->len : MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN;
        memset(asl_high_latency, 0, MAVLINK_MSG_ID_ASL_HIGH_LATENCY_LEN);
    memcpy(asl_high_latency, _MAV_PAYLOAD(msg), len);
#endif
}
