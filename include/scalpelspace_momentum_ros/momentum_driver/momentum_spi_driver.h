/*******************************************************************************
 * @file momentum_spi_driver.h
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

#ifndef MOMENTUM__SPI_DRIVER_H
#define MOMENTUM__SPI_DRIVER_H

/** Includes. *****************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** Definitions. **************************************************************/

#define MOMENTUM_MAX_DATA_SIZE 32

#define MOMENTUM_CRC_INITIAL 0xFFFF

#define MOMENTUM_START_OF_COMMAND_FRAME 0x00
#define MOMENTUM_START_OF_REQUEST_FRAME 0x40
#define MOMENTUM_START_OF_RESPONSE_FRAME 0x41

// Master request (no response from Momentum) frame types.
// Starts after MOMENTUM_START_OF_COMMAND_FRAME.
// Starts before MOMENTUM_START_OF_REQUEST_FRAME.
#define MOMENTUM_FRAME_TYPE_RESET 0x06
#define MOMENTUM_FRAME_TYPE_LED 0x10

// Master request and response from Momentum frame types.
// Starts after MOMENTUM_START_OF_RESPONSE_FRAME.
// Starts before 0xFF.
#define MOMENTUM_FRAME_TYPE_VERSION 0x42

#define MOMENTUM_FRAME_TYPE_IMU_ACCEL 0x71 // SH-2 ID + 0x70.
#define MOMENTUM_FRAME_TYPE_IMU_GYRO 0x72
#define MOMENTUM_FRAME_TYPE_IMU_LINACCEL 0x74
#define MOMENTUM_FRAME_TYPE_IMU_QUAT 0x75
#define MOMENTUM_FRAME_TYPE_IMU_GRAV 0x76

#define MOMENTUM_FRAME_TYPE_BAR_ENV 0xA0

#define MOMENTUM_FRAME_TYPE_GPS_DATETIME 0xB0
#define MOMENTUM_FRAME_TYPE_GPS_COORD 0xB1
#define MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED 0xB2
#define MOMENTUM_FRAME_TYPE_GPS_HEAD 0xB3
#define MOMENTUM_FRAME_TYPE_GPS_STATS 0xB4

/** Public types. *************************************************************/

/**
 * @brief Momentum sensor hub SPI communication frame.
 */
typedef struct __attribute__((packed)) {
  uint8_t start_of_frame;                  // Header: start of frame (SOF).
  uint8_t frame_type;                      // Header: frame type identifier.
  uint8_t sequence;                        // Header: roll-over counter.
  uint8_t length;                          // Header: Number of data bytes.
  uint8_t payload[MOMENTUM_MAX_DATA_SIZE]; // Data payload.
  uint16_t crc;                            // CRC-16 of frame_type...data[-1].
} momentum_frame_t;

/**
 * @brief Momentum sensor hub SPI communication frame processing status.
 */
typedef enum {
  MOMENTUM_OK = 0,
  MOMENTUM_ERROR_CRC = 1,        // Invalid CRC.
  MOMENTUM_ERROR_FRAME_TYPE = 2, // Invalid frame type.
  MOMENTUM_ERROR_BAD_FRAME = 3,  // Invalid start of frame (SOF) or length.
} momentum_status_t;

typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  char identifier;
} version_t;

/**
 * @brief Momentum sensor hub sensor data.
 */
typedef struct {
  float quaternion_i;
  float quaternion_j;
  float quaternion_k;
  float quaternion_real;
  float quaternion_accuracy_rad;
  float quaternion_accuracy_deg;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
  float lin_accel_x;
  float lin_accel_y;
  float lin_accel_z;
  float gravity_x;
  float gravity_y;
  float gravity_z;
  float temperature;
  float pressure;
  uint8_t gps_position_fix;
  uint8_t year;          // RTC date, year.
  uint8_t month;         // RTC date, month.
  uint8_t day;           // RTC date, day.
  uint8_t hour;          // RTC time, hour.
  uint8_t minute;        // RTC time, minute.
  uint8_t second;        // RTC time, second.
  float latitude;        // Latitude in decimal degrees.
  char latitude_dir;     // Latitude Direction (N/S).
  float longitude;       // Longitude in decimal degrees.
  char longitude_dir;    // Longitude Direction (E/W).
  float altitude;        // Altitude in meters.
  float geoid_sep;       // Geoidal Separation.
  float ground_speed;    // Speed over the ground in knots.
  float ground_course;   // Course over ground in degrees.
  float magnetic_var;    // Magnetic variation in degrees.
  char magnetic_var_dir; // Magnetic variation direction (E/W).
  uint8_t satellites;    // Number of Satellites.
  float hdop;            // Horizontal Dilution of Precision (HDOP).
} sensor_data_t;

typedef struct {
  uint8_t index;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_data_t;

/** Public functions. *********************************************************/

/**
 * @brief Compute CRC-16-CCITT over a byte buffer.
 *
 * @param crc Initial CRC value (usually 0xFFFF).
 * @param buf Pointer to the data bytes to checksum.
 * @param len Number of bytes in buf.
 *
 * @return The updated CRC16.
 */
uint16_t crc16_ccitt(uint16_t crc, const uint8_t *buf, size_t len);

/**
 * @brief Build and append CRC to a momentum_frame_t.
 *
 * Calculates the CRC-16-CCITT over the frame header and payload, then writes
 * the resulting 16-bit CRC into f->crc.
 *
 * @param f Pointer to the frame whose CRC field will be updated.
 */
void build_crc(momentum_frame_t *f);

/**
 * @brief Verify the CRC of a received frame.
 *
 * Recomputes CRC-16-CCITT over the received frame_type, sequence, length,
 * and payload bytes, then compares against the 16-bit CRC field in the frame.
 *
 * @param f Pointer to the received frame.
 *
 * @return true if the computed CRC matches f->crc, false otherwise.
 */
bool verify_crc(const momentum_frame_t *f);

/**
 * @brief Pack version data into the frame payload.
 *
 * @param f Pointer to the frame to populate.
 * @param v Pointer to the version_t containing version values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_version_payload(momentum_frame_t *f, version_t *v);

/**
 * @brief Unpack version data from the frame payload.
 *
 * @param f Pointer to the frame to read.
 * @param v Pointer to the version_t to update version values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_version_payload(const momentum_frame_t *f, version_t *v);

/**
 * @brief Pack quaternion data into the frame payload.
 *
 * Serializes quaternion components (i, j, k, real) and quaternion accuracy (in
 * radians and degrees) from the provided sensor data into the frame payload.
 * Updates f->length to reflect the number of bytes written.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source quaternion values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack quaternion data from the frame payload.
 *
 * Deserializes quaternion components (i, j, k, real) and quaternion accuracy
 * (in radians and degrees) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update quaternion values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_quaternion_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack gyroscope data into the frame payload.
 *
 * Serializes gyroscope readings (x, y, z) from the provided sensor data into
 * the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source gyroscope values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack gyroscope data from the frame payload.
 *
 * Deserializes gyroscope readings (x, y, z) from the provided frame payload
 * into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update gyroscope values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gyro_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack accelerometer data into the frame payload.
 *
 * Serializes accelerometer readings (x, y, z) from the provided sensor data
 * into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source accelerometer values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack accelerometer data from the frame payload.
 *
 * Deserializes accelerometer readings (x, y, z) from the provided frame payload
 * into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update accelerometer values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_accel_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack linear acceleration data into the frame payload.
 *
 * Serializes linear acceleration readings (x, y, z) from the provided sensor
 * data into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source linear acceleration
 *          values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack linear acceleration data from the frame payload.
 *
 * Deserializes linear acceleration readings (x, y, z) from the provided frame
 * payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update linear acceleration values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_lin_accel_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack gravity vector data into the frame payload.
 *
 * Serializes gravity vector components (x, y, z) from the provided sensor data
 * into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source gravity values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack gravity vector data from the frame payload.
 *
 * Deserializes gravity vector components (x, y, z) from the provided frame
 * payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update gravity values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gravity_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack barometer temperature and pressure data into the frame payload.
 *
 * Serializes barometric temperature and pressure readings from the provided
 * sensor data into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source temperature and
 *          pressure values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack barometer temperature and pressure data from the frame payload.
 *
 * Deserializes barometric temperature and pressure readings from the provided
 * frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update temperature and pressure
 *          values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_pressure_temp_payload(const momentum_frame_t *f,
                                    sensor_data_t *s);

/**
 * @brief Pack GPS date and time data into the frame payload.
 *
 * Serializes GPS date and time fields (hour, minute, second, day, month, year)
 * from the provided sensor data into the frame payload. Updates f->length
 * accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS date/time values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS date and time data from the frame payload.
 *
 * Deserializes GPS date and time fields (hour, minute, second, day, month,
 * year) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS date/time values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_datetime_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS coordinate data into the frame payload.
 *
 * Serializes GPS position fields (latitude, latitude direction, longitude,
 * longitude direction) from the provided sensor data into the frame payload.
 * Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS coordinates.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS coordinate data from the frame payload.
 *
 * Deserializes GPS position fields (latitude, latitude direction, longitude,
 * longitude direction) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS coordinates.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_coord_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS altitude and speed data into the frame payload.
 *
 * Serializes GPS altitude fields (altitude, geoidal separation) and speed
 * (speed knots) from the provided sensor data into the frame payload. Updates
 * f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS altitude and
 *          speed values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_altitude_speed_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS altitude and speed data from the frame payload.
 *
 * Deserializes GPS altitude fields (altitude, geoidal separation) and speed
 * (speed knots) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS altitude and speed
 *          values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_altitude_speed_payload(const momentum_frame_t *f,
                                         sensor_data_t *s);

/**
 * @brief Pack GPS heading data into the frame payload.
 *
 * Serializes GPS heading fields (course, magnetic variation, magnetic variation
 * direction) from the provided sensor data into the frame payload. Updates
 * f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS heading values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_heading_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS heading data from the frame payload.
 *
 * Deserializes GPS heading fields (course, magnetic variation, magnetic
 * variation direction) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS heading values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_heading_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS status and statistics into the frame payload.
 *
 * Serializes GPS status fields (position fix type, number of satellites, HDOP)
 * from the provided sensor data into the frame payload. Updates f->length
 * accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS status values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS status and statistics from the frame payload.
 *
 * Deserializes GPS status fields (position fix type, number of satellites,
 * HDOP) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS status values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_stats_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack LED data into the frame payload.
 *
 * @param f Pointer to the frame to populate.
 * @param l Pointer to the led_data_t to populate.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t build_led_payload(momentum_frame_t *f, led_data_t *l);

/**
 * @brief Unpack LED data from the frame payload.
 *
 * @param f Pointer to the frame to read.
 * @param l Pointer to the led_data_t to read.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_led_payload(const momentum_frame_t *f, led_data_t *l);

/**
 * @brief Top-level parser for any incoming request response momentum_frame_t.
 *
 * @param f Pointer to received, packed frame.
 * @param s Destination sensor_data_t to populate on success.
 * @param v Destination version_t to populate on success.
 *
 * @return  MOMENTUM_OK on success, otherwise an error code:
 *          - MOMENTUM_ERROR_CRC if CRC mismatch.
 *          - MOMENTUM_ERROR_FRAME_TYPE if unknown frame_type.
 *          - MOMENTUM_ERROR_BAD_FRAME if SOF/length bad.
 */
momentum_status_t parse_momentum_response_frame(const momentum_frame_t *f,
                                                sensor_data_t *s, version_t *v);

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif
