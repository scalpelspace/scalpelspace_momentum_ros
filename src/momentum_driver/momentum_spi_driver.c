/*******************************************************************************
 * @file momentum_spi_driver.c
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_spi_driver.h"

/** Private functions. ********************************************************/

static inline uint8_t *pack_uint_8(uint8_t *p, uint8_t v) {
  *p++ = v;
  return p;
}

static inline uint8_t *pack_char_8(uint8_t *p, char c) {
  *p++ = (uint8_t)c;
  return p;
}

static inline uint8_t *pack_float_32(uint8_t *p, float v) {
  memcpy(p, &v, 4);
  return p + 4;
}

static inline uint8_t *pack_double_64(uint8_t *p, double v) {
  memcpy(p, &v, 8);
  return p + 8;
}

static inline const uint8_t *unpack_uint_8(const uint8_t *p, uint8_t *v) {
  *v = *p;
  return p + 1;
}

static inline const uint8_t *unpack_char_8(const uint8_t *p, char *c) {
  *c = (char)*p;
  return p + 1;
}

static inline const uint8_t *unpack_float_32(const uint8_t *p, float *f) {
  memcpy(f, p, 4);
  return p + 4;
}

static inline const uint8_t *unpack_double_64(const uint8_t *p, double *d) {
  memcpy(d, p, 8);
  return p + 8;
}

static inline uint8_t update_payload_length(momentum_frame_t *f,
                                            const uint8_t *start,
                                            const uint8_t *end) {
  uint8_t len = (uint8_t)(end - start);
  f->length = len;
  return len;
}

/** Public functions. *********************************************************/

uint16_t crc16_ccitt(uint16_t crc, const uint8_t *buf, size_t len) {
  while (len--) {
    crc ^= (uint16_t)(*buf++) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void build_crc(momentum_frame_t *f) {
  // Check: frame_type(1) + sequence(1) + length(1) + data[length].
  size_t n = 3 + f->length;
  // `&f->frame_type` is the address of first byte to CRC.
  f->crc = crc16_ccitt(MOMENTUM_CRC_INITIAL, &f->frame_type, n);
}

bool verify_crc(const momentum_frame_t *f) {
  // Check: frame_type(1) + sequence(1) + length(1) + data[length].
  size_t n = 3 + f->length;
  uint16_t crc = crc16_ccitt(MOMENTUM_CRC_INITIAL, &f->frame_type, n);
  return (crc == f->crc);
}

uint8_t build_version_payload(momentum_frame_t *f, version_t *v) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, v->major);
  p = pack_uint_8(p, v->minor);
  p = pack_uint_8(p, v->patch);
  p = pack_char_8(p, v->identifier);
  return update_payload_length(f, start, p);
}

uint8_t parse_version_payload(const momentum_frame_t *f, version_t *v) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &v->major);
  p = unpack_uint_8(p, &v->minor);
  p = unpack_uint_8(p, &v->patch);
  p = unpack_char_8(p, &v->identifier);
  return (uint8_t)(p - f->payload);
}

uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->quaternion_i);
  p = pack_float_32(p, s->quaternion_j);
  p = pack_float_32(p, s->quaternion_k);
  p = pack_float_32(p, s->quaternion_real);
  p = pack_float_32(p, s->quaternion_accuracy_rad);
  p = pack_float_32(p, s->quaternion_accuracy_deg);
  return update_payload_length(f, start, p);
}

uint8_t parse_quaternion_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->quaternion_i);
  p = unpack_float_32(p, &s->quaternion_j);
  p = unpack_float_32(p, &s->quaternion_k);
  p = unpack_float_32(p, &s->quaternion_real);
  p = unpack_float_32(p, &s->quaternion_accuracy_rad);
  p = unpack_float_32(p, &s->quaternion_accuracy_deg);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->gyro_x);
  p = pack_float_32(p, s->gyro_y);
  p = pack_float_32(p, s->gyro_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_gyro_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->gyro_x);
  p = unpack_float_32(p, &s->gyro_y);
  p = unpack_float_32(p, &s->gyro_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->accel_x);
  p = pack_float_32(p, s->accel_y);
  p = pack_float_32(p, s->accel_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_accel_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->accel_x);
  p = unpack_float_32(p, &s->accel_y);
  p = unpack_float_32(p, &s->accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->lin_accel_x);
  p = pack_float_32(p, s->lin_accel_y);
  p = pack_float_32(p, s->lin_accel_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_lin_accel_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->lin_accel_x);
  p = unpack_float_32(p, &s->lin_accel_y);
  p = unpack_float_32(p, &s->lin_accel_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->gravity_x);
  p = pack_float_32(p, s->gravity_y);
  p = pack_float_32(p, s->gravity_z);
  return update_payload_length(f, start, p);
}

uint8_t parse_gravity_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->gravity_x);
  p = unpack_float_32(p, &s->gravity_y);
  p = unpack_float_32(p, &s->gravity_z);
  return (uint8_t)(p - f->payload);
}

uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->temperature);
  p = pack_float_32(p, s->pressure);
  return update_payload_length(f, start, p);
}

uint8_t parse_pressure_temp_payload(const momentum_frame_t *f,
                                    sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->temperature);
  p = unpack_float_32(p, &s->pressure);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->hour);
  p = pack_uint_8(p, s->minute);
  p = pack_uint_8(p, s->second);
  p = pack_uint_8(p, s->day);
  p = pack_uint_8(p, s->month);
  p = pack_uint_8(p, s->year);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_datetime_payload(const momentum_frame_t *f,
                                   sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &s->hour);
  p = unpack_uint_8(p, &s->minute);
  p = unpack_uint_8(p, &s->second);
  p = unpack_uint_8(p, &s->day);
  p = unpack_uint_8(p, &s->month);
  p = unpack_uint_8(p, &s->year);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->latitude);
  p = pack_char_8(p, s->latitude_dir);
  p = pack_float_32(p, s->longitude);
  p = pack_char_8(p, s->longitude_dir);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_coord_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->latitude);
  p = unpack_char_8(p, &s->latitude_dir);
  p = unpack_float_32(p, &s->longitude);
  p = unpack_char_8(p, &s->longitude_dir);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_altitude_speed_payload(momentum_frame_t *f,
                                         sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->altitude);
  p = pack_float_32(p, s->geoid_sep);
  p = pack_float_32(p, s->ground_speed);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_altitude_speed_payload(const momentum_frame_t *f,
                                         sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->altitude);
  p = unpack_float_32(p, &s->geoid_sep);
  p = unpack_float_32(p, &s->ground_speed);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_heading_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_float_32(p, s->ground_course);
  p = pack_float_32(p, s->magnetic_var);
  p = pack_char_8(p, s->magnetic_var_dir);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_heading_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_float_32(p, &s->ground_course);
  p = unpack_float_32(p, &s->magnetic_var);
  p = unpack_char_8(p, &s->magnetic_var_dir);
  return (uint8_t)(p - f->payload);
}

uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, s->gps_position_fix);
  p = pack_uint_8(p, s->satellites);
  p = pack_float_32(p, s->hdop);
  return update_payload_length(f, start, p);
}

uint8_t parse_gps_stats_payload(const momentum_frame_t *f, sensor_data_t *s) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &s->gps_position_fix);
  p = unpack_uint_8(p, &s->satellites);
  p = unpack_float_32(p, &s->hdop);
  return (uint8_t)(p - f->payload);
}

uint8_t build_led_payload(momentum_frame_t *f, led_data_t *l) {
  uint8_t *start = f->payload;
  uint8_t *p = f->payload;
  p = pack_uint_8(p, l->index);
  p = pack_uint_8(p, l->r);
  p = pack_uint_8(p, l->g);
  p = pack_uint_8(p, l->b);
  return update_payload_length(f, start, p);
}

uint8_t parse_led_payload(const momentum_frame_t *f, led_data_t *l) {
  const uint8_t *p = f->payload;
  p = unpack_uint_8(p, &l->index);
  p = unpack_uint_8(p, &l->r);
  p = unpack_uint_8(p, &l->g);
  p = unpack_uint_8(p, &l->b);
  return (uint8_t)(p - f->payload);
}

momentum_status_t parse_momentum_response_frame(const momentum_frame_t *f,
                                                sensor_data_t *s,
                                                version_t *v) {
  if (f->start_of_frame != MOMENTUM_START_OF_RESPONSE_FRAME ||
      f->length > MOMENTUM_MAX_DATA_SIZE)
    return MOMENTUM_ERROR_BAD_FRAME;

  if (!verify_crc(f))
    return MOMENTUM_ERROR_CRC;

  switch (f->frame_type) {
  case MOMENTUM_FRAME_TYPE_VERSION:
    parse_version_payload(f, v);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_QUAT:
    parse_quaternion_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GYRO:
    parse_gyro_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_ACCEL:
    parse_accel_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_LINACCEL:
    parse_lin_accel_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_IMU_GRAV:
    parse_gravity_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_BAR_ENV:
    parse_pressure_temp_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_DATETIME:
    parse_gps_datetime_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_COORD:
    parse_gps_coord_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED:
    parse_gps_altitude_speed_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_HEAD:
    parse_gps_heading_payload(f, s);
    break;
  case MOMENTUM_FRAME_TYPE_GPS_STATS:
    parse_gps_stats_payload(f, s);
    break;
  default:
    return MOMENTUM_ERROR_FRAME_TYPE;
  }

  return MOMENTUM_OK; // Successful frame unpacking.
}
