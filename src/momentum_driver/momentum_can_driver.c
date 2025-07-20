/*******************************************************************************
 * @file momentum_can_driver.c
 * @brief Momentum driver for CAN based communication.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "momentum_can_driver.h"
#include "math.h"

/** Public functions. *********************************************************/

uint32_t uint_to_raw(uint32_t physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < (uint32_t)signal->min_value)
    physical_value = (uint32_t)signal->min_value;
  if (physical_value > (uint32_t)signal->max_value)
    physical_value = (uint32_t)signal->max_value;

  // Normalize into raw units (cast to float just for the calculation).
  float normalized = ((float)physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)roundf(normalized);

  return raw;
}

uint32_t float_to_raw(float physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < signal->min_value)
    physical_value = signal->min_value;
  if (physical_value > signal->max_value)
    physical_value = signal->max_value;

  // Normalize into raw units.
  float normalized = (physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)roundf(normalized);

  return raw;
}

uint32_t double_to_raw(double physical_value, const can_signal_t *signal) {
  // Clamp physical value into [min, max].
  if (physical_value < signal->min_value)
    physical_value = signal->min_value;
  if (physical_value > signal->max_value)
    physical_value = signal->max_value;

  // Normalize into raw units.
  double normalized = (physical_value - signal->offset) / signal->scale;

  // Round to nearest uint32_t.
  uint32_t raw = (uint32_t)llround(normalized);

  return raw;
}

void pack_signal_raw32(const can_signal_t *signal, uint8_t *data,
                       uint32_t raw_value) {
  // Mask off any bits above bit_length.
  if (signal->bit_length < 32) {
    raw_value &= ((1UL << signal->bit_length) - 1UL);
  }

  // Pack each bit into data[].
  for (uint32_t bit = 0; bit < signal->bit_length; ++bit) {
    // Determine the absolute bit position in the 64‑bit CAN payload.
    uint32_t bit_pos = (signal->byte_order == CAN_LITTLE_ENDIAN)
                           ? (signal->start_bit + bit)  // Intel/little‑endian.
                           : (signal->start_bit - bit); // Motorola/big‑endian.

    uint32_t byte_index = bit_pos / 8;
    uint32_t bit_index = bit_pos % 8;

    // Extract the next raw bit, then OR it into the right place.
    uint8_t raw_bit = (raw_value >> bit) & 0x1U;
    data[byte_index] |= (raw_bit << bit_index);
  }
}

float decode_signal(const can_signal_t *signal, const uint8_t *data) {
  uint64_t raw_value = 0;

  // Extract raw bits from the CAN message payload.
  for (int i = 0; i < signal->bit_length; i++) {
    int bit_position = signal->start_bit + i;
    int byte_index = bit_position / 8;
    int bit_index = bit_position % 8;

    // Use the enumerated constant for byte order.
    if (signal->byte_order == CAN_BIG_ENDIAN) {
      raw_value |= ((data[byte_index] >> (7 - bit_index)) & 0x1)
                   << (signal->bit_length - 1 - i);
    } else { // CAN_LITTLE_ENDIAN.
      raw_value |= ((data[byte_index] >> bit_index) & 0x1) << i;
    }
  }

  // Convert raw value to physical value by applying scale and offset.
  return ((float)raw_value * signal->scale) + signal->offset;
}
