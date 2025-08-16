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
  // Mask raw_value to bit_length (and avoid UB on 32).
  if (signal->bit_length == 0)
    return;
  if (signal->bit_length < 32) {
    raw_value &= (uint32_t)((1u << signal->bit_length) - 1u);
  }

  for (uint32_t i = 0; i < signal->bit_length; ++i) {
    uint32_t byte_index, bit_index; // bit_index: 0 = LSB of byte.

    if (signal->byte_order == CAN_LITTLE_ENDIAN) {
      // Intel: LSB-first across payload.
      const uint32_t bit_pos = signal->start_bit + i;
      byte_index = bit_pos / 8;
      bit_index = bit_pos % 8;
    } else {
      // Motorola: MSB-first within a byte, then previous byte.
      const uint32_t start_byte = signal->start_bit / 8;
      const uint32_t start_bit_in_byte = 7u - (signal->start_bit % 8u); // 0..7.
      const uint32_t msb_walk = start_bit_in_byte + i; // Walk in MSB space.
      byte_index = start_byte - (msb_walk / 8u);
      bit_index = 7u - (msb_walk % 8u);
    }

    const uint8_t bit = (uint8_t)((raw_value >> i) & 0x1u);
    // Clear then set to avoid leaving stale bits if buffer is reused.
    data[byte_index] =
        (uint8_t)((data[byte_index] & ~(1u << bit_index)) | (bit << bit_index));
  }
}

float decode_signal(const can_signal_t *signal, const uint8_t *data) {
  if (signal->bit_length == 0)
    return 0.0f;

  uint32_t raw = 0;

  for (uint32_t i = 0; i < signal->bit_length; ++i) {
    uint32_t byte_index, bit_index; // bit_index: 0 = LSB of byte.

    if (signal->byte_order == CAN_LITTLE_ENDIAN) {
      // Intel: LSB-first across payload.
      const uint32_t bit_pos = signal->start_bit + i;
      byte_index = bit_pos / 8;
      bit_index = bit_pos % 8;
    } else {
      // Motorola: MSB-first within a byte, then previous byte.
      const uint32_t start_byte = signal->start_bit / 8;
      const uint32_t start_bit_in_byte = 7u - (signal->start_bit % 8u); // 0..7.
      const uint32_t msb_walk = start_bit_in_byte + i; // Walk in MSB space.
      byte_index = start_byte - (msb_walk / 8u);
      bit_index = 7u - (msb_walk % 8u);
    }

    const uint64_t bit = (uint64_t)((data[byte_index] >> bit_index) & 0x1u);
    raw |= (bit << i); // Assemble raw LSB-first.
  }

  // Apply scale and offset (raw -> physical).
  return (float)((double)raw * (double)signal->scale + (double)signal->offset);
}
