#include <stdint.h>
#include <float.h>

typedef struct {
  uint16_t type; // type of the message (always 0x0001)
  uint32_t mcu_id_1; // 1st part of the id of the mcu
  uint32_t mcu_id_2; // 2nd part of the id of the mcu
  uint32_t mcu_id_3; // 3rd part of the id of the mcu
  uint32_t message_index; // index of the message (counted up since last reset)
  uint16_t sensor_id; // id of the sensor
  float temperature; // temperature in degrees celcius
  float humidity; // humidity in %
  uint32_t _rng; // random number for proper alignment
} message_0001_t;
