#ifndef W600_TEMPERATURE_H
#define W600_TEMPERATURE_H
#include <stdint.h>

enum { W600_TEMPERATURE_READINGS = 5 };
enum w600_temperature_status {
    W600_TEMPERATURE_OK,
    W600_TEMPERATURE_INVALID_INPUT,
    W600_TEMPERATURE_SENSOR_RANGE
};
struct w600_temperature_result {
    uint16_t filtered_adc;
    int16_t sensor_decic;
    int16_t adjusted_decic;
};

/* Five12-bit ADC readings: sum all five, remove one minimum and one maximum,
 * average the remaining three. ADC acquisition/bias timing are separate.
 * offset_decic is the explicit configurable -50..50 offset, not B954's reporting
 * compensation. No B954 correction or external-temperature selection is applied.
 * Pointers must reference five readable samples and one writable result. NULL,
 * invalid input or derived ADC outside101..4031 fails without changing result. */
enum w600_temperature_status w600_temperature_convert(
    const uint16_t readings[W600_TEMPERATURE_READINGS], int16_t offset_decic,
    struct w600_temperature_result *result);
#endif
