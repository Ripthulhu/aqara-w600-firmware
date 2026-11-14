#include "w600_temperature.h"

static int16_t sensor_temperature(uint32_t adc)
{
    /* Stock FLASH6320104b,12 little-endian ADC knots at0800E77A..0800E791. */
    static const uint16_t knots[]={1094,1273,1462,1657,1853,2048,2237,2418,2588,2747,2892,3025};
    int32_t decic=0;
    if (adc>=3025) decic=545;
    else if (adc>1094) {
        unsigned upper=1;
        while (adc>=knots[upper]) ++upper;
        uint32_t interpolated=(adc-knots[upper-1])*50u/(knots[upper]-knots[upper-1]);
        decic=(int32_t)((upper-1)*50u+interpolated);
        decic=decic<5 ? 0 : decic-5;
    }
    return (int16_t)decic;
}

enum w600_temperature_status w600_temperature_convert(
    const uint16_t readings[W600_TEMPERATURE_READINGS], int16_t offset_decic,
    struct w600_temperature_result *result)
{
    if (!readings || !result || offset_decic < -50 || offset_decic > 50)
        return W600_TEMPERATURE_INVALID_INPUT;
    uint32_t sum=0, minimum=4095, maximum=0;
    for (unsigned i=0;i<W600_TEMPERATURE_READINGS;++i) {
        uint32_t raw=readings[i];
        if (raw>4095) return W600_TEMPERATURE_INVALID_INPUT;
        if (raw<minimum) minimum=raw;
        if (raw>maximum) maximum=raw;
        sum+=raw;
    }
    /* Don't copy stock's asymmetric first-read/min-max treatment. */
    uint32_t adc=(sum-minimum-maximum)/3u;
    if (adc<101 || adc>4031) return W600_TEMPERATURE_SENSOR_RANGE;
    int16_t decic=sensor_temperature(adc);
    *result=(struct w600_temperature_result){(uint16_t)adc,decic,
                                            (int16_t)(decic+offset_decic)};
    return W600_TEMPERATURE_OK;
}
