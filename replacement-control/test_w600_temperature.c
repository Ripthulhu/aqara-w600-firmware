#include "w600_temperature.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned cases;
static int compare(const void *a,const void *b)
{
    return (int)*(const uint16_t *)a-(int)*(const uint16_t *)b;
}
/* Independent corrected-filter oracle: sort the five inputs, average the middle
 * three, then apply stock's explicit intervals rather than the knot search. */
static enum w600_temperature_status oracle(const uint16_t *readings,int16_t offset,
                                           struct w600_temperature_result *out)
{
    static const struct { uint16_t lo,hi; } intervals[]={
        {1094,1273},{1273,1462},{1462,1657},{1657,1853},{1853,2048},{2048,2237},
        {2237,2418},{2418,2588},{2588,2747},{2747,2892},{2892,3025}};
    if (!readings || !out || offset < -50 || offset > 50)
        return W600_TEMPERATURE_INVALID_INPUT;
    uint16_t sorted[5];
    for (unsigned i=0;i<5;++i) {
        if (readings[i]>4095) return W600_TEMPERATURE_INVALID_INPUT;
        sorted[i]=readings[i];
    }
    qsort(sorted,5,sizeof *sorted,compare);
    int filtered=(sorted[1]+sorted[2]+sorted[3])/3;
    if (filtered<101 || filtered>4031) return W600_TEMPERATURE_SENSOR_RANGE;
    int temperature=filtered>=3025 ? 545 : 0;
    for (unsigned i=0;i<11;++i) if (filtered>=intervals[i].lo && filtered<intervals[i].hi) {
        int value=(filtered-intervals[i].lo)*50/(intervals[i].hi-intervals[i].lo)+(int)i*50;
        temperature=value<=4 ? 0 : value-5;
        break;
    }
    *out=(struct w600_temperature_result){(uint16_t)filtered,(int16_t)temperature,
                                         (int16_t)(temperature+offset)};
    return W600_TEMPERATURE_OK;
}
static void check(const uint16_t readings[5],int16_t offset)
{
    struct w600_temperature_result actual={0xabcd,1234,-2345}, expected=actual;
    enum w600_temperature_status status=w600_temperature_convert(readings,offset,&actual);
    assert(status==oracle(readings,offset,&expected));
    assert(!memcmp(&actual,&expected,sizeof actual));
    ++cases;
}
int main(void)
{
    uint16_t readings[5];
    for (unsigned raw=0;raw<=4095;++raw) {
        for (unsigned i=0;i<5;++i) readings[i]=(uint16_t)raw;
        for (int offset=-50;offset<=50;++offset) check(readings,(int16_t)offset);
    }
    const uint16_t anchors[]={0,101,1094,1273,2000,3025,4031,4095};
    for (unsigned a=0;a<sizeof anchors/sizeof *anchors;++a) for (unsigned position=0;position<5;++position) {
        for (unsigned raw=0;raw<=4095;++raw) {
            for (unsigned i=0;i<5;++i) readings[i]=anchors[a];
            readings[position]=(uint16_t)raw;
            check(readings,0); check(readings,position&1 ? -50 : 50);
        }
    }
    uint32_t random=0x6320104b;
    for (unsigned n=0;n<10000;++n) {
        for (unsigned i=0;i<5;++i) { random=random*1664525u+1013904223u; readings[i]=(uint16_t)(random>>20); }
        check(readings,(int16_t)((int)(n%101)-50));
    }
    for (unsigned i=0;i<5;++i) readings[i]=2048;
    struct w600_temperature_result result={0};
    assert(w600_temperature_convert(readings,0,&result)==W600_TEMPERATURE_OK);
    assert(result.filtered_adc==2048 && result.sensor_decic==245 && result.adjusted_decic==245);
    assert(w600_temperature_convert(readings,50,&result)==W600_TEMPERATURE_OK && result.adjusted_decic==295);
    for (unsigned i=0;i<5;++i) readings[i]=3025;
    assert(w600_temperature_convert(readings,50,&result)==W600_TEMPERATURE_OK && result.adjusted_decic==595);
    /* All sample positions are equivalent; either outlier must be discarded. */
    const uint16_t unusual[]={0,2000,2000,2000,2000};
    assert(w600_temperature_convert(unusual,0,&result)==W600_TEMPERATURE_OK && result.filtered_adc==2000);
    const uint16_t high_outlier[]={4095,2000,2000,2000,2000};
    assert(w600_temperature_convert(high_outlier,0,&result)==W600_TEMPERATURE_OK && result.filtered_adc==2000);
    const uint16_t mixed[]={4095,1999,2000,2001,0};
    assert(w600_temperature_convert(mixed,0,&result)==W600_TEMPERATURE_OK && result.filtered_adc==2000);
    const uint16_t truncation[]={0,2000,2001,2001,4095};
    assert(w600_temperature_convert(truncation,0,&result)==W600_TEMPERATURE_OK && result.filtered_adc==2000);
    const uint16_t cold[]={4095,0,0,0,0}; check(cold,0);
    for (unsigned p=0;p<5;++p) {
        for (unsigned i=0;i<5;++i) readings[i]=2048;
        readings[p]=4096; check(readings,0);
        readings[p]=UINT16_MAX; check(readings,0);
    }
    for (unsigned i=0;i<5;++i) readings[i]=2048;
    check(readings,-51); check(readings,51); check(readings,INT16_MIN); check(readings,INT16_MAX);
    struct w600_temperature_result preserved={0xabcd,1234,-2345}; result=preserved;
    assert(w600_temperature_convert(NULL,0,&result)==W600_TEMPERATURE_INVALID_INPUT);
    assert(!memcmp(&result,&preserved,sizeof result));
    assert(w600_temperature_convert(readings,0,NULL)==W600_TEMPERATURE_INVALID_INPUT);
    printf("PASS %u temperature cases: ADC domain/all offsets, varying each input, random/noisy batches, corrected trimmed mean, stock interpolation, range/input rejection; no ADC hardware tested\n",cases);
}
