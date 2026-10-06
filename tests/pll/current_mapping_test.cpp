#include "../../user_lib/drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
    ADC_TypeDef registers{2048,2048};ADC_HandleTypeDef adc{&registers};
    stm32_two_shunt_current_config config{&adc,.001f,.001f,-1,-1,two_shunt_phase_mapping::CB};
    stm32_two_shunt_current_sensor sensor(config);phase_current_sample sample{};
    assert(sensor.init()==foc_result::OK);
    assert(sensor.calibrate_task(4)==foc_result::CALIBRATING);
    for(int n=0;n<4;n++)assert(sensor.read_conversion_from_isr(n*50,sample)==foc_result::CALIBRATING);
    assert(sensor.calibrate_task(4)==foc_result::OK);
    // A unit current vector at six independent stator orientations. ADC0
    // senses -C and ADC1 senses -B before the configured -1 polarities.
    for(int k=0;k<6;k++){
        float theta=k*3.14159265359f/3;
        float a=cosf(theta),b=cosf(theta-2.09439510239f),c=cosf(theta+2.09439510239f);
        registers.JDR1=(uint32_t)lroundf(2048-c*1000);
        registers.JDR2=(uint32_t)lroundf(2048-b*1000);
        assert(sensor.read_conversion_from_isr(1000+k*50,sample)==foc_result::OK);
        assert(fabsf(sample.current_a-a)<.0011f && fabsf(sample.current_b-b)<.0011f && fabsf(sample.current_c-c)<.0011f);
        float beta=(sample.current_a+2*sample.current_b)/sqrtf(3);
        assert(fabsf(beta-sinf(theta))<.0011f);
    }
    puts("PASS: production ADC offset calibration, C/B reconstruction and six stator current vectors");
}
