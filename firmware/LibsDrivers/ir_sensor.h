#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include <gpio.h>
#include <biquad_filter.h>

#define IR_SENSORS_COUNT        ((unsigned int)4)


/*
    IR left         :   PB1     ADC1, ADC2, IN9
    IR front left   :   PC5     ADC1, ADC2, IN15
    IR front right  :   PC4     ADC1, ADC2, IN14
    IR right        :   PB0     ADC1, ADC2, IN8
*/

class IRSensor
{
    public:
        void init();

        void callback();

        float* get();

        void print();
    
    private:
        float calibration(float *callibration, float x);

    public:
        uint32_t measurement_id;

    private: 
        uint32_t state;
        Gpio<TGPIOC, 14, GPIO_MODE_OUT> ir_led;         //ir led
  

        int ir_off[IR_SENSORS_COUNT];
        int ir_on[IR_SENSORS_COUNT];

        float filter_coeff;
        

    private:
        float distance[IR_SENSORS_COUNT];

};

#endif