#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include <kalman_filter.h>
#include <gpio.h>

#define IR_SENSORS_COUNT        ((unsigned int)4)


#define IR_SENSORS_VARIANCE     ((float)0.1)

class IRSensor
{
    public:
        void init();

        void callback();

        float* get();

    private:
        float calibration(float *callibration, float x);

    private: 
        uint32_t state, measurement_id;
        Gpio<TGPIOC, 14, GPIO_MODE_OUT> ir_led;         //ir led
  
        //KalmanFilter filters[IR_SENSORS_COUNT];

        int ir_off[IR_SENSORS_COUNT];
        int ir_on[IR_SENSORS_COUNT];

        float filter_coeef;

        

    private:
        float distance[IR_SENSORS_COUNT];

};

#endif