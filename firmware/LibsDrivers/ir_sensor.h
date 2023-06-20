#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include <kalman_filter.h>

#define IR_SENSORS_COUNT        ((unsigned int)4)


#define IR_SENSORS_VARIANCE     ((float)0.1)

class IRSensor
{
    public:
        void init();

        void callback();

    private:
        float calibration(int idx, float x);

    private:
        int state;
        int ir_led;

        KalmanFilter filters[IR_SENSORS_COUNT];

        int ir_off[IR_SENSORS_COUNT];
        int ir_on[IR_SENSORS_COUNT];

        //quadratic calibration values
        float cal_a[IR_SENSORS_COUNT];
        float cal_b[IR_SENSORS_COUNT];
        float cal_c[IR_SENSORS_COUNT];

    public:
        float distance[IR_SENSORS_COUNT];
};

#endif