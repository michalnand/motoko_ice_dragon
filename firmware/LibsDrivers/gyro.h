#ifndef _GYRO_H_
#define _GYRO_H_


#include <i2c_interface.h>
 

class Gyro
{
    public:
        int init(I2C_Interface &i2c_interface);

        void callback();
        int32_t  read();

        void reset_angle();

    private:
        I2C_Interface *i2c;
        int odr;

        int32_t sensitivity;
        int32_t offset_z;

        float angular_rate_old[3];

    public:
        //1.0 is equal to 1 full 360degrees rotation / s
        float angular_rate;

        //low pass filtered angular rate
        float angular_rate_filtered;

        //1.0  is equal  to 1 full circle in counter clockwise direction
        //-1.0 is equal to 1 full circle in clockwise direction
        float angle;      

    public:
        uint32_t measurement_id;  
};


#endif