#ifndef _GYRO_H_
#define _GYRO_H_


#include <i2c_interface.h>
 

class Gyro
{
    public:
        int init(I2C_Interface &i2c_interface, int dt);

        void callback();
        int32_t  read();

        //1.0 is equal to 1 full 360degrees rotation / s
        float get_angular_rate();

        //1.0  is equal  to 1 full circle in counter clockwise direction
        //-1.0 is equal to 1 full circle in clockwise direction
        float get_angle();

    private:
        I2C_Interface *i2c;
        int dt;

        int32_t sensitivity;
        int32_t offset_z;

    public:
        float angular_rate_z;
        float angle_z;      

    public:
        uint32_t measurement_id;  
};


#endif