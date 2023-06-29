#ifndef _GYRO_H_
#define _GYRO_H_


#include <i2c_interface.h>
 

class Gyro
{
    public:
        int init(I2C_Interface &i2c_interface, int dt);

        void callback();
        int32_t  read();

        int32_t get_angular_rate();
        int32_t get_angle();

    private:
        I2C_Interface *i2c;
        int dt;

        int32_t sensitivity;
        int32_t offset_z;

        int32_t angular_rate_z;
        int32_t angle_z;      

    public:
        uint32_t measurement_id;  
};


#endif