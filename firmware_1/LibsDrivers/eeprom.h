#ifndef _EEPROM_H_
#define _EEPROM_H_


#include <drivers.h>


class EEPROM
{
    public:
        void init(I2C_Interface &i2c_interface);
        void write(uint32_t page_adr, uint8_t *buffer);
        void read(uint32_t page_adr, uint8_t *buffer);

    private:
        I2C_Interface *i2c;
};


#endif