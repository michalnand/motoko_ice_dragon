#include <eeprom.h>
#include <drivers.h>


#define EEPROM_ADDRESS              ((unsigned char)0xA0)
#define EEPROM_PAGE_SIZE            ((unsigned int)32)
#define EEPROM_PAGE_COUNT           ((unsigned int)(8192/EEPROM_PAGE_SIZE))

void EEPROM::init(I2C_Interface &i2c_interface)
{
    this->i2c = &i2c_interface;
}

void EEPROM::write(uint32_t page_adr, uint8_t *buffer)
{
    uint32_t adr_tmp = page_adr*EEPROM_PAGE_SIZE;

    i2c->start();
    i2c->write(EEPROM_ADDRESS);
    i2c->write(adr_tmp>>8); 
    i2c->write(adr_tmp); 

    for (unsigned int i = 0; i < EEPROM_PAGE_SIZE; i++)
    {
        i2c->write(buffer[i]);
    }

    i2c->stop();
}


void EEPROM::read(uint32_t page_adr, uint8_t *buffer)
{
    uint32_t adr_tmp = page_adr*EEPROM_PAGE_SIZE;

    i2c->start(); 
    i2c->write(EEPROM_ADDRESS);
    i2c->write(adr_tmp>>8); 
    i2c->write(adr_tmp); 

    i2c->start();
    i2c->write(EEPROM_ADDRESS|0x01);

    for (unsigned int i = 0; i < EEPROM_PAGE_SIZE; i++)
    {
        buffer[i] = i2c->read(1);
    }

    i2c->stop();
}

