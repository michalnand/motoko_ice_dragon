#include "drivers.h"

Timer                       timer;
Terminal                    terminal;
ADC_driver                  adc;
TI2C<TGPIOD, 1, 2, 255>     i2c;
 
IRSensor     ir_sensor;
LineSensor   line_sensor;
Gyro         gyro;   

 
MotorControl motor_control;
 
void drivers_init()
{
    SetSysClock(SysClok216_8HSE);

    timer.init();
    timer.delay_ms(100);

    terminal.init();
    i2c.init();
    
    ir_sensor.init(); 
    adc.init();
 
    line_sensor.init(4);

    gyro.init(i2c, 5);
    
    timer.delay_ms(100);

    motor_control.init();
}

