#include "device.h"
#include <drivers.h>
#include <tests.h>

#include <pwm.h>
 
#define LED_GPIO        TGPIOE
#define LED_PIN         2

#define KEY_GPIO        TGPIOE
#define KEY_PIN         3


#include <identification.h> 



int main(void) 
{
  drivers_init();

  terminal << "\n\n\n";
  terminal << "machine ready\n";

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
  led = 1; 

  Gpio<KEY_GPIO, KEY_PIN, GPIO_MODE_IN_PULLUP> key;  //user button


  //ir_sensor_test();
  //line_sensor_test();
  //gyro_sensor_test();
  //encoder_sensor_test();
  //sensors_test();
  

  //left_motor_pwm_test();
  //right_motor_pwm_test();
  //encoder_sensor_test();
  //motor_driver_test();

  //mcu_usage();
 
  turn_dynamics_identification();
  

  while (1)
  {
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(100);

    led = 1; 
    timer.delay_ms(200);

    led = 0; 
    timer.delay_ms(400);
  }
 
  return 0;
} 