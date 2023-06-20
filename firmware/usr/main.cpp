#include "device.h"
#include <clock.h>

#include <gpio.h>
#include <timer.h>
 
#define LED_GPIO        TGPIOE
#define LED_PIN         2

#define KEY_GPIO        TGPIOE
#define KEY_PIN         3
  

Timer timer;
 

int main(void) 
{
  SetSysClock(SysClok216_8HSE);

  timer.init();
  
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
  led = 1; 

  Gpio<KEY_GPIO, KEY_PIN, GPIO_MODE_IN_PULLUP> key;  //user button


  //Gpio<TGPIOC, 14, GPIO_MODE_OUT> ir_led;         //ir led
  //ir_led = 1; 

  Gpio<TGPIOC, 15, GPIO_MODE_OUT> line_led;         //line led
  line_led = 1; 


  while (1)
  {
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(100);

    led = 1; 
    timer.delay_ms(300);

    led = 0; 
    timer.delay_ms(900); 
  }

  return 0;
} 