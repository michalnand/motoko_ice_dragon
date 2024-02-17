#include "device.h"
#include <drivers.h>
#include <tests.h>

#include <pwm.h>

#include <identification.h>
#include <position_control.h>

#include <fmath.h>
 
#define LED_1_GPIO        TGPIOE
#define LED_1_PIN         3
 
#define LED_2_GPIO        TGPIOE 
#define LED_2_PIN         2   

#define LED_3_GPIO        TGPIOD 
#define LED_3_PIN         13

#define KEY_GPIO          TGPIOE 
#define KEY_PIN           4 

#include <matrix.h> 

#include <lqg.h>     
     
int main(void)      
{ 
  /*
  Matrix<float, 3, 4> mat_a;
  Matrix<float, 3, 4> mat_b;
  Matrix<float, 3, 4> mat_c;
  Matrix<float, 3, 4> mat_y;
 
  mat_b = mat_a; 

  Matrix<float, 3, 4> mat_d = mat_a*3.0 + mat_b*2.1 - mat_c*5;
  */  

  LQG<6, 2, 3> lqg;  

  lqg.step(); 


  drivers_init();

  terminal << "\n\n\n";
  terminal << "machine ready\n";

  Gpio<LED_1_GPIO, LED_1_PIN, GPIO_MODE_OUT> led_1;   //user led
  led_1 = 1;  

  Gpio<LED_2_GPIO, LED_2_PIN, GPIO_MODE_OUT> led_2;   //bottom led
  Gpio<LED_3_GPIO, LED_3_PIN, GPIO_MODE_OUT> led_3;   //bottom led
    

  Gpio<KEY_GPIO, KEY_PIN, GPIO_MODE_IN_PULLUP> key;   //user button
 
  for (unsigned int i = 0; i < 5; i++)
  {     
    led_2 = 1;  
    led_3 = 1; 
    timer.delay_ms(50);
 
    led_2 = 0; 
    led_3 = 0; 
    timer.delay_ms(100);
  }

  led_2 = 1; 
  led_3 = 1; 


  while (key != 0)
  {
    led_1 = 1; 
    timer.delay_ms(100);

    led_1 = 0; 
    timer.delay_ms(200);
  }

  led_1 = 1; 
  

  //timer_test();

  //left_motor_connect_test();
  //right_motor_connect_test();

  //ir_sensor_test();
  //line_sensor_test();
  //gyro_sensor_test(); 
  //encoder_sensor_test();
  //sensors_test();
  

  //left_motor_pwm_test();
  //right_motor_pwm_test();
  //encoder_sensor_test();

  //motor_identification();
  //motor_driver_test();  
  //smooth_motor_driver_test();
  //robot_dynamics_identification();

  //line_stabilise(); 
  //gyro_turn_test(); 

  //line_follow_test();


  //motor_control.set_torque(0, 1000);

  
  PositionControl position_control;

  position_control.init();

  float req_dist[]  = {0.0, 0.08, 0.0, 0.08, 0.0, 0.08, 0.0,  0.0, 0.0,  0.0,   0.0, 0.0};
  float req_angle[] = {0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0,  0.0, 90.0, 0.0, -90.0, 0.0};

  uint32_t steps = 0;

  while (1) 
  {
    led_1 = 1;   
    
    float dist  = req_dist[(steps/400)%12];
    float angle = req_angle[(steps/400)%12]*PI/180.0;
 
    position_control.step(dist, angle);
    position_control.callback_position(); 

    led_1 = 0;  

    steps++;
    timer.delay_ms(4);
  }
  

  while (1) 
  {
    led_1 = 1; 
    timer.delay_ms(100);

    led_1 = 0; 
    timer.delay_ms(100);

    led_1 = 1; 
    timer.delay_ms(200);

    led_1 = 0; 
    timer.delay_ms(400);
  }
 
  return 0;
} 