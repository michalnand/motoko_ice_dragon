#include <tests.h>
#include <drivers.h>

#define LED_GPIO        TGPIOE
#define LED_PIN         2


void ir_sensor_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
    
  while(1)
  {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(50);

      uint32_t measurement_id_prev = ir_sensor.measurement_id;
      timer.delay_ms(100);
      uint32_t measurement_id_now = ir_sensor.measurement_id;

      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      terminal << "ir readings    : ";

      for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
      {
        terminal << ir_sensor.get()[i] << " ";
      }
      terminal << "\n\n\n";
  }
}



void line_sensor_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
    
    while(1)
    {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(50);
 
      uint32_t measurement_id_prev = line_sensor.result.measurement_id;
      timer.delay_ms(100);
      uint32_t measurement_id_now  = line_sensor.result.measurement_id;

      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      line_sensor.print();

      terminal << "\n\n\n";
  }
}


void gyro_sensor_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
    
  while(1)
  {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(50);

      
      uint32_t measurement_id_prev = gyro.measurement_id;
      timer.delay_ms(100); 
      //gyro.callback();
      uint32_t measurement_id_now  = gyro.measurement_id;

      terminal << "measurements   : " << gyro.measurement_id << "\n";
      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      terminal << "gyro reading   : " << gyro.get_angular_rate() << " " << gyro.get_angle() << "\n";
      terminal << "\n\n";
  }
} 
  




void encoder_sensor_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

    
    motor_control.set_torque(MOTOR_CONTROL_MAX/4, 0);

    while(1)  
    {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(50);

      
      terminal << "encoder_l = " << motor_control.left_encoder.angle  << " " << motor_control.left_encoder.position  << " " << motor_control.left_encoder.angular_velocity << "\n";
      terminal << "encoder_r = " << motor_control.right_encoder.angle << " " << motor_control.right_encoder.position << " " << motor_control.right_encoder.angular_velocity << "\n";
      terminal << "\n\n";
  }
}



void sensors_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

  uint32_t counter_no_load = 14391346/5;

  while(1)
  {
    uint32_t time_start = timer.get_time();
    uint32_t time_stop  = time_start + 200;
    uint32_t counter = 0;

    led = 1;
    while (timer.get_time() < time_stop)
    {
      counter++;
    }
    led = 0;

    uint32_t cpu_usage = 100 - (100*counter)/counter_no_load;

    terminal << "cpu_usage = " << cpu_usage << " [%]\n\n";

    uint32_t ir_measurement_id_prev   = ir_sensor.measurement_id;
    uint32_t line_measurement_id_prev = line_sensor.result.measurement_id;
    uint32_t gyro_measurement_id_prev = gyro.measurement_id;
    
    timer.delay_ms(100);
    
    uint32_t ir_measurement_id_now    = ir_sensor.measurement_id;
    uint32_t line_measurement_id_now  = line_sensor.result.measurement_id;
    uint32_t gyro_measurement_id_now  = gyro.measurement_id;


    terminal << "ir_sensor : \n";
    terminal << "measurements/s : " << 10*(ir_measurement_id_now - ir_measurement_id_prev) << "\n";
    ir_sensor.print();

    
    terminal << "line_sensor : \n";
    terminal << "measurements/s : " << 10*(line_measurement_id_now - line_measurement_id_prev) << "\n";
    line_sensor.print();


    terminal << "gyro :\n";
    terminal << "measurements/s : " << 10*(gyro_measurement_id_now - gyro_measurement_id_prev) << "\n";
    terminal << "reading        : " << gyro.get_angular_rate() << " " << gyro.get_angle() << "\n";
    terminal << "\n\n\n";

    terminal << "encoder\n";
    terminal << "left : " << motor_control.left_encoder.angle  << " " << motor_control.left_encoder.position  << " " << motor_control.left_encoder.angular_velocity << "\n";
    terminal << "righ : " << motor_control.right_encoder.angle << " " << motor_control.right_encoder.position << " " << motor_control.right_encoder.angular_velocity << "\n";
    terminal << "\n\n\n";

    terminal << "\n\n\n\n";
  }
}

void left_motor_connect_test()
{
  uint32_t state = 0;

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

  Gpio<TGPIOC, 9, GPIO_MODE_OUT> la;
  Gpio<TGPIOC, 8, GPIO_MODE_OUT> lb;
  Gpio<TGPIOC, 7, GPIO_MODE_OUT> lc;
  
  Gpio<TGPIOA, 8, GPIO_MODE_OUT> l_enable;

  l_enable = 1;

  uint32_t speed = 100;
   
  while(1)
  {
    if (state == 0)
    {
      la = 1;
      lb = 0;
      lc = 0;
    }
    else if (state == 1)
    {
      la = 1;
      lb = 1;
      lc = 0;
    }
    else if (state == 2)
    {
      la = 0;
      lb = 1;
      lc = 0;
    }
    else if (state == 3)
    {
      la = 0;
      lb = 1;
      lc = 1;
    }
    else if (state == 4)
    {
      la = 0;
      lb = 0;
      lc = 1;
    }
    else if (state == 5)
    {
      la = 1;
      lb = 0;
      lc = 1;
    }

    state = (state + 1)%6;

    led = 1; 
    timer.delay_ms(speed);

    led = 0; 
    timer.delay_ms(speed);

    if (speed > 2)
    {
      speed--; 
    }
  }
}


void left_motor_pwm_test()
{
  uint32_t state = 0;

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

  PWMLeft pwm;
  pwm.init();
  pwm.set(0, 0, 0);

  uint32_t pwm_value = PWM_PERIOD/2;
  uint32_t speed = 100;


   
  while(1)
  {
    if (state == 0)
    {
      pwm.set(pwm_value, 0, 0);
    }
    else if (state == 1)
    {
      pwm.set(pwm_value, pwm_value, 0);
    }
    else if (state == 2)
    {
      pwm.set(0, pwm_value, 0);
    }
    else if (state == 3)
    {
      pwm.set(0, pwm_value, pwm_value);
    }
    else if (state == 4)
    {
      pwm.set(0, 0, pwm_value);
    }
    else if (state == 5)
    {
      pwm.set(pwm_value, 0, pwm_value);
    }

    state = (state + 1)%6;

    led = 1; 
    timer.delay_ms(speed);

    led = 0; 
    timer.delay_ms(speed);

    if (speed > 2)
    {
      speed--; 
    }
  }
}




//PWMA      : PB8, TIM4_CH3
//PWMB      : PB7, TIM4_CH2
//PWMC      : PB6, TIM4_CH1
//enable    : PB9
void right_motor_connect_test()
{
  uint32_t state = 0;

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

  Gpio<TGPIOB, 8, GPIO_MODE_OUT> ra;
  Gpio<TGPIOB, 7, GPIO_MODE_OUT> rb;
  Gpio<TGPIOB, 6, GPIO_MODE_OUT> rc;
  
  Gpio<TGPIOB, 9, GPIO_MODE_OUT> r_enable;

  r_enable = 1;

  uint32_t speed = 100;
   
  while(1)
  {
    if (state == 0)
    {
      ra = 1;
      rb = 0;
      rc = 0;
    }
    else if (state == 1)
    {
      ra = 1;
      rb = 1;
      rc = 0;
    }
    else if (state == 2)
    {
      ra = 0;
      rb = 1;
      rc = 0;
    }
    else if (state == 3)
    {
      ra = 0;
      rb = 1;
      rc = 1;
    }
    else if (state == 4)
    {
      ra = 0;
      rb = 0;
      rc = 1;
    }
    else if (state == 5)
    {
      ra = 1;
      rb = 0;
      rc = 1;
    }

    state = (state + 1)%6;

    led = 1; 
    timer.delay_ms(speed);

    led = 0; 
    timer.delay_ms(speed);

    if (speed > 2)
    { 
      speed--; 
    }
  }
}




void right_motor_pwm_test()
{
  uint32_t state = 0;

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

  PWMRight pwm;
  pwm.init();
  pwm.set(0, 0, 0);

  uint32_t pwm_value = PWM_PERIOD/2;
  uint32_t speed = 100;


   
  while(1)
  {
    if (state == 0)
    {
      pwm.set(pwm_value, 0, 0);
    }
    else if (state == 1)
    {
      pwm.set(pwm_value, pwm_value, 0);
    }
    else if (state == 2)
    {
      pwm.set(0, pwm_value, 0);
    }
    else if (state == 3)
    {
      pwm.set(0, pwm_value, pwm_value);
    }
    else if (state == 4)
    {
      pwm.set(0, 0, pwm_value);
    }
    else if (state == 5)
    {
      pwm.set(pwm_value, 0, pwm_value);
    }

    state = (state + 1)%6;

    led = 1; 
    timer.delay_ms(speed);

    led = 0; 
    timer.delay_ms(speed);

    if (speed > 2)
    {
      speed--; 
    }
  }
}




void motor_driver_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

    
    int speed = 0;
    unsigned int state = 0;


    while(1)  
    {
      led = 1; 
      timer.delay_ms(10);

      led = 0; 
      timer.delay_ms(10);

      if (state == 0)
      {
        speed+= 5;
        if (speed >= MOTOR_CONTROL_MAX)
        {
          timer.delay_ms(1000);
          state = 1;
        }
      }
      else if (state == 1)
      {
        speed-= 5;
        
        if (speed <= 5)
        {
          state = 2;
          motor_control.set_torque(0, 0);
        }
      }
      else
      {
        for (unsigned int i = 0; i < 5; i++)
        {
          motor_control.set_torque(MOTOR_CONTROL_MAX, -MOTOR_CONTROL_MAX); 
          timer.delay_ms(1000);
          motor_control.set_torque(0, 0); 
          timer.delay_ms(300);
        }

        state = 0;
      }


      motor_control.set_torque(speed, -speed);      
  }
}


void mcu_usage()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

  uint32_t counter_no_load = 14391346;

  while(1)
  {
    uint32_t time_start = timer.get_time();
    uint32_t time_stop  = time_start + 1000;
    uint32_t counter = 0;

    led = 1;
    while (timer.get_time() < time_stop)
    {
      counter++;
    }
    led = 0;

    uint32_t cpu_usage = 100 - (100*counter)/counter_no_load;

    terminal << "cpu_usage = " << cpu_usage << " [%]\n";
  }
}


