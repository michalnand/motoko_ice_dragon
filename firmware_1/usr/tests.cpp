#include <tests.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper.h>

//#include <position_control_lqr.h>
#include <position_control_lqg.h>

#define LED_GPIO        TGPIOE
#define LED_PIN         3
   



void timer_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
  
  timer.reset();

  while(1)
  {
      led = 1; 
      timer.delay_ms(100);
      led = 0; 
      

      uint32_t gyro_m_prev = gyro_sensor.measurement_id;
      uint32_t ir_m_prev   = ir_sensor.measurement_id;
      uint32_t line_m_prev = line_sensor.measurement_id;
      
      timer.delay_ms(1000);
      
      uint32_t gyro_m_now  = gyro_sensor.measurement_id;
      uint32_t ir_m_now    = ir_sensor.measurement_id;
      uint32_t line_m_now  = line_sensor.measurement_id;


      uint32_t gyro_rate = gyro_m_now - gyro_m_prev;
      uint32_t ir_rate   = ir_m_now - ir_m_prev;
      uint32_t line_rate = line_m_now - line_m_prev;

      terminal << "time = " << timer.get_time()/1000 << " ";
      terminal << gyro_rate << " ";
      terminal << ir_rate << " ";
      terminal << line_rate << "\n";
  }
}

void ir_sensor_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
  
  led = 1; 

  while(1)
  {
      uint32_t measurement_id_prev = ir_sensor.measurement_id;
      timer.delay_ms(100);
      uint32_t measurement_id_now = ir_sensor.measurement_id;

      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      terminal << "ir readings    : ";


      for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
      {
        terminal << ir_sensor.get()[i] << " ";
      }
      terminal << ir_sensor.obstacle_detected() << " ";
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
 
      uint32_t measurement_id_prev = line_sensor.measurement_id;
      timer.delay_ms(500);
      uint32_t measurement_id_now  = line_sensor.measurement_id;

      terminal << "measurements/s : " << 2*(measurement_id_now - measurement_id_prev) << "\n";
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

      
      uint32_t measurement_id_prev = gyro_sensor.measurement_id;
      timer.delay_ms(100); 
      //gyro.callback();
      uint32_t measurement_id_now  = gyro_sensor.measurement_id;

      terminal << "measurements   : " << gyro_sensor.measurement_id << "\n";
      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      terminal << "gyro reading   : " << gyro_sensor.angular_rate << " " << gyro_sensor.angle << "\n";
      terminal << "\n\n";
  }
} 
  




void encoder_sensor_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

    
    while(1)  
    {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(450);

      
      terminal << "encoder\n";
      terminal << "left   : " << motor_control.get_left_angle()   << " " << motor_control.get_left_position()*180.0/PI   << " " << motor_control.get_left_velocity()*60.0/(2.0*PI) << "\n";
      terminal << "right  : " << motor_control.get_right_angle()  << " " << motor_control.get_right_position()*180.0/PI  << " " << motor_control.get_right_velocity()*60.0/(2.0*PI) << "\n";
      terminal << "\n\n\n";

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
    uint32_t line_measurement_id_prev = line_sensor.measurement_id;
    uint32_t gyro_measurement_id_prev = gyro_sensor.measurement_id;
    
    timer.delay_ms(100);
     
    uint32_t ir_measurement_id_now    = ir_sensor.measurement_id;
    uint32_t line_measurement_id_now  = line_sensor.measurement_id;
    uint32_t gyro_measurement_id_now  = gyro_sensor.measurement_id;


    terminal << "ir_sensor : \n";
    terminal << "measurements/s : " << 10*(ir_measurement_id_now - ir_measurement_id_prev) << "\n";
    ir_sensor.print();

    
    terminal << "line_sensor : \n";
    terminal << "measurements/s : " << 10*(line_measurement_id_now - line_measurement_id_prev) << "\n";
    line_sensor.print();


    terminal << "gyro :\n";
    terminal << "measurements/s : " << 10*(gyro_measurement_id_now - gyro_measurement_id_prev) << "\n";
    terminal << "reading        : " << gyro_sensor.angular_rate << " " << gyro_sensor.angle << "\n";
    terminal << "\n\n\n";

    terminal << "encoder\n";
    terminal << "left   : " << motor_control.get_left_angle()  << " " << motor_control.get_left_position()  << " " << motor_control.get_left_velocity() << "\n";
    terminal << "right  : " << motor_control.get_right_angle()  << " " << motor_control.get_right_position()  << " " << motor_control.get_right_velocity() << "\n";
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

    //required RPM velocity
    const float required[] = {0, 10, 50, 200, 1000, 1500, 0, -10, -50, -200, -1000, -1500};
    //const float required[] = {0, 1500}; 

    while (1)   
    {
      uint32_t time = timer.get_time();
      uint32_t required_idx = (time/4000)%12;

      //convert rpm to rad/s
      float req = required[required_idx]*2.0*PI/60.0;
        
      motor_control.set_velocity(req, req);
 

      if ((time/50)%10 == 0)
      { 
        led = 1;  

        terminal << "req   = " << req*60.0/(2.0*PI) << "\n";  
        terminal << "left  = " <<  motor_control.get_left_velocity()*60.0/(2.0*PI) << "\n";
        terminal << "right = " <<  motor_control.get_right_velocity()*60.0/(2.0*PI) << "\n";
        terminal << "\n\n"; 
        
      }
      else
      {
        led = 0;
      }
    }
}




void smooth_motor_driver_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led


    led = 0;
    timer.delay_ms(500); 

    float speed_max_rpm = 1500.0;

    //required max velocity
    const float required[] = {1.0, 0.0}; 

    //shaper options
    float dx_max_list[] = {1.0, 0.1, 0.05, 0.025};

  
    uint32_t n_steps = 1000;
    uint32_t dt = 4; //4ms
    uint32_t shaper_id = 0;

    Shaper shaper;  
    shaper.init(1.0, -1.0); 

    while (1)    
    { 
      float dx_max = dx_max_list[shaper_id];

      shaper.set_limits(dx_max, -dx_max);
 
      terminal << "shaper value = " << dx_max << "\n";

      for (unsigned int n = 0; n < n_steps; n++)
      {
        uint32_t required_idx = n/(n_steps/2);

        //convert rpm to rad/s
        float req = required[required_idx]; 

        if (req > 0.0)
        {
          led = 1;
        }
        else
        {
          led = 0;
        }

        //shape signal
        float req_shaped = shaper.step(req);

        req_shaped = req_shaped*speed_max_rpm*2.0*PI/60.0;

        motor_control.set_velocity(req_shaped, req_shaped);
 
        timer.delay_ms(dt);
      }


      shaper_id = (shaper_id + 1)%4;
    }
}



void turn_test()
{
  position_control.init();

  //turn test
  float req_angle[] = {0.0, 90.0, 0.0, -90.0, 0.0, 90.0, 0.0, -90.0, 0.0, 90.0, 0.0, -90.0};


  for (unsigned int n = 0; n < 12; n++)
  {
    float dist  = 0;
    float angle = req_angle[n]*PI/180.0;

    position_control.set(dist, angle);

    timer.delay_ms(300);
  } 

  position_control.set(0, 0);
}

void forward_test()
{
  position_control.init();

  //forward test
  float req_distance[] = {0.0, 100.0, 0.0, 150.0, 0.0, 200.0, 0.0, 250.0, 0.0, 500.0};

  for (unsigned int n = 0; n < 10; n++) 
  {
    float dist  = req_distance[n];
    float angle = 0.0;

    position_control.set(dist, angle);

    timer.delay_ms(800);
  } 

  position_control.set(0, 0);
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





void noise_measurement()
{
  uint32_t n_max = 1000;

  float mean_distance          = 0.0;
  float mean_angle             = 0.0;
  float mean_velocity          = 0.0;
  float mean_angular_velocity  = 0.0;


  float var_distance          = 0.0;
  float var_angle             = 0.0;
  float var_velocity          = 0.0;
  float var_angular_velocity  = 0.0;

  terminal << "measuring mean and variance\n";


  float k = 0.99; 

  for (unsigned int n = 0; n < n_max; n++)
  {
    float d  = position_control.distance; 
    float a  = position_control.angle;
    float dd = position_control.distance - position_control.distance_prev;
    float da = position_control.angle    - position_control.angle_prev;

    mean_distance         = k*mean_distance +         (1.0 - k)*d;
    mean_angle            = k*mean_angle +            (1.0 - k)*a;
    mean_velocity         = k*mean_velocity +         (1.0 - k)*dd;
    mean_angular_velocity = k*mean_angular_velocity + (1.0 - k)*da;


    var_distance         = k*var_distance +         (1.0 - k)*(d - mean_distance)*(d - mean_distance);
    var_angle            = k*var_angle +            (1.0 - k)*(a - mean_angle)*(a - mean_angle);
    var_velocity         = k*var_velocity +         (1.0 - k)*(dd - mean_velocity)*(dd - mean_velocity);
    var_angular_velocity = k*var_angular_velocity + (1.0 - k)*(da - mean_angular_velocity)*(da - mean_angular_velocity);

    timer.delay_ms(4);
  }

  var_distance*= 1000;
  var_angle*= 1000;
  var_velocity*= 1000;
  var_angular_velocity*= 1000;


  terminal << "noise mean\n";
  terminal << mean_distance << " " << mean_angle << " " << mean_velocity << " " << mean_angular_velocity << "\n";
  terminal << "\n\n";

  terminal << "noise var\n";
  terminal << var_distance << " " << var_angle << " " << var_velocity << " " << var_angular_velocity << "\n";
  terminal << "\n\n";

  var_distance         = fsqrt(var_distance);
  var_angle            = fsqrt(var_angle);
  var_velocity         = fsqrt(var_velocity);
  var_angular_velocity = fsqrt(var_angular_velocity);

  terminal << "noise std\n";
  terminal << var_distance << " " << var_angle << " " << var_velocity << " " << var_angular_velocity << "\n";
  terminal << "\n\n";


}




/*
void gyro_turn_test()
{
  timer.delay_ms(1000);

  float angles[] = {0, 45, 90, 0, -45, -90};

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

  led = 1;

  uint32_t steps = 0;
  int32_t dt = 4;

  LQRServo lqr;

  float speed_max = 1500.0*2.0*PI/60.0;

  lqr.init(2.00146, 52.10817, 316.22777, speed_max, dt/1000.0);
 
  
  while(1)
  {
    uint32_t idx = (steps/500)%6; 

    float angle         = gyro_sensor.angle;
    float angular_rate  = gyro_sensor.angular_rate_filtered;

    float xr = angles[idx]*PI/180.0; 
     
 
    float u = lqr.step(xr, angle, angular_rate);

    motor_control.set_velocity(-u, u); 
    timer.delay_ms(dt);
    
    steps++;
  }
}




void line_follow_test()
{
  timer.delay_ms(1000);

  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

  led = 1;

  int32_t dt = 4;

  float forward = 0.0;

  LQRServo lqr;

  float turn_speed_max     = 1500.0*2.0*PI/60.0;
  float forward_speed_max  = 200.0*2.0*PI/60.0;
  //lqr.init(2.00146, 52.10817, 316.22777, turn_speed_max, dt/1000.0);
  
  lqr.init(0.5, 80.0, 316.0, turn_speed_max, dt/1000.0);
  
   
  while(1)
  {

    float angle         = -line_sensor.angle;
    float angular_rate  = -line_sensor.angular_rate;
     
    float turn       = lqr.step(0.0, angle, angular_rate);
    

    if (line_sensor.line_lost_type == LINE_LOST_NONE)
    { 
      float k = 0.9;
      forward = k*forward + (1.0 - k)*forward_speed_max;
    }
    else 
    {
      forward = 0.0;
    }

    motor_control.set_velocity(-turn + forward, turn + forward); 
    timer.delay_ms(dt);
  }
}

*/