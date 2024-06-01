#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper.h>
#include <lqr_single.h>

#include <pid.h>

//#include <position_control_lqr.h>
#include <position_control_lqg.h>


#define LED_GPIO        TGPIOE
#define LED_PIN         2



/*
void motor_identification()
{
    motor_control.set_torque(0, 0); 
    timer.delay_ms(200); 

    //run motor on full speed
    motor_control.set_torque(MOTOR_CONTROL_MAX, 0); 
    timer.delay_ms(800);
    unsigned int steps_max = 400;

    //motor free run speed, in RPM
    float motor_max_speed_mean = 0;
    float motor_max_speed_var  = 0;

    //average maximum speed
    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        motor_max_speed_mean+= speed;
        timer.delay_ms(2);
    } 

    motor_max_speed_mean = motor_max_speed_mean/steps_max;

    //encoder velocity measurement variance
    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        float dif   = speed - motor_max_speed_mean;
        motor_max_speed_var+= dif*dif;
        timer.delay_ms(2); 
    } 

    motor_max_speed_var = motor_max_speed_var/steps_max;

    motor_control.set_torque(0, 0); 
    timer.delay_ms(200);

    //print results
    terminal << "motor_max_speed_mean   " << motor_max_speed_mean << " rpm\n";
    terminal << "motor_max_speed_var    " << motor_max_speed_var << " rpm^2\n";

    //time constant estimation
    steps_max = 20;

    float time_constant = 0.0;

    for (unsigned int i = 0; i < steps_max; i++)
    {
        int32_t time_start = timer.get_time();

        motor_control.set_torque(MOTOR_CONTROL_MAX, 0); 

        while (motor_control.get_left_velocity()*(60.0/(2.0*PI)) < 0.632*motor_max_speed_mean)
        {
            __asm("nop");
        }

        int32_t time_stop = timer.get_time();

        motor_control.set_torque(0, 0); 
        timer.delay_ms(100);

        int32_t dt = time_stop - time_start; 

        time_constant+= dt;
    }

    time_constant = time_constant/steps_max;

    terminal << "time_constant   " << time_constant << " ms\n";
    terminal << "\n\n";
}
*/


void motor_identification()
{
    //stop motor
    motor_control.set_torque(0, 0); 
    timer.delay_ms(200); 

    uint32_t n_steps = 500;

    //1, estimate motor constant k, on different input values

    float u_values[10] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    float k_mean     = 0.0;
    float x_var_mean = 0.0;

    for (unsigned int j = 0; j < 10; j++)
    {
        //run motor with desired input and wait for steady state
        float u_in = u_values[j];

        motor_control.set_torque(MOTOR_CONTROL_MAX*u_in, 0); 
        timer.delay_ms(500);

        //estimate average velocity
        float x_mean = 0.0;
        for (unsigned int i = 0; i < n_steps; i++)
        {
            float x = motor_control.get_left_velocity();
            x_mean+= x;
            timer.delay_ms(1); 
        }
        x_mean = x_mean/n_steps;    

        //estimate average variance - encoder noise
        float x_var = 0.0;
        for (unsigned int i = 0; i < n_steps; i++)
        {
            float x = motor_control.get_left_velocity();
            x_var+= (x - x_mean)*(x - x_mean);
            timer.delay_ms(1);
        }
        x_var = x_var/n_steps;  

        float k = x_mean/u_in;
        k_mean+= x_mean/u_in;   
        x_var_mean+= x_var;

        terminal << "u_in   = " << u_in << "\n";
        terminal << "k      = " << k << "\n";
        terminal << "x_mean = " << x_mean << "\n";
        terminal << "x_var  = " << x_var << "\n";
        terminal << "\n\n";
    }   


    //print summary results
    k_mean     = k_mean/10.0;
    x_var_mean = x_var_mean/10.0;

    terminal << "k          = " << k_mean << "\n";
    terminal << "x_var_mean = " << x_var_mean << "\n";
    
    terminal << "\n\n";


    //2, estimate time constant by oscilating motor

    motor_control.set_torque(0, 0); 
    timer.delay_ms(200); 

    float u_in = 0.25;
    uint32_t periods = 0; 

    n_steps = 2000;

    for (unsigned int i = 0; i < n_steps; i++)
    {
        motor_control.set_torque(u_in*MOTOR_CONTROL_MAX, 0);
        float x = motor_control.get_left_velocity()*60.0/(2.0*PI);

        if (u_in > 0.0)
        {
            if (x > 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;
            }
        }
        else
        {
            if (x < 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;  
            }
        }

        timer.delay_ms(1);
    }
    
    motor_control.set_torque(0, 0); 
    timer.delay_ms(200);     
    
    float t_period = (2*n_steps/periods)/PI;
    terminal << "periods = " << periods << "\n";
    terminal << "tau     = " << t_period << "[ms]\n";
    terminal << "\n\n";
}

/*
void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //float left_required_positions[]  = {0.0,  1.0, -1.0, 0.0,  0.0, 1.0, -1.0, 0.0};
    //float right_required_positions[] = {0.0,  0.0,  0.0, 1.0, -1.0, -1.0, 1.0, 0.0};

    float left_required_positions[]  = {0.0, 1.0, 0.0, -1.0,   0.0, -1.0, 0.0,  1.0,   0.0, 1.0, 0.0, -1.0,   0.0, 0.0, 0.0,  0.0};
    float right_required_positions[] = {0.0, 1.0, 0.0, -1.0,   0.0,  1.0, 0.0, -1.0,   0.0, 0.0, 0.0, 0.0,    0.0, 1.0, 0.0, -1.0};

    //max allowed speed, rpm to rad/s
    float speed_max = 1500.0*2.0*PI/60.0;

    //float k  = 10.0;  
    //float ki = 0.4;         

    float k  = 5.0;   
    float ki = 0.2;         


    LQRSingle left_lqr, right_lqr;

    left_lqr.init(k, ki, speed_max);
    right_lqr.init(k, ki, speed_max);

    Shaper shaper_left, shaper_right;  

    shaper_left.init(0.5, -2.0); 
    shaper_right.init(0.5, -2.0); 

 
    unsigned int steps = 0; 

    int32_t dt = 4;
    float curr_dt_fil = 0.0;

    while (1) 
    {    
        int32_t time_start = timer.get_time();

        //obtain required value
        unsigned int idx = (steps/200)%16;   
        float left_required  = left_required_positions[idx]*PI*0.75;
        float right_required = right_required_positions[idx]*PI*0.75;
 
        //obtain measured
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        //LQR controller
        float left_u   = left_lqr.step(left_required, left_position);
        float right_u  = right_lqr.step(right_required, right_position);
  
        //output shaping
        float vl_shaped = shaper_left.step(left_u); 
        float vr_shaped = shaper_right.step(right_u);

        //send to motors
        motor_control.set_velocity(vl_shaped, vr_shaped);

        terminal << steps << " " << left_u << " " << right_u << " " << left_position << " " << right_position << " " << curr_dt_fil << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        int32_t dif = time_stop - time_start;
        int32_t curr_dt = dt - dif;

        curr_dt_fil = 0.9*curr_dt_fil + 0.1*dif;

        if (curr_dt > 0)
        {
            timer.delay_ms(curr_dt);
        }
    }
}
*/



/*
void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;

    float shaper_ramp   = 0.005;

    //max speed, 1500rpm
    float speed_max     = 1500*2.0*PI/60.0; 

    float required_forward[] = {0.0, 50.0, 0.0, -50.0, 0.0,  0.0,  0.0,   0.0,  0.0};
    float required_turn[]    = {0.0, 0.0,  0.0,  0.0,  0.0, 90.0, 0.0, -90.0, 0.0};

    //float required_forward[] = {0.0, 0.0, 0.0, 0.0};
    //float required_turn[]    = {0.0, 90.0, 0.0, -90.0};

    //controllers
    float pf = 0.01;
    float df = 0.03;

    float pt = 0.08; 
    float dt = 0.1; 

    //input shaper
    Shaper shaper_left, shaper_right;
    
    //shaper init
    shaper_left.init(shaper_ramp, -shaper_ramp); 
    shaper_right.init(shaper_ramp, -shaper_ramp); 

    uint32_t steps = 0;

    float e0_forward = 0.0;
    float e1_forward = 0.0;

    float e0_turn = 0.0;
    float e1_turn = 0.0;

    float d_time = 0.0;
    while (1)  
    {     
        int32_t time_start = timer.get_time();

        uint32_t idx = (steps/200)%9;

        //required values
        float req_forward = required_forward[idx];
        float req_turn    = required_turn[idx]*PI/180.0;

        //obtain current state

        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        //compute controller
        e1_forward = e0_forward;
        e0_forward = req_forward - distance;

        e1_turn = e0_turn;
        e0_turn = req_turn - angle;

        float u_forward = pf*e0_forward + df*(e0_forward - e1_forward);
        float u_turn    = pt*e0_turn + dt*(e0_turn - e1_turn); 

        float u_left  = u_forward - u_turn;
        float u_right = u_forward + u_turn; 

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0);


        float u_left_shaped  = shaper_left.step(u_left);
        float u_right_shaped = shaper_right.step(u_right);

        motor_control.set_velocity(u_left_shaped*speed_max, u_right_shaped*speed_max);

        terminal << steps << " " << u_left << " " << u_right << " " << u_left_shaped << " " << u_right_shaped << " " << distance << " " <<  angle << " " << d_time << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        d_time = 0.9*d_time + 0.1*(time_stop - time_start);

        int32_t time_wait = 4 - (time_stop - time_start);

        if (time_wait < 0)
        {
            time_wait = 1;
        }

        timer.delay_ms(time_wait);
    }
}
*/



void shaper_test() 
{
    float speed_max     = 0.5; 

    float distance_max  = 50.0;    //required distance in mm
    
    float shaper_ramp   = 0.002; 
    
    
    float wheel_diameter = 34.0; 
    
    Shaper shaper;
    shaper.init(shaper_ramp, -4*shaper_ramp);
    
    

    float u = speed_max;
    while (1) 
    {
        float u_shaped = shaper.step(u);
        
        u_shaped = (1500*2.0*PI/60)*u_shaped;

        motor_control.set_velocity(u_shaped, u_shaped);

        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;

        if (u > 0.0 && distance > distance_max)
        { 
            u = -speed_max; 
        }
        else if (u < 0.0 && distance < -distance_max)
        {
            u = speed_max;
        }


        //terminal << left_position << " " << right_position << " " << distance << "\n";

        timer.delay_ms(4);
    }
}



void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;

    PID forward_pid, turn_pid;

    forward_pid.init(0.002, 0.0, 0.0, 1.0); 
    turn_pid.init(0.2, 0.0, 1.0, 1.0);  
 
    //input shaper
    Shaper forward_shaper, turn_shaper;
    
    //shaper init   
    forward_shaper.init(0.004, -0.004);    
    turn_shaper.init(0.1, -0.1);      

    float distances[]  = {0.0, 150.0, 0.0, -150.0, 0.0, 0.0,  0.0,  0.0};
    float angles[]     = {0.0, 0.0, 0.0,  0.0, 0.0, 90.0, 0.0, -90.0};

    //float distances[]  = {0.0, 100.0};
    //float angles[]     = {0.0, 90.0}; 


    float speed_max = 1500.0*2.0*PI/60.0;

    uint32_t steps = 0; 

    while (true)
    {
        int32_t time_start = timer.get_time();

        uint32_t idx = (steps/200)%8;
        
        float req_forward = distances[idx]; 
        float req_turn    = angles[idx]*PI/180.0;


        //obtain current state
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        // compute controller output
        float forward_curr = forward_pid.step(req_forward, distance);
        float turn_curr    = turn_pid.step(req_turn, angle);
      
        // compute shaped output
        float forward_curr_shaped = forward_shaper.step(forward_curr);
        float turn_curr_shaped    = turn_shaper.step(turn_curr);

        float u_left  = forward_curr_shaped - turn_curr_shaped;
        float u_right = forward_curr_shaped + turn_curr_shaped; 

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0);

        // send to motors
        motor_control.set_velocity(u_left*speed_max, u_right*speed_max);

        terminal << steps << " " << forward_curr << " " << turn_curr << " " << forward_curr_shaped << " " << turn_curr_shaped << " " << distance << " " <<  angle << " " << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        //compute delay time
        int32_t time_wait = 4 - (time_stop - time_start);
        if (time_wait < 0)
        {
            time_wait = 1;
        }

        timer.delay_ms(time_wait);
    }
}



/*
void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;


     //input shaper
    Shaper forward_shaper, turn_shaper;
    
    //shaper init
    forward_shaper.init(0.002, -0.002); 
    turn_shaper.init(0.005, -0.005);    

    uint32_t forward_state  = 0;
    uint32_t forward_steps  = 0;
    float forward_control   = 0.2; 
    float forward_distance  = 20.0;
    float forward_curr      = 0.0;

    uint32_t turn_state     = 0;
    uint32_t turn_steps     = 0;
    float turn_control      = 0.2;
    float turn_angle        = 30.0*PI/180.0;
    float turn_curr         = 0.0;

    uint32_t wait_steps     = 200; 
    float speed_max = 1500.0*2.0*PI/60.0;

    uint32_t steps = 0; 

    while (true)
    {
        int32_t time_start = timer.get_time();

        //obtain current state
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        //forward and backward moving

        // wait state
        if (forward_state == 0) 
        {
            forward_curr = 0;
            forward_steps++;
            if (forward_steps > wait_steps)
            {
                forward_steps = 0;
                forward_state = 1;
            }
        }
        //move forward
        else if (forward_state == 1)
        {
            forward_curr = forward_control;
            if (distance > forward_distance)
            {
                forward_state = 2;
            }
        }
        // wait state
        else if (forward_state == 2)
        {
            forward_curr = 0;
            forward_steps++;
            if (forward_steps > wait_steps)
            {
                forward_steps = 0;
                forward_state = 3;
            }
        }
        else if (forward_state == 3)
        // move backward
        {
            forward_curr = -forward_control;
            if (distance < -forward_distance)
            {
                forward_state = 0;
            }
        }


      
        
        //turn moving

        // wait state
        if (turn_state == 0) 
        {
            turn_curr = 0;
            turn_steps++;
            if (turn_steps > wait_steps)
            {
                turn_steps = 0;
                turn_state = 1;
            }
        }
        //move turn
        else if (turn_state == 1)
        {
            turn_curr = turn_control;
            if (angle > turn_angle) 
            {
                turn_state = 2;
            }
        }
        // wait state
        else if (turn_state == 2)
        {
            turn_curr = 0;
            turn_steps++;
            if (turn_steps > wait_steps)
            {
                turn_steps = 0;
                turn_state = 3;
            }
        }
        else if (turn_state == 3)
        // move backward
        {
            turn_curr = -turn_control;
            if (angle < -turn_angle)
            {
                turn_state = 0;
            }
        }




      
        // compute shaped output
        float forward_curr_shaped = forward_shaper.step(forward_curr);
        float turn_curr_shaped    = turn_shaper.step(turn_curr);

        float u_left  = forward_curr_shaped - turn_curr_shaped;
        float u_right = forward_curr_shaped + turn_curr_shaped; 

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0);

        // send to motors
        motor_control.set_velocity(u_left*speed_max, u_right*speed_max);

        terminal << steps << " " << forward_curr << " " << turn_curr << " " << forward_curr_shaped << " " << turn_curr_shaped << " " << distance << " " <<  angle << " " << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        //compute delay time
        int32_t time_wait = 4 - (time_stop - time_start);
        if (time_wait < 0)
        {
            time_wait = 1;
        }

        timer.delay_ms(time_wait);
    }
}
*/

/*
void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;

    float shaper_ramp   = 0.004;

    //max speed, 1500rpm
    float speed_max     = 1500*2.0*PI/60.0; 

    float required_forward[] = {0.0, 100.0, 0.0, -100.0, 0.0,  0.0,  0.0,   0.0,  0.0,  100.0, 0.0,  -100.0,  0.0,  -100.0,  0.0,  100.0};
    float required_turn[]    = {0.0, 0.0,   0.0,    0.0, 0.0,  90.0, 0.0,  -90.0, 0.0,  90.0, 0.0,    -90.0,  0.0,    90.0,  0.0,  -90.0};

   
    //controllers
    PID pid_forward(0.008, 0.0, 0.01, 1.0);
    PID pid_turn(0.04, 0.0, 0.05, 1.0);

    //input shaper
    Shaper shaper_left, shaper_right;
    
    //shaper init
    shaper_left.init(shaper_ramp, -shaper_ramp); 
    shaper_right.init(shaper_ramp, -shaper_ramp); 

    uint32_t steps = 0;


    float d_time = 0.0;
    while (1)  
    {     
        int32_t time_start = timer.get_time();

        uint32_t idx = (steps/200)%16;

        //required values
        float req_forward = required_forward[idx];
        float req_turn    = required_turn[idx]*PI/180.0;

        //obtain current state
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        //compute controller
        float u_forward = pid_forward.step(req_forward, distance);
        float u_turn    = pid_turn.step(req_turn, angle);

        float u_left  = u_forward - u_turn;
        float u_right = u_forward + u_turn; 

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0);


        float u_left_shaped  = shaper_left.step(u_left);
        float u_right_shaped = shaper_right.step(u_right);

        motor_control.set_velocity(u_left_shaped*speed_max, u_right_shaped*speed_max);

        terminal << steps << " " << u_left << " " << u_right << " " << u_left_shaped << " " << u_right_shaped << " " << distance << " " <<  angle << " " << d_time << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        d_time = 0.9*d_time + 0.1*(time_stop - time_start);

        int32_t time_wait = 4 - (time_stop - time_start);

        if (time_wait < 0)
        {
            time_wait = 1;
        }

        timer.delay_ms(time_wait);
    }
}
*/