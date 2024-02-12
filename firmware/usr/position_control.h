#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

class PositionControl
{   
    public:
        void init(float du_p_max, float du_n_max, uint32_t dt_us);
        
        //x      required position
        //theta  required theta
        void step(float x, float theta);


        void callback_position();

    private:
        float du_p_max;     
        float du_n_max;    
        float u_left_curr;  
        float u_right_curr; 

        float u_left_saturation;
        float u_right_saturation;

        float x, theta;
};

#endif