#ifndef _LQR_H_
#define _LQR_H_

#include <stdint.h>
#include <matrix.h>


/*
LQR - linear quadratic regulator
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs>
class LQR
{

    public:
        void init(float *k, float *ki, float antiwindup)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
              
            this->k.from_array(k);
            this->ki.from_array(ki);

            this->integral_action.init();

            this->antiwindup = antiwindup;
        }

        void step()
        { 
            // integral action  
            auto error = this->xr - this->x;
            auto integral_action_new = this->integral_action + this->ki*error;

            //LQR controll law 
            auto u_new = this->k*this->x*(-1.0) + this->integral_action;

            //antiwindup with conditional integration
            this->u = u_new.clip(-antiwindup, antiwindup);
            
            this->integral_action = integral_action_new - (u_new - this->u);
        } 

    


    public:
        Matrix<float, system_order, 1> x;
        Matrix<float, system_order, 1> xr;
        Matrix<float, system_inputs, 1>  u;

    private:
        float antiwindup;

        //controller gain
        Matrix<float, system_inputs, system_order> k;
        Matrix<float, system_inputs, system_order> ki;

    public:
        Matrix<float, system_inputs, 1> integral_action;
};


#endif