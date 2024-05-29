#ifndef _LQG_H_
#define _LQG_H_

#include <stdint.h>
#include <matrix.h>


/*
LQG - linear quadratic gaussian control with Kalman observer
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
system_outputs  : size of observed variables
*/
template<uint32_t system_order, uint32_t system_inputs, uint32_t system_outputs>
class LQG
{

    public:
        void init(float *a, float *b, float *c, float *k, float *ki, float *f, float antiwindup)
        {   
            this->y.init();
            this->yr.init();
            this->u.init();
              
            this->a.from_array(a);
            this->b.from_array(b);
            this->c.from_array(c); 

            this->k.from_array(k);
            this->ki.from_array(ki);
            this->f.from_array(f);

            this->integral_action.init();
            this->x_hat.init();

            this->antiwindup = antiwindup;
        }

        void step()
        { 
            // integral action  
            auto error = this->yr - this->y;
            auto integral_action_new = this->integral_action + this->ki*error;
            
            //LQR controll law 
            auto u_new = this->k*this->x_hat*(-1.0) + integral_action_new;

            //antiwindup with conditional integration
            this->u = u_new.clip(-antiwindup, antiwindup);
            this->integral_action = integral_action_new - (u_new - this->u);
            
            // kalman observer
            // only y is known, and using knowledge of dynamics, 
            // the full state x_hat can be reconstructed
            auto prediction_error = this->y - this->c*this->x_hat;
            this->x_hat = this->a*this->x_hat + this->b*this->u + this->f*prediction_error;
        }       

    
    public:
        //inputs and outputs
        Matrix<float, system_outputs, 1> y;
        Matrix<float, system_outputs, 1> yr;
        Matrix<float, system_inputs, 1>  u;

    private:
        float antiwindup;

        //plant dynamics
        Matrix<float, system_order, system_order> a;
        Matrix<float, system_order, system_inputs> b; 
        Matrix<float, system_outputs, system_order> c;

        //kalman gain
        Matrix<float, system_order, system_outputs> f; 
        
        //controller gain
        Matrix<float, system_inputs, system_order> k;
        Matrix<float, system_inputs, system_outputs> ki;

    public:
        //internal state
        Matrix<float, system_inputs, 1> integral_action;
        Matrix<float, system_order, 1> x_hat;
};


#endif