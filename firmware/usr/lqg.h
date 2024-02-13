#ifndef _LQG_H_
#define _LQG_H_

#include <stdint.h>
#include <matrix.h>


/*
n : system order
m : system inputs count
k : systen outputs count
*/
template<uint32_t system_order, uint32_t system_inputs, uint32_t system_outputs>
class LQG
{

    public:
        void init()
        {   
            y.init();
            yr.init();
            u.init();
              
            a.init();
            b.init();
            c.init();

            f.init();
            k.init();
            ki.init();

            integral_action.init();
            x_hat.init();
        }

        void step()
        { 
            // - kalman observer
            // - only y is known, and using knowledge of dynamics, 
            // - the full state x_hat can be reconstructed
            auto prediction_error = y - c*x_hat;    //system_outputs + 2*system_outputs*system_order
            x_hat = a*x_hat + b*u;                  //2*system_order*system_order + 2*system_order*system_inputs

            // integral action
            auto error = yr - y;                    //system_outputs
            integral_action = integral_action + ki*error; //system_inputs + 2*system_inputs*system_outputs

            // LQR controll law
            u = (k*x_hat)*(-1.0) + integral_action; //2*system_inputs*system_order + system_inputs
        }


    private:
        Matrix<float, system_outputs, 1> y;
        Matrix<float, system_outputs, 1> yr;
        Matrix<float, system_inputs, 1> u;

        //plant dynamics
        Matrix<float, system_order, system_order> a;
        Matrix<float, system_order, system_inputs> b; 
        Matrix<float, system_outputs, system_order> c;

        //kalman gain
        Matrix<float, system_order, system_outputs> f; 
        
        //controller gain
        Matrix<float, system_inputs, system_order> k;
        Matrix<float, system_inputs, system_outputs> ki;

    private:
        Matrix<float, system_inputs, 1> integral_action;
        Matrix<float, system_order, 1> x_hat;
};


#endif