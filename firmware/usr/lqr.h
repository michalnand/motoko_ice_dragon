#ifndef _LQR_H_
#define _LQR_H_

/*
k=
 [[ 0.93947  7.54465  7.5173  37.63855]
 [-0.93947 -7.54465  7.5173  37.63855]] 

ki=
 [[  0.       22.36068   0.       70.71068]
 [  0.      -22.36068   0.       70.71068]] 
*/

template <unsigned int outputs_count, unsigned int system_order>
class LQR
{   
    public:
        LQR()
        {

        }

        virtual ~LQR()
        {

        } 

        void init(float *k, float *ki, float antiwindup, float dt)
        {
            for (unsigned int j = 0; j < outputs_count*system_order; j++)
            {
                this->k[j] = k[j];
            }

            for (unsigned int j = 0; j < outputs_count*system_order; j++)
            {
                this->ki[j] = ki[j];
            }
           
            for (unsigned int j = 0; j < outputs_count; j++)
            {
                this->integral_action[j] = 0.0;
            }

            for (unsigned int j = 0; j < outputs_count; j++)
            {
                this->u[j] = 0.0;
            }

            this->antiwindup = antiwindup;
            this->dt         = dt;
        }

        void step()
        {
            //integral action
            //error = xr - x
            //integral_action+= ki@error * dt
            float integral_action_new[outputs_count];

            for (unsigned int j = 0; j < outputs_count; j++)
            {
                float sum = 0.0;
                for (unsigned int i = 0; i < system_order; i++)
                {
                    float error = this->xr[i] - this->x[i]; 
                    sum+= this->ki[j*system_order + i]*error;
                }

                integral_action_new[j] = this->integral_action[j] + sum*this->dt;
            } 
            

            //LQR controller with integral action
            //u = -k@x + ki@error_sum
            for (unsigned int j = 0; j < outputs_count; j++)
            {
                //control law
                float u_sum = 0.0;
                for (unsigned int i = 0; i < system_order; i++)
                {
                    u_sum+= -this->x[i]*this->k[j*system_order + i] + this->integral_action[j];
                }
                
                //antiwindup with conditional integration
                this->u[j] = _clip(u_sum, -antiwindup, antiwindup);
 
                if (_abs(u_sum - this->u[j]) <= 10e-10)
                {
                    this->integral_action[j] = integral_action_new[j];
                } 
            }
        }


    private:
        float _abs(float v)
        {
            if (v < 0)
            {
                v = -v;
            }

            return v;
        }

        float _clip(float v, float min_v, float max_v)
        {
            if (v < min_v)
            {
                v = min_v;
            }
            else if (v > max_v)
            {
                v = max_v;
            }

            return v;
        }


    public:
        float k[outputs_count*system_order];
        float ki[outputs_count*system_order];

        float dt;
        float antiwindup;

    public:
        float integral_action[outputs_count];

    public:
        //required state
        float xr[system_order];

        //system state
        float x[system_order];
        
        //controller output
        float u[outputs_count];
};

#endif
