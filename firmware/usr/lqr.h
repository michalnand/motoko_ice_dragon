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

template <unsigned int inputs_count, unsigned int system_order, unsigned int parallel_count = 1>
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
            for (unsigned int j = 0; j < parallel_count*inputs_count*system_order; j++)
            {
                this->k[j] = k[j];
            }

            for (unsigned int j = 0; j < parallel_count*inputs_count*system_order; j++)
            {
                this->ki[j] = ki[j];
            }
           
            for (unsigned int j = 0; j < system_order; j++)
            {
                error_sum[j] = 0.0;
            }

            for (unsigned int j = 0; j < inputs_count; j++)
            {
                u[j] = 0.0;
            }

            this->antiwindup = antiwindup;
            this->dt         = dt;
        }



        void step(unsigned int controller_id = 0)
        {
            unsigned int ofs = controller_id*inputs_count*system_order;

            //integral action error
            //error     = xr - x
            //error_sum = error_sum + error*dt
            for (unsigned int i = 0; i < system_order; i++)
            {
                float e = xr[i] - x[i];

                error_sum[i] = error_sum[i] + e*dt;
            }
 
            //apply controll law
            //u = -k@x + ki@error_sum

            /*
            for (unsigned int j = 0; j < inputs_count; j++)
            {
                float u_sum = 0.0;
                for (unsigned int i = 0; i < system_order; i++)
                {
                    u_sum+= -k[i + j*system_order + ofs]*x[i];
                    u_sum+= ki[i * j*system_order + ofs]*error_sum[i];
                }  
 
                u[j] = u_sum;
            }
            */
            
            float u_sum = 0.0;
            u_sum+= -k[0]*x[0];
            u_sum+= -k[1]*x[1];
            u_sum+= ki[0]*error_sum[0];
            u_sum+= ki[1]*error_sum[1];
            u_tmp[0] = u_sum;

            for (unsigned int i = 0; i < inputs_count; i++)
            {
                u[i] = u_tmp[i];
                if (u[i] > antiwindup)
                {
                    u[i] = antiwindup;
                }

                if (u[i] < -antiwindup)
                {
                    u[i] = -antiwindup;
                }
            }

            //antiwindup
            //raising error_sum is no more effecting u,
            //since u is out of range 
            for (unsigned int i = 0; i < inputs_count; i++)
            {
                float e = u[i] - u_tmp[i];
                error_sum[1]+= e*dt;
            }

        }

        /*
        void copy(LQR<inputs_count, system_order, parallel_count> &other)
        {
            for (unsigned int j = 0; j < parallel_count*inputs_count*system_order; j++)
            {
                this->k[j] = other.k[j];
            }

            for (unsigned int j = 0; j < parallel_count*inputs_count*system_order; j++)
            {
                this->ki[j] = other.ki[j];
            }

            this->antiwindup = other.antiwindup;
            this->dt         = other.dt;

            copy_state(other);
        }

        void copy_state(LQR<inputs_count, system_order, parallel_count> &other)
        {
            for (unsigned int j = 0; j < system_order; j++)
            {
                xr[j] = other.xr[j];
            }

            for (unsigned int j = 0; j < system_order; j++)
            {
                x[j] = other.x[j];
            }
           
            for (unsigned int j = 0; j < system_order; j++)
            {
                error_sum[j] = other.error_sum[j];
            }

            for (unsigned int j = 0; j < inputs_count; j++)
            {
                u[j] =other.u[j];
            }
        }
        */

    public:
        float k[parallel_count*inputs_count*system_order];
        float ki[parallel_count*inputs_count*system_order];

        float dt;
        float antiwindup;

    public:
        float error_sum[system_order];

    public:
        //required state
        float xr[system_order];

        //system state
        float x[system_order];
        
        //controller output
        float u_tmp[inputs_count];
        float u[inputs_count];
};

#endif
