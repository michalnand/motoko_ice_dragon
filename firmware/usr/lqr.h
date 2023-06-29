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
            for (unsigned int j = 0; j < inputs_count; j++)
            {
                float u_sum = 0.0;
                for (unsigned int i = 0; i < system_order; i++)
                {
                    u_sum+= -k[i + j*system_order + ofs]*x[i] + ki[i * j*system_order + ofs]*error_sum[i];
                }

                u[j] = u_sum;
            }


            //antiwindup
            //raising error_sum is no more effecting u,
            //since u is out of range 
            for (unsigned int i = 0; i < inputs_count; i++)
            {
                if (u[i] > antiwindup)
                {
                    u[i] = antiwindup;
                    u[i]-= error_sum[i];
                }
                
                if (u[i] < -antiwindup)
                {
                    u[i] = -antiwindup;
                    u[i]-= error_sum[i];
                }
            }
        }


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

    private:
        float k[parallel_count*inputs_count*system_order];
        float ki[parallel_count*inputs_count*system_order];

        float dt;
        float antiwindup;

    private:
        float error_sum[system_order];

    public:
        //required state
        float xr[system_order];

        //system state
        float x[system_order];
        
        //controller output
        float u[inputs_count];
};

#endif
