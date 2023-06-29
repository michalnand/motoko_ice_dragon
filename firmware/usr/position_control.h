#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include <drivers.h>
#include <lqr.h>

#define POSITION_CONTROL_DT     ((int32_t)4)

class PositionControl
{
    public:
        void init();
        void set(float dtheta, float dx);
        void set_state(uint32_t idx, float x);

        void callback();

    private:
        LQR<2, 4> lqr;
};


#endif