#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include <drivers.h>
#include <lqr.h>

#define POSITION_CONTROL_DT     ((int32_t)4)

class PositionControl
{
    public:
        void init();
        void set_required(float dtheta, float dx);
        void set_state(uint32_t idx, float x);

        void callback();

    private:
        bool line_following_mode;
        LQR<2, 4> lqr;
};


#endif