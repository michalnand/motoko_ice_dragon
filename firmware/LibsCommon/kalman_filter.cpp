#include "kalman_filter.h"

void KalmanFilter::init()
{
    x_hat   = 0.0;
    p       = 0.5;
}
 
/*
    z   - position measurement
    pz  - position measurement uncertaininty (variance)
*/
float KalmanFilter::step(float z, float pz)
{
    //1, prediction
    //predict the state and uncertaininty
    //considering constat state model
    x_hat  = x_hat;

    //2, kalman gain
    float k     = p/(p + pz);
    
    //3, update
    x_hat  = x_hat + k*(z - x_hat);
    p      = (1.0 - k)*p;
    
    return x_hat;
}
