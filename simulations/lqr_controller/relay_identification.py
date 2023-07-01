import LibsControl
import numpy
import matplotlib.pyplot as plt

from utils.differential_robot import *


def relay_tunning(model, u_max, y_max, steps):
    u = numpy.ones((1, 1))*u_max
    x = numpy.zeros((2, 1))

    u_log = []
    x_log = []

    state = 0

    zero_crossing = 0

    amplitude = 0.0

    for n in range(steps):
        #servo step
        x, _ = model.forward(x, u)

        
        #relay oscilator
        if x[1][0] >= y_max and state == 0:
            u =  -numpy.ones((1, 1))*u_max
            zero_crossing+=1
            state = 1 
        elif x[1][0] <= -y_max and state == 1:
            u = numpy.ones((1, 1))*u_max
            zero_crossing+=1
            state = 0


        if abs(x[0, 0]) > amplitude:
            amplitude = abs(x[0, 0])
        
        u_log.append(u[:, 0])
        x_log.append(x[:, 0])

    u_log   = numpy.array(u_log)
    x_log   = numpy.array(x_log)

    r = amplitude/u_max
    t = model.dt*steps/(zero_crossing + 10**-6)


    return u_log, x_log, r, t


if __name__ == "__main__":

    dt    = 1.0/250.0

    k_   = [1.0] #0.1, 1.0, 10.0, 20.0]
    taus = [0.1, 0.5, 1.0, 2.0, 5.0]
    

    for k in k_:
        for tau in taus:
            #2nd order servo model
            #tau = 0.5
            #k   = 20.1

            mat_a = numpy.zeros((2, 2))
            mat_b = numpy.zeros((2, 1))
            mat_c = numpy.eye(2)
            

            mat_a[0][0] = -1.0/tau
            mat_a[1][0] = 1.0
            mat_b[0][0] = k/tau

            model = LibsControl.DynamicalSystem(mat_a, mat_b, mat_c, dt)

            u_max = 1.0
            y_max = 1.0
            steps = 100000

            u_log, x_log, r, t = relay_tunning(model, u_max, y_max, steps)


            print(k, tau, round(r, 5), round(t, 5) )
    
    '''
    print(u_log.shape)
    print(x_log.shape)

    
    plt.plot(u_log[:, 0], label="u")
    plt.plot(x_log[:, 0], label="omega rad/s")
    plt.plot(x_log[:, 1], label="angle rad")
    plt.legend()
    plt.show()
    '''
    
    