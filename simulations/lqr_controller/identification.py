import LibsControl
import numpy
import matplotlib.pyplot as plt

from utils.differential_robot import *


def servo_oscilations(model, u_turn_max, u_forward_max, theta_max, distance_max, steps):
    u_log           = []
    x_log           = []

    state_theta     = 0
    state_forward   = 0
    
    for n in range(steps):
        
        x = model.x

        
        theta    = x[1, 0]
        distance = x[3, 0]

        u = numpy.zeros((2, 1))

        if state_theta == 0:
            u_turn = u_turn_max
            if theta > theta_max:
                state_theta = 1
        else:
            u_turn = -u_turn_max
            if theta < -theta_max:
                state_theta = 0


        if state_forward == 0:
            u_forward = u_forward_max
            if distance > distance_max:
                state_forward = 1
        else:
            u_forward = -u_forward_max
            if distance < -distance_max:
                state_forward = 0


        u = numpy.zeros((2, 1))

        u[0, 0] = u_forward + u_turn
        u[1, 0] = u_forward - u_turn

        x, _ = model.forward(u)

        u_log.append(u[:, 0])
        x_log.append(x[:, 0])
    
    u_log   = numpy.array(u_log)
    x_log   = numpy.array(x_log)


    return u_log, x_log


if __name__ == "__main__":

    dt    = 1.0/250.0
    robot = DifferentialRobot(dt)

    steps = 10000

    
    u_turn_max = 1.3
    u_forward_max = 1.0
    theta_max = 1.1
    distance_max = 1.6

    u, x =  servo_oscilations(robot, u_turn_max, u_forward_max, theta_max, distance_max, steps)

    
    models, loss = LibsControl.identification(u, x, dt, steps_count=20, augmentations = [])

    model = models[7]

    ab      = model.T
    order   = x.shape[1]
    a_hat = ab[:, 0:order]
    b_hat = ab[:, order:]
    
    print("ground truth")
    print(numpy.round(robot.mat_a, 3))
    print(numpy.round(robot.mat_b, 3))
    print("\n\n")

    print("model")
    print(numpy.round(a_hat, 3))
    print(numpy.round(b_hat, 3))

    print(loss)

    '''
    plt.plot(u_log[:, 0], label="u0")
    plt.plot(u_log[:, 1], label="u1")
    plt.plot(theta_log, label="theta")
    plt.plot(distance_log, label="distance")
    plt.legend()
    plt.show()
    '''