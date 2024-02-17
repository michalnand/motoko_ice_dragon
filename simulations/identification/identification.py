import LibsControl
import numpy
import matplotlib.pyplot as plt

import data_creator

def load_from_file(file_name):
    v = numpy.loadtxt(file_name)

    u = numpy.array(v[:, 1:3])
    x = numpy.array(v[:, 3:5])

    dt= v[:, -1].mean()
    t = numpy.arange(x.shape[0])*dt*0.001
    
    return u, x, t

def difference(x):
    #add velocity terms
    x_pad = numpy.zeros((1, x.shape[1]))
    x_tmp = numpy.concatenate([x_pad, x], axis=0)

    #estimate velocity term
    dx = x_tmp[1:, :]  - x_tmp[0:-1, :]

    return dx


def state_augmentation(x, wheel_diameter = 34.0*0.001, robot_brace = 80.0*0.001):

    n_steps = x.shape[0]

    distance = 0.5*(x[:, 0] + x[:, 1])*wheel_diameter
    angle    = (x[:, 0] - x[:, 1])*wheel_diameter / robot_brace

    distance = distance - distance[0]
    angle    = angle - angle[0]

    x_aug = numpy.zeros((n_steps, 2))
    x_aug[:, 0] = distance
    x_aug[:, 1] = angle

    vel = difference(x_aug[:, 0:2])

    x_aug = numpy.concatenate([x_aug, vel], axis=1)

    return x_aug


if __name__ == "__main__":
    
    '''
    n_steps = 5000
    k_forward   = 1.35
    k_turn      = 1.17
    tau_forward = 0.9
    tau_turn    = 0.8
  
    u_result, x_result = data_creator.create_data(n_steps, k_forward, k_turn, tau_forward, tau_turn)
    x_result = x_result[:, 0:2]
    x_result = x_result + 1.0*numpy.random.randn(x_result.shape[0], x_result.shape[1])
    '''

    #obtain response
    u_result, x_result, t_result = load_from_file("./data/run_0.log")
    x_result = state_augmentation(x_result)

    print(u_result.shape, x_result.shape)
    
    
    n_steps  = u_result.shape[0]
    n_inputs = u_result.shape[1]
    n_states = x_result.shape[1]

    r = 1.0*numpy.eye(n_states)
    q = 0.0*numpy.eye(n_states + n_inputs)

  
    #a_est, b_est = LibsControl.recursive_kalman_ls_identification(u_result, x_result, r, q, False)
    a_est, b_est = LibsControl.recursive_ls_identification(u_result, x_result)
    
   
    print("mat_a = \n", a_est, "\n\n")
    print("mat_b = \n", b_est, "\n\n")

    poles = numpy.linalg.eigvals(a_est) + 0j

    

    ds = LibsControl.DynamicalSystemDiscrete(a_est, b_est, None)    

    x_initial = numpy.expand_dims(x_result[0], axis=0).T
    ds.reset(x_initial)

    x_hat_result = []

    for n in range(n_steps):

        u = u_result[n, :]
        u = numpy.expand_dims(u, axis=0).T

        x, y = ds.forward_state(u)

        x_hat_result.append(x[:, 0])

    x_hat_result = numpy.array(x_hat_result)

    plt.clf()

    fig, axs = plt.subplots(6, 1, figsize=(8, 2*5))


    steps_start = 0 #n_steps - 1000
    steps_max   = 4000 #n_steps 


    axs[0].plot(t_result[steps_start:steps_max], u_result[steps_start:steps_max, 0], color="purple")
    axs[0].set_xlabel("time [s]")
    axs[0].set_ylabel("control left")
    axs[0].grid()

    axs[1].plot(t_result[steps_start:steps_max], u_result[steps_start:steps_max, 1], color="purple")
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("control right")
    axs[1].grid()


    axs[2].plot(t_result[steps_start:steps_max], x_result[steps_start:steps_max, 0], label="ground truth", color="red")
    axs[2].plot(t_result[steps_start:steps_max], x_hat_result[steps_start:steps_max, 0], label="prediction", color="blue")
    axs[2].set_xlabel("time [s]")
    axs[2].set_ylabel("distance [m]")
    axs[2].legend()
    axs[2].grid()

    axs[3].plot(t_result[steps_start:steps_max], x_result[steps_start:steps_max, 1], color="red")
    axs[3].plot(t_result[steps_start:steps_max], x_hat_result[steps_start:steps_max, 1], color="blue")
    axs[3].set_xlabel("time [s]")
    axs[3].set_ylabel("angle [rad]")
    axs[3].grid()

    
    axs[4].plot(t_result[steps_start:steps_max], x_result[steps_start:steps_max, 2], color="red")
    axs[4].plot(t_result[steps_start:steps_max], x_hat_result[steps_start:steps_max, 2], color="blue")
    axs[4].set_xlabel("time [s]")
    axs[4].set_ylabel("velocity")
    axs[4].grid()

    axs[5].plot(t_result[steps_start:steps_max], x_result[steps_start:steps_max, 3], color="red")
    axs[5].plot(t_result[steps_start:steps_max], x_hat_result[steps_start:steps_max, 3], color="blue")
    axs[5].set_xlabel("time [s]")
    axs[5].set_ylabel("angular velocity")
    axs[5].grid()

    

    fig.legend()

    plt.tight_layout()

    plt.savefig("result.png", dpi = 300)
    

    #plot_poles_cl(re_ol, im_ol, re_cl, im_cl, "poles_png")


    c_est = numpy.eye(a_est.shape[0])

    #process and observation noise covariance
    q_noise = 0.1*numpy.eye(a_est.shape[0]) 
    r_noise = 0.1*numpy.eye(c_est.shape[0]) 


    #create loss weighting matrices (diagonal)
    q = numpy.diag([1.0, 1.0, 0.0, 0.0] )
    r = numpy.diag( [1.0, 1.0]) 

   

    #solve LQG controller
    lqg = LibsControl.LQGDiscrete(a_est, b_est, c_est, q, r, q_noise, r_noise)

    print("controller")
    print("k  = \n", lqg.k, "\n")
    print("ki = \n", lqg.ki, "\n")
    print("f  = \n", lqg.f, "\n")
