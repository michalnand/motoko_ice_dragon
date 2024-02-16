import LibsControl
import numpy
import matplotlib.pyplot as plt


def load_from_file(file_name):
    v = numpy.loadtxt(file_name)

    u = numpy.array(v[:, 1:3])
    x = numpy.array(v[:, 3:5])
    
    return u, x, x.shape[0] 

def velocity_augmentation(u, x):
    u_result = u[1:, :]
    v = x[0:-1, :] - x[1:, :]

    n_steps = u_result.shape[0]

    x_result = numpy.zeros((n_steps, 4))

    x0_sum = 0
    x1_sum = 0
    for n in range(n_steps):

        x_result[n, 0] = x0_sum
        x_result[n, 1] = x1_sum
        x_result[n, 2] = v[n, 0]
        x_result[n, 3] = v[n, 1]

        x0_sum = x0_sum + v[n, 0]
        x1_sum = x1_sum + v[n, 1]

    return u_result, x_result

if __name__ == "__main__":
    dt          = 1.0/250.0

    #obtain response
    u_result, x_result, steps = load_from_file("./data/run_0.log")

    u_result, x_result = velocity_augmentation(u_result, x_result)

    n_steps  = u_result.shape[0]
    n_inputs = u_result.shape[1]
    n_states = x_result.shape[1]

    r = 0.1*numpy.eye(n_states)
    q = 0.1*numpy.eye(n_states + n_inputs)

    print("u_result = ", u_result.shape)
    print("x_result = ", x_result.shape)
  
    a_est, b_est = LibsControl.recursive_kalman_ls_identification(u_result, x_result, r, q, True)
    

    a_est[0, :] = 0
    a_est[1, :] = 0
    a_est[0][0] = 1
    a_est[1][1] = 1
    print(a_est)
    print(b_est)

    print(numpy.linalg.eigvals(a_est))

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

    steps_max = 1000


    axs[0].plot(u_result[0:steps_max, 0], color="purple")
    axs[0].set_xlabel("time [s]")
    axs[0].set_ylabel("control left")
    axs[0].grid()

    axs[1].plot(u_result[0:steps_max, 1], color="purple")
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("control right")
    axs[1].grid()


    axs[2].plot(x_result[0:steps_max, 0], color="red")
    axs[2].plot(x_hat_result[0:steps_max, 0], color="blue")
    axs[2].set_xlabel("time [s]")
    axs[2].set_ylabel("position left")
    axs[2].grid()

    axs[3].plot(x_result[0:steps_max, 1], color="red")
    axs[3].plot(x_hat_result[0:steps_max, 1], color="blue")
    axs[3].set_xlabel("time [s]")
    axs[3].set_ylabel("position right")
    axs[3].grid()

    axs[4].plot(x_result[0:steps_max, 2], color="red")
    axs[4].plot(x_hat_result[0:steps_max, 2], color="blue")
    axs[4].set_xlabel("time [s]")
    axs[4].set_ylabel("velocity left")
    axs[4].grid()

    axs[5].plot(x_result[0:steps_max, 3], color="red")
    axs[5].plot(x_hat_result[0:steps_max, 3], color="blue")
    axs[5].set_xlabel("time [s]")
    axs[5].set_ylabel("velocity right")
    axs[5].grid()



    fig.legend()

    plt.tight_layout()

    plt.savefig("result.png", dpi = 300)
