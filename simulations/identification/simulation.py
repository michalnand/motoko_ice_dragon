import LibsControl
import numpy
import matplotlib.pyplot as plt


def servo_oscilations(model, u_max, x_max, steps):
    u_result = []
    x_result = []

    u = numpy.ones((1, 1))*u_max

    x = numpy.zeros((2, 1))
    for n in range(steps):
        theta    = x[1, 0]

        
        if theta > x_max:
            u = -u_max*numpy.ones((1, 1))
        elif theta < -x_max:
            u = u_max*numpy.ones((1, 1))
        
        x, _ = model.forward(x, u) 

        u_result.append(u[:, 0])
        x_result.append(x[:, 0])
    
    u_result   = numpy.array(u_result)
    x_result   = numpy.array(x_result)
    

    return u_result, x_result


def plant_response(model, u_result):
    x_result = []

    x = numpy.zeros((2, 1))
    for n in range(steps):

        u    = u_result[n, :]
        u    = numpy.expand_dims(u, axis=1)
        x, _ = model.forward(x, u) 

        x_result.append(x[:, 0])


    x_result   = numpy.array(x_result)
    return x_result




if __name__ == "__main__":


    dt          = 1.0/250.0
    steps       = 4000
    t_result    = numpy.arange(steps)*dt


    tau = 0.25
    k = 2.9


    #fill matrices
    mat_a = numpy.zeros((2, 2))
    mat_b = numpy.zeros((2, 1))

    mat_a[0][0] = -1.0/tau
    mat_a[0][1] = 0.0
    mat_a[1][0] = 1.0
    mat_a[1][1] = 0.0

    mat_b[0][0] = k/tau

    
    print("reference :")
    print("tau_ref = ", tau)
    print("k_ref   = ", k)
    print("\n\n")

    ds = LibsControl.dynamical_system.DynamicalSystem(mat_a, mat_b, dt = dt)
    
    #obtain response
    u_result, x_result = servo_oscilations(ds, 1.0, 1.0, steps)

    #add noise
    x_noised = x_result.copy()
    x_noised[:, 0]+= 0.5*numpy.random.randn(x_result.shape[0])
    x_noised[:, 1]+= 0.5*numpy.random.randn(x_result.shape[0])
    x_noised[:, 1]+= 2.0*numpy.arange(x_noised.shape[0])/x_noised.shape[0]
    
    #identification
    x_denoised = LibsControl.fft_denoising(x_noised, [100.0, 300.0])
    models, loss    = LibsControl.identification(u_result, x_denoised, dt)

    model   = models[-1]
    ab      = model.T

    order   = x_result.shape[1]
    a = ab[0, 0]
    b = ab[0, 2]

    tau_hat = -1.0/a
    k_hat   = b*tau_hat

    #fill matrices
    mat_a_hat = numpy.zeros((2, 2))
    mat_b_hat = numpy.zeros((2, 1))

    mat_a_hat[0][0] = -1.0/tau_hat
    mat_a_hat[0][1] = 0.0
    mat_a_hat[1][0] = 1.0
    mat_a_hat[1][1] = 0.0

    mat_b_hat[0][0] = k_hat/tau_hat

   
    print("model : ")
    print("tau  = ", tau_hat)
    print("k    = ", k_hat)
    print("\n\n")


    ds_hat = LibsControl.dynamical_system.DynamicalSystem(mat_a_hat, mat_b_hat, dt = dt)

    x_hat_result = plant_response(ds_hat, u_result)

   
    fig, axs = plt.subplots(3, 1, figsize=(8, 8))

    axs[0].plot(t_result, u_result[:, 0], label="input u", color="purple")
    axs[0].set_xlabel("time [s]")
    axs[0].set_ylabel("motor control")
    axs[0].grid()

    axs[1].plot(t_result, x_noised[:, 0],   color="red", alpha=0.5)
    axs[1].plot(t_result, x_result[:, 0],   label="gt", color="red", lw=3.0)
    axs[1].plot(t_result, x_hat_result[:, 0],     label="model", color="deepskyblue", lw=2.0)
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("angular velocity")
    axs[1].legend()
    axs[1].grid()   

    axs[2].plot(t_result, x_noised[:, 1],   color="red", alpha=0.5)
    axs[2].plot(t_result, x_result[:, 1],   label="gt", color="red", lw=3.0)
    axs[2].plot(t_result, x_hat_result[:, 1],     label="model", color="deepskyblue", lw=2.0)
    axs[2].set_xlabel("time [s]")
    axs[2].set_ylabel("angle")
    axs[2].legend()
    axs[2].grid()


    plt.tight_layout()
    plt.show()