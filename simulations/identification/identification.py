import LibsControl
import numpy
import matplotlib.pyplot as plt


def load_from_file(file_name):
    v = numpy.loadtxt(file_name)

    u = numpy.expand_dims(v[:, 1], axis=1)
    x = v[:, 2:]

    
    return u, x, x.shape[0] 

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

    #obtain response
    #u_result, x_result, steps = load_from_file("./data/turn_4.log")
    u_result, x_result, steps = load_from_file("./data/forward_1.log")
    
    t_result    = numpy.arange(steps)*dt

      
    #identification
    x_denoised, fft_result= LibsControl.fft_denoising(x_result, [100.0, 10.0])
    models, loss    = LibsControl.identification(u_result, x_denoised, dt)

    model   = models[2]
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


    print(mat_a_hat)
    print(mat_b_hat)


    ds_hat = LibsControl.dynamical_system.DynamicalSystem(mat_a_hat, mat_b_hat, dt = dt)
    x_hat_result = plant_response(ds_hat, u_result)

   
    fig, axs = plt.subplots(3, 1, figsize=(8, 8))

    axs[0].plot(t_result, u_result[:, 0], label="input u", color="purple")
    axs[0].set_xlabel("time [s]")
    axs[0].set_ylabel("motor control")
    axs[0].grid()

    axs[1].plot(t_result, x_result[:, 0],   label="observed", color="red", alpha=0.5)
    axs[1].plot(t_result, x_denoised[:, 0],   label="denoised", color="red", lw=2.0)
    axs[1].plot(t_result, x_hat_result[:, 0],     label="model", color="deepskyblue", lw=2.0)
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("angular velocity")
    axs[1].legend()
    axs[1].grid()   

    axs[2].plot(t_result, x_result[:, 1],   label="observed", color="red", alpha=0.5)
    axs[2].plot(t_result, x_denoised[:, 1],   label="denoised", color="red", lw=2.0)
    axs[2].plot(t_result, x_hat_result[:, 1],     label="model", color="deepskyblue", lw=2.0)
    axs[2].set_xlabel("time [s]")
    axs[2].set_ylabel("angle")
    axs[2].legend()
    axs[2].grid()


    plt.tight_layout()
    plt.show()



    '''
    q = [0.0, 1.0]
    q = numpy.diag(q)

  
    r = [0.00001] 
    r =  numpy.diag(r)


    mat_c_hat = numpy.eye(2)
  
    lqri     = LibsControl.LQRISolver(mat_a_hat, mat_b_hat, mat_c_hat, q, r, dt)

    k, ki    = lqri.solve() 
    
    #print solved controller matrices
    print("controller\n\n")
    print("k=\n", numpy.round(k, 5), "\n")
    print("ki=\n", numpy.round(ki, 5), "\n")
    print("\n\n")


    steps       = 500
    t_result    = numpy.arange(steps)*dt

    #required state 
    yr = numpy.array([[0.0, 90.0*numpy.pi/180.0]]).T

    #step response
    u_result, x_result, y_result = lqri.closed_loop_response(yr, steps, noise = 0, disturbance = True)

    u_result[:, :]*= 60.0/numpy.pi
    x_result[:, 0]*= 180.0/numpy.pi
    x_result[:, 1]*= 180.0/numpy.pi
    
    LibsControl.plot_closed_loop_response(t_result, u_result, x_result, file_name = "closed_loop_response.png", u_labels = ["u [rpm]"], x_labels = ["dtheta [deg/s]", "theta [deg]"] )
    '''
