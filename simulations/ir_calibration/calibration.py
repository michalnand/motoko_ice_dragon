import LibsControl
import numpy
import matplotlib.pyplot as plt


def print_result(model):

    str_res = "y = "
    for i in range(model.shape[0]):
        str_res+= str(round(model[i][0], 10))

        if i == 0:
            str_res+= str(i) 
        else:
            str_res+= "*x^" + str(i) 
        
        if i != model.shape[0]-1:
            str_res+= " + "
    
    print(str_res)
    print("\n\n")

    print(model[:, 0])
    print("\n\n")


def interpolate(x, inter_points):

    count       = x.shape[0]*inter_points-1
    features    = x.shape[1]
    result      = numpy.zeros((count, features))

    for i in range(count):

        idx_a = i//inter_points
        idx_b = (i+1)//inter_points

        alpha = (i%inter_points)/(inter_points-1)

        result[i,:] = x[idx_a, :]*(1 - alpha) + x[idx_b, :]*alpha
        
    return result


if __name__ == "__main__":


    front_left_sensor = []
    front_left_sensor.append([0,    430])
    front_left_sensor.append([20,   508])
    front_left_sensor.append([50,   915])
    front_left_sensor.append([80,   1820])
    front_left_sensor.append([100,  2170])
   
    
    front_right_sensor = []
    front_right_sensor.append([0,   415])
    front_right_sensor.append([20,  470])
    front_right_sensor.append([50,  580])
    front_right_sensor.append([80,  1300])
    front_right_sensor.append([100, 1630])


    left_sensor = []
    left_sensor.append([0,    335])
    left_sensor.append([20,   390])
    left_sensor.append([50,   800])
    left_sensor.append([80,   1720])
    left_sensor.append([100,  2060])



    right_sensor = []
    right_sensor.append([0,    380])
    right_sensor.append([20,   430])
    right_sensor.append([50,   950])
    right_sensor.append([80,   1460])
    right_sensor.append([100,  1640])
   
    
    data_xy  = numpy.array(front_right_sensor)

    data_x = numpy.expand_dims(data_xy[:, 1], axis=1)
    data_y = numpy.expand_dims(data_xy[:, 0], axis=1)

    data_x_int = interpolate(data_x, 20)

    print(data_x)
    print(data_y)

    max_order = 3


    theta = LibsControl.polynom_fit(data_y, data_x, max_order)

    print_result(theta)
    

    data_y_hat = LibsControl.polynom_predict(data_x_int, theta)


    plt.plot(data_x, data_y,     label="data", color="blue", alpha=0.75)
    plt.plot(data_x_int, data_y_hat, label="prediction", color="red", alpha=0.75)
    plt.xlabel("reading")
    plt.ylabel("distance [mm]")
    plt.legend()
    plt.show()
