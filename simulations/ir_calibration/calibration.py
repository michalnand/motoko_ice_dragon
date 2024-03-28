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

    #robot A
    front_left_sensor = []
    front_left_sensor.append([30,    400])
    front_left_sensor.append([50,   1620])
    front_left_sensor.append([70,   2320])
    front_left_sensor.append([100,   2940])
    front_left_sensor.append([120,  3190])
    front_left_sensor.append([150,  3390])
   
    front_right_sensor = []
    front_right_sensor.append([30,   710])
    front_right_sensor.append([50,  2000])
    front_right_sensor.append([70,  2620])
    front_right_sensor.append([100,  3110])
    front_right_sensor.append([120, 3300])
    front_right_sensor.append([150, 3530])

    left_sensor = []
    left_sensor.append([30,   250])
    left_sensor.append([50,  290])
    left_sensor.append([70,  500])
    left_sensor.append([100,  1150])
    left_sensor.append([120, 1430])
    left_sensor.append([150, 1720])


    right_sensor = []
    right_sensor.append([30,   240])
    right_sensor.append([50,  300])
    right_sensor.append([70,  700])
    right_sensor.append([100,  1320])
    right_sensor.append([120, 1560])
    right_sensor.append([150, 1780])







    #robot B
    front_left_sensor = []
    front_left_sensor.append([30,    262])
    front_left_sensor.append([50,   780])
    front_left_sensor.append([70,   1820])
    front_left_sensor.append([100,   2600])
    front_left_sensor.append([120,  2940])
    front_left_sensor.append([150,  3240])
   
    front_right_sensor = []
    front_right_sensor.append([30,   360])
    front_right_sensor.append([50,  1550])
    front_right_sensor.append([70,  2300])
    front_right_sensor.append([100,  2900])
    front_right_sensor.append([120, 3120])
    front_right_sensor.append([150, 3340])

    left_sensor = []
    left_sensor.append([30,   265])
    left_sensor.append([50,  620])
    left_sensor.append([70,  1380])
    left_sensor.append([100,  1950])
    left_sensor.append([120, 2250])
    left_sensor.append([150, 2500])


    right_sensor = []
    right_sensor.append([30,   290])
    right_sensor.append([50,  1200])
    right_sensor.append([70,  1980])
    right_sensor.append([100,  2580])
    right_sensor.append([120, 2830])
    right_sensor.append([150, 3120])

    
    data_xy  = numpy.array(right_sensor)

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
