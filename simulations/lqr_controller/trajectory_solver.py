import LibsControl
import numpy

class TrajectorySolver:

    def __init__(self, mat_a, mat_b, mat_c, qa, qb, r, dt, controllers_count):

        self.mat_a = mat_a
        self.mat_b = mat_b
        self.mat_c = mat_c

        self.n = self.mat_a.shape[0]  #system order
        self.m = self.mat_b.shape[1]  #inputs count
        self.k = self.mat_c.shape[0]  #outputs count

        self.controllers_count = controllers_count

        self.dt = dt


        #synthetise multiple controlers for different Q
        self.k_mat  = []
        self.ki_mat = []


        for i in range(controllers_count):
            k        = i/(controllers_count - 1)
            q        = (1.0 - k)*qa + k*qb
            lqri     = LibsControl.LQRISolver(mat_a, mat_b, mat_c, q, r, self.dt)

            k, ki    = lqri.solve() 

            self.k_mat.append(k)
            self.ki_mat.append(ki)

        self.k_mat  = numpy.array(self.k_mat)
        self.ki_mat = numpy.array(self.ki_mat)


    def predict(self, target_x, target_y, robot_x, robot_y, robot_theta, x_initial, error_sum_initial, steps):

        x           = numpy.zeros((self.controllers_count, self.n))
        x[:]        = x_initial[:, 0]

        y           = x@self.mat_c.T
        yr          = numpy.zeros((self.controllers_count, self.k))
        error_sum   = numpy.zeros((self.controllers_count, self.n))

        error_sum[:,:] = error_sum_initial[:, 0]

        x_pos = numpy.zeros((self.controllers_count,)) + robot_x
        y_pos = numpy.zeros((self.controllers_count,)) + robot_y
        theta = numpy.zeros((self.controllers_count,)) + robot_theta


        result = numpy.zeros((steps, self.controllers_count, 3))

        for n in range(steps):
            d_distance, d_theta = self._get_required(target_x, target_y, x_pos, y_pos, theta)

            yr[:, 1] = y[:, 1] + d_theta
            yr[:, 3] = y[:, 3] + d_distance


            #controller step
            u, error_sum = self._lqri_step(yr, y, x, error_sum)

            #plant step
            x, y = self._plant_step(x, u)


            #angular velocity
            w  = y[:, 0]
            #velocity
            v = y[:, 2]

            x_pos+= v*numpy.cos(theta)*self.dt
            y_pos+= v*numpy.sin(theta)*self.dt
            theta+= w*self.dt

            #wrap into 0 .. 2pi
            theta =  numpy.mod(theta, 2.0*numpy.pi)
            

            result[n, :, 0] = x_pos
            result[n, :, 1] = y_pos
            result[n, :, 2] = theta

        trajectory_eval = self._trajectory_eval(target_x, target_y, result)


        return result, trajectory_eval
           

    def _lqri_step(self, yr, y, x, error_sum):
        #compute error
        error     = yr - y

        #integral action
        error_sum_new = error_sum + error*self.dt

        #apply controll law
        x_tmp = numpy.expand_dims(x, 2)
        error_sum_new_tmp = numpy.expand_dims(error_sum_new, 2)

        u = -self.k_mat@x_tmp + self.ki_mat@error_sum_new_tmp
        u = u[:, :, 0]
            
        return u, error_sum_new 
    
    def _plant_step(self, x, u):
        dx     = x@self.mat_a.T + u@self.mat_b.T

        x_new  = x + dx*self.dt 
        y_new  = x@self.mat_c.T

        return x_new, y_new




    def _get_required(self, target_x, target_y, x_pos, y_pos, robot_theta):
        
        d_distance = (target_x - x_pos)**2 + (target_y - y_pos)**2
        d_distance = d_distance**0.5

        required_theta    = numpy.arctan2(target_y - y_pos, target_x - x_pos)
        required_theta    = numpy.mod(required_theta, 2.0*numpy.pi)

        d_theta = required_theta - robot_theta

        '''
        if d_theta < 0:
            d_theta+= 2.0*numpy.pi
        if d_theta > numpy.pi:
            d_theta-= 2.0*numpy.pi
        '''

        idx = numpy.where(d_theta < 0)
        d_theta[idx]+=2.0*numpy.pi

        idx = numpy.where(d_theta > numpy.pi)
        d_theta[idx]-=2.0*numpy.pi

        return d_distance, d_theta

    def _trajectory_eval(self, target_x, target_y, result):

        d = (target_x - result[:, :, 0])**2 + (target_y - result[:, :, 1])**2

        cost = d.mean(axis=0)
      
        return cost
       
