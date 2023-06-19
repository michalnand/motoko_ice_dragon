import numpy


'''
    log indices : 
    0   steps
    1   d_distance
    2   d_theta

    3   model.x[0, 0]   #radial velocity
    4   model.x[1, 0]   #theta
    5   model.x[2, 0]   #velocity
    6   model.x[3, 0]   #distance

    7   model.x_pos
    8   model.y_pos
    9   model.theta

    10  best_controller_idx
'''



class LoadLogs:
    #def __init__(self, file_names, seq_length = 32, input_indices = [1, 2, 3, 5], target_indices = [10]):

    def __init__(self, file_names, seq_length = 32, input_indices = [1, 2, 3, 4, 5, 6, 7, 8, 9], target_indices = [10]):

        self.data = []

        for f_name in file_names:
            data = numpy.loadtxt(f_name)
            print("loading ", f_name, data.shape)
            self.data.append(data)

        self.input_indices  = numpy.array(input_indices)
        self.target_indices = numpy.array(target_indices)

        self.length = 0
        for j in range(len(self.data)):
            self.length+= self.data[j].shape[0]

        self.seq_length = seq_length

    def __len__(self):
        return self.length


    def get_batch(self, batch_size = 64):

        x_batch = numpy.zeros((batch_size, self.seq_length, self.input_indices.shape[0]),  dtype=numpy.float32)
        y_batch = numpy.zeros((batch_size, self.target_indices.shape[0]), dtype=numpy.float32)

        for j in range(batch_size):
            data_idx    = numpy.random.randint(0, len(self.data))
            line_idx    = numpy.random.randint(0, self.data[data_idx].shape[0] - self.seq_length)

            x = self.data[data_idx][line_idx:line_idx+self.seq_length, self.input_indices]
            y = self.data[data_idx][line_idx+self.seq_length, self.target_indices]

            x_batch[j] = x
            y_batch[j] = y

        return x_batch, y_batch


        


