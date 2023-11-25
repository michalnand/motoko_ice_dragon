from utils.load_logs import *
import torch


class Model(torch.nn.Module):
    def __init__(self, inputs_count, outputs_count):
        super().__init__()

        self.rnn  = torch.nn.GRU(input_size = inputs_count, hidden_size = 64, batch_first=True)
        self.lin  = torch.nn.Linear(64, outputs_count)
    
    def forward(self, x):

        _, hn   = self.rnn(x)        
        y       = self.lin(hn[0])
        
        return y
    
def eval_acc(y_pred, y_target, classes_count):

    y_target   = y_target.detach().numpy()
    y_pred_tmp = torch.argmax(y_pred, dim=1).detach().numpy()

    acc = (y_pred_tmp == y_target).mean()

    cm = numpy.zeros((classes_count, classes_count), dtype=int)

    for i in range(y_target.shape[0]):
        cm[y_target[i]][y_pred_tmp[i]]+= 1

    return acc, cm
    

if __name__ == "__main__":

    training_logs = []
    training_logs.append("logs/trajectory_0.log")
    training_logs.append("logs/trajectory_1.log")
    training_logs.append("logs/trajectory_2.log")
    training_logs.append("logs/trajectory_3.log")
    training_logs.append("logs/trajectory_4.log")
    training_logs.append("logs/trajectory_5.log")
    training_logs.append("logs/trajectory_6.log")
    training_logs.append("logs/trajectory_7.log")
    training_logs.append("logs/trajectory_8.log")
    training_logs.append("logs/trajectory_9.log")
    training_logs.append("logs/trajectory_10.log")
    training_logs.append("logs/trajectory_11.log")
    training_logs.append("logs/trajectory_12.log")
    training_logs.append("logs/trajectory_13.log")
    training_logs.append("logs/trajectory_14.log")
    training_logs.append("logs/trajectory_15.log")
    training_logs.append("logs/trajectory_16.log")
    training_logs.append("logs/trajectory_17.log")
    training_logs.append("logs/trajectory_18.log")
    training_logs.append("logs/trajectory_19.log")
    training_logs.append("logs/trajectory_20.log")
    

    testing_logs = []
    testing_logs.append("logs/trajectory_21.log")
    testing_logs.append("logs/trajectory_22.log")
    testing_logs.append("logs/trajectory_23.log")
    testing_logs.append("logs/trajectory_24.log")

    training_dataset = LoadLogs(training_logs)

    testing_dataset  = LoadLogs(testing_logs)


    batch_size      = 256
    classes_count   = 16

    model       = Model(9, classes_count)
    loss_func   = torch.nn.CrossEntropyLoss()
    optimizer   = torch.optim.Adam(model.parameters(), lr = 0.001)
    
    print(model)

    for epoch in range(100):
        for n in range(len(training_dataset)//batch_size):
            x, y        = training_dataset.get_batch(batch_size)

            x_in        = torch.from_numpy(x).float()
            y_target    = torch.from_numpy(y).long().squeeze(-1)

            y_pred      = model(x_in)

            loss        = loss_func(y_pred, y_target)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step() 
        

        cm_sum = 0
        for n in range(len(testing_dataset)//batch_size):
            x, y        = testing_dataset.get_batch(batch_size)

            x_in        = torch.from_numpy(x).float()
            y_target    = torch.from_numpy(y).long().squeeze(-1)

            y_pred      = model(x_in)

            acc, cm         = eval_acc(y_pred, y_target, classes_count)
            cm_sum = cm_sum + cm


        class_acc = numpy.diag(cm_sum)/cm_sum.sum(axis=0)
        cm_rel    = cm_sum/cm_sum.sum(axis=0)
        print(numpy.round(100.0*class_acc, 1))
        print(numpy.round(100.0*cm_rel, 1))
        print("\n\n")
           