import numpy
import matplotlib.pyplot as plt


if __name__ == "__main__":

    noise_level     = 0.1
    samples_count   = 1000

    transmit        = numpy.random.randint(0, 10, samples_count) == 0
    
    received        = (1.0 - noise_level)*0*transmit + noise_level*numpy.random.randn(samples_count)
    received        = numpy.clip(received, 0, 1)
    
    y = 0.0
    result          = numpy.zeros(samples_count)
    for i in range(samples_count):
        if transmit[i] == 0:
            y = y + received[i]
        
        y = numpy.clip(y, 0, 1)

        result[i] = y
    
    plt.plot(transmit,    label="transmit")
    plt.plot(received,    label="received")
    plt.plot(result,      label="result")
    plt.legend()
    plt.show()
