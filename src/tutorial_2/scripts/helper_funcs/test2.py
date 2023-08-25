import numpy as np
import matplotlib.pyplot as plt
k = 10
plt.ion()
array = np.zeros((k, k))
for i in range(k):
    for j in range(k):
        array[i, j] = 1
        plt.imshow(array)
        plt.show()
        plt.pause(0.001)
        plt.clf()