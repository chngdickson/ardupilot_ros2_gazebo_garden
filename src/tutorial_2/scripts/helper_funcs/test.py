import numpy as np

xyz = np.array([1,1,1])
integral = np.array([0,0,0])
prev_err = np.array([0, 0, 0])
kps, kis, kds = np.array([.25, .25, .1]), np.array([0.01, 0.01, 0.01]), np.array([0.01, 0.01, 0.01])
error = -xyz

P = kps * error

integrals =  integral + error * .1
I = kis * integrals

D = kds * (error - prev_err) * .1
prev_err = error

print(P+I+D)