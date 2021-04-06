import numpy as np
from casadi import *
import matplotlib.pyplot as plt

##2D Casadi
#xgrid = np.linspace(-5,5,11)
#ygrid = np.linspace(-4,4,9)
#X,Y = np.meshgrid(xgrid,ygrid,indexing='ij')
#R = np.sqrt(5*X**2 + Y**2)+ 1
#data = np.sin(R)/R
#data_flat = data.ravel(order='F')
#lut = interpolant('name','bspline',[xgrid,ygrid],data_flat)
#print(lut([0.5,1]))

#1D Casadi
x = np.linspace(1,6,6)
V = [-1,-1,-2,-3,0,2]
xq = np.arange(1, 6, 0.1).tolist()
lut = interpolant('LUT','bspline',[x],V)
#print(lut(2.5))
result = lut(xq)
print(result)

plt.figure(1)
plt.subplot(1,1,1)
plt.scatter(x,V, color='r',  marker='o')
plt.plot(xq,result, color='b', marker='.', linestyle='-')
#plt.plot(lut)
plt.ylabel('x')
plt.xlabel('y')
plt.legend(['curve', 'original'])
#plt.show()

# Using SciPy
import scipy.interpolate as ip
interp = ip.InterpolatedUnivariateSpline(x, V)
result = interp(xq)
plt.figure(2)
plt.subplot(1,1,1)
plt.scatter(x,V, color='r',  marker='o')
plt.plot(xq,result, color='b', marker='.', linestyle='-')
#plt.plot(lut)
plt.ylabel('x')
plt.xlabel('y')
plt.legend(['curve', 'original'])
plt.show()
