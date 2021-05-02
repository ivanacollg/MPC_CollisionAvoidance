#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

# author: Daniel Kloeser

import time, os
import numpy as np
from random import random

from acados_settings import *
from plotFcn import *
#from tracks.readDataFcn import getTrack
import matplotlib.pyplot as plt

"""
Example of the frc_racecars in simulation without obstacle avoidance:
This example is for the optimal racing of the frc race cars. The model is a simple bycicle model and the lateral acceleration is constraint in order to validate the model assumptions.
The simulation starts at s=-2m until one round is completed(s=8.71m). The beginning is cut in the final plots to simulate a 'warm start'. 
"""

#track = "LMS_Track.txt"
#[Sref, _, _, _, _] = getTrack(track)

Tf = 5.0  # prediction horizon
N = 100  # number of discretization steps
T = 50.00  # maximum simulation time[s]
#sref_N = 3  # reference for final reference progress

# load model
constraint, model, acados_solver = acados_settings(Tf, N)

# dimensions
nx = model.x.size()[0]
nu = model.U.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
simError = np.ndarray((Nsim, 3))

obsx = np.array([4,4,4,4])
obsy = np.array([4,8,12,20])
radius = np.array([1.0,1.0,1.0,1.0,0,0,0,0]) #0.5
pobs = np.ones(16)*100
robs = np.zeros(8)

#s0 = model.x0[0]
tcomp_sum = 0
tcomp_max = 0

x_pos = 0.0
y_pos = 0.0
x_vel_last = 0.0
y_vel_last = 0.0
psi_error = 0.0
psi_mae = 0.0 # Mean Absolute Error 
ye_mae = 0.0
psi_mse = 0.0 # Mean Square Error 
ye_mse = 0.0


#Start values
nedx = 0
nedy = 0
psi = 0.0
u = 0.7
v = 0.0

x1 = 4.0
y1 = -5.0
x2 = 4.0
y2 = 25.

ak = np.math.atan2(y2-y1, x2-x1)
ye = -(nedx-x1)*np.sin(ak)+(nedy-y1)*np.cos(ak)
psie = psi - ak
x0 = np.array([u, v, ye, psie, psie, nedx, nedy, psi])

acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)


# simulate
for i in range(Nsim):
    # update reference
    #u_ref = 1.4 #u0 + #sref_N
    for ii in range(len(obsx)):
        pobs[2*ii] = obsx[ii]
        pobs[2*ii+1] = obsy[ii]
        robs[ii] = radius[ii] + 0.2
    for j in range(N):
        yref=np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        acados_solver.set(j, "yref", yref)
        acados_solver.set(j, "p", pobs)
        acados_solver.constraints_set(j, "lh", robs)
    yref_N = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    acados_solver.set(N, "yref", yref_N)
    acados_solver.set(N, "p", pobs)

    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    elapsed = time.time() - t

    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")


    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]
    
    psi_error = x0[3]
    ye_error = x0[2]
    simError[i,0] = psi_error
    simError[i,1] = ye_error

    if (i>400):
        psi_mae += abs(psi_error)
        ye_mae += abs(ye_error)
        psi_mse += psi_error*psi_error
        ye_mse += ye_error*ye_error


    # update initial condition
    x0 = acados_solver.get(1, "x")
    # Add noise
    #if (abs(x0[2]) > (np.pi)):
    #    x0[2] = (x0[2]/abs(x0[2]))*(abs(x0[2]) - 2*np.pi)

    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    #s0 = x0[0]
    '''
    # check if one lap is done and break and remove entries beyond
    if x0[0] > Sref[-1] + 0.1:
        # find where vehicle first crosses start line
        N0 = np.where(np.diff(np.sign(simX[:, 0])))[0][0]
        Nsim = i - N0  # correct to final number of simulation steps for plotting
        simX = simX[N0:i, :]
        simU = simU[N0:i, :]
        break
    '''


# Plot Results
t = np.linspace(0.0, Nsim * Tf / N, Nsim)

plotRes(simX, simU, simError, obsx, obsy, radius, t)
#plotTrackProj(simX, track)
#plotalat(simX, simU, constraint, t)

# Print some stats
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
#print("Average speed:{}m/s".format(np.average(simX[:, 3])))
print("Lap time: {}s".format(Tf * Nsim / N))

#print("Mean Absolute Error psi: {}".format(psi_mae/600))
print("Mean Square Error psi: {}".format(psi_mse/600))
print("Mean Absolute Error ye: {}".format(ye_mae/600))
print("Mean Square Error ye: {}".format(ye_mse/600))


# avoid plotting when running on Travis
if os.environ.get("ACADOS_ON_TRAVIS") is None:
    plt.show()