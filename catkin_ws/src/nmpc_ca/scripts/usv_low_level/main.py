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

Tf = 1.0  # prediction horizon
N = 100  # number of discretization steps
T = 10.00  # maximum simulation time[s]
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
simdis = np.ndarray((Nsim, 2))
simError = np.ndarray((Nsim, 2))

#s0 = model.x0[0]
tcomp_sum = 0
tcomp_max = 0

psi_ref = 1.0#np.pi/2.0
sinpsi_ref = np.sin(psi_ref)
cospsi_ref = np.cos(psi_ref)
u_ref = 0.8

x_pos = 0.0
y_pos = 0.0
x_vel_last = 0.0
y_vel_last = 0.0
psi_error = 0.0
u_error = 0.0
psi_mae = 0.0 # Mean Absolute Error 
u_mae = 0.0 # Mean Absolute Error 
psi_mse = 0.0 # Mean Square Error 
u_mse = 0.0 # Mean Square Error 

# simulate
for i in range(Nsim):
    # update reference
    #u_ref = 1.4 #u0 + #sref_N
    for j in range(N):
        #yref = np.array([u0 + (u_ref - u0) * j / N, 0, 0, 0, 0, 0, 0, 0])
        yref=np.array([0, sinpsi_ref, cospsi_ref, u_ref, 0, 0, 0, 0, 0, 0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([0, sinpsi_ref, cospsi_ref, u_ref, 0, 0, 0, 0])
    # yref_N=np.array([0,0,0,0,0,0])
    acados_solver.set(N, "yref", yref_N)

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

    #Get values for grafic display
    x_vel = x0[3]*np.cos(x0[0])-x0[4]*np.sin(x0[0])
    y_vel = x0[3]*np.sin(x0[0])+x0[4]*np.cos(x0[0])
    x_pos = ((x_vel+x_vel_last)/2)*(Tf/N) + x_pos
    y_pos = ((y_vel+y_vel_last)/2)*(Tf/N) + y_pos
    x_vel_last = x_vel
    y_vel_last = y_vel
    simdis[i,0] = x_pos
    simdis[i,1] = y_pos

    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]
    
    psi_error = x0[0] - psi_ref
    u_error = x0[3] - u_ref
    simError[i,0] = psi_error
    simError[i,1] = u_error

    if (i>400):
        psi_mae += abs(psi_error)
        u_mae += abs(u_error)
        psi_mse += psi_error*psi_error
        u_mse += u_error*u_error

    # update initial condition
    x0 = acados_solver.get(1, "x")
    # Add noise
    #x0[3] = x0[3] #+ random()#*0.0002
    #x0[5] = x0[5] #+ random()#*0.0002

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

plotRes(simX, simU, simdis, simError, t)
#plotTrackProj(simX, track)
#plotalat(simX, simU, constraint, t)

# Print some stats
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
#print("Average speed:{}m/s".format(np.average(simX[:, 3])))
print("Lap time: {}s".format(Tf * Nsim / N))

print("Mean Absolute Error psi: {}".format(psi_mae/600))
print("Mean Square Error psi: {}".format(psi_mse/600))
print("Mean Absolute Error u: {}".format(u_mae/600))
print("Mean Square Error u: {}".format(u_mse/600))

# avoid plotting when running on Travis
if os.environ.get("ACADOS_ON_TRAVIS") is None:
    plt.show()
