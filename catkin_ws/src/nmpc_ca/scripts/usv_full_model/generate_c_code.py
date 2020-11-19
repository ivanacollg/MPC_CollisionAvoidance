# The MIT License (MIT)
#
# Copyright 2020 Barbara Barros Carlos, Tommaso Sartor
#
# This file is part of crazyflie_nmpc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from acados_template import *
import acados_template as at
from export_ode_model import *
import numpy as np
import scipy.linalg
from ctypes import *
from os.path import dirname, join, abspath

ACADOS_PATH = join(dirname(abspath(__file__)), "../../acados")

# create render arguments
ra = acados_ocp_nlp()

# export model
model = export_ode_model()

Tf = 1.0
N = 100
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
nlp_dims     = ra.dims
nlp_dims.nx  = nx
nlp_dims.ny  = ny
nlp_dims.ny_e = ny_e
nlp_dims.nbx = 0
nlp_dims.nbu = nu
nlp_dims.nbx_e = 0
nlp_dims.nu  = model.u.size()[0]
nlp_dims.N   = N

# parameters
#g0  = 9.8066    # [m.s^2] accerelation of gravity
#mq  = 33e-3     # [kg] total mass (with one marker)
#Ct  = 3.25e-4   # [N/krpm^2] Thrust coef

# bounds
hov_w = np.sqrt((mq*g0)/(4*Ct))
max_thrust = 22

# set weighting matrices
nlp_cost = ra.cost
Q = np.eye(nx)
Q[0,0] = 1e0      # x
Q[1,1] = 0.0      # y
Q[2,2] = 1e0      # z
Q[3,3] = 0.0     # qw
Q[4,4] = 0.0     # qx

R = np.eye(nu)
R[0,0] = 0.0    # w1
R[1,1] = 0.0    # w2

nlp_cost.W = scipy.linalg.block_diag(Q, R)

Vx = np.zeros((ny, nx))
Vx[0,0] = 1.0
Vx[1,1] = 1.0
Vx[2,2] = 1.0
Vx[3,3] = 1.0
Vx[4,4] = 1.0
nlp_cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[5,0] = 1.0
Vu[6,1] = 1.0
nlp_cost.Vu = Vu

nlp_cost.W_e = 50*Q

Vx_e = np.zeros((ny_e, nx))
Vx_e[0,0] = 1.0
Vx_e[1,1] = 1.0
Vx_e[2,2] = 1.0
Vx_e[3,3] = 1.0
Vx_e[4,4] = 1.0

nlp_cost.Vx_e = Vx_e

#nlp_cost.yref   = np.array([0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, hov_w, hov_w, hov_w, hov_w])
nlp_cost.yref   = np.array([1, 0, 0, 0, 0, 0, 0])
nlp_cost.yref_e = np.array([0, 0, 0, 0, 0])

nlp_con = ra.constraints

'''
nlp_con.lbu = np.array([0,0,0,0])
nlp_con.ubu = np.array([+max_thrust,+max_thrust,+max_thrust,+max_thrust])
nlp_con.x0  = np.array([0,0,0,1,0,0,0,0,0,0,0,0,0])
nlp_con.idxbu = np.array([0, 1, 2, 3])
'''
# setting constraints
nlp_con.lbx = np.array([model.u_min])
nlp_con.ubx = np.array([model.u_max])
nlp_con.idxbx = np.array([0])
nlp_con.lbu = np.array([model.Tportdot_min, model.Tstbddot_min])
nlp_con.ubu = np.array([model.Tportdot_max, model.Tstbddot_max])
nlp_con.idxbu = np.array([0, 1])
nlp_con.x0  = np.array([0,0,0,0,0])

## set QP solver
#ra.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ra.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ra.solver_options.hessian_approx = 'GAUSS_NEWTON'
ra.solver_options.integrator_type = 'ERK'

# set prediction horizon
ra.solver_options.tf = Tf
ra.solver_options.nlp_solver_type = 'SQP_RTI'
#ra.solver_options.nlp_solver_type = 'SQP'

# set header path
ra.acados_include_path  = f'{ACADOS_PATH}/include'
ra.acados_lib_path      = f'{ACADOS_PATH}/lib'

ra.model = model

acados_solver = generate_solver(ra, json_file = 'acados_ocp.json')

print('>> NMPC exported')
