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

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from usv_model import usv_model
import scipy.linalg
import numpy as np


def acados_settings(Tf, N):
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = usv_model()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.U
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # define constraint
    #model_ac.con_h_expr = constraint.expr

    # set dimensions
    nx = model.x.size()[0]
    nu = model.U.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N
    #ns = 2
    #nsh = 2

    # set cost
    Q = np.diag([0, 0, 0, 0.1, 0.1, 0, 0, 0, 0.8, 0, 0, 0])
    
    R = np.eye(nu)
    R[0, 0] = 0.01

    Qe = np.diag([0, 0, 0, 0.1, 0.1, 0, 0, 0, 0.8, 0, 0, 0])


    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    #unscale = N / Tf

    #ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    #ocp.cost.W_e = Qe / unscale
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx
 
    Vu = np.zeros((ny, nu))
    Vu[12, 0] = 1.0

    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    '''ocp.cost.zl = 0 * np.ones((ns,)) #previously 100
    ocp.cost.Zl = 0 * np.ones((ns,))
    ocp.cost.zu = 0 * np.ones((ns,)) #previously 100
    ocp.cost.Zu = 0 * np.ones((ns,))'''

    # set intial references
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    # setting constraints
    ocp.constraints.lbx = np.array([model.psid_min, model.rd_min])
    ocp.constraints.ubx = np.array([model.psid_max, model.rd_max])
    ocp.constraints.idxbx = np.array([10, 11])
    ocp.constraints.lbu = np.array([model.rddot_min])
    ocp.constraints.ubu = np.array([model.rddot_max])
    ocp.constraints.idxbu = np.array([0])
    # ocp.constraints.lsbx=np.zero s([1])
    # ocp.constraints.usbx=np.zeros([1])
    # ocp.constraints.idxsbx=np.array([1])
    '''ocp.constraints.lh = np.array(
        [
            model.u_min,
            model.u_min,
            model.r_min,
            model.Tport_min,
            model.Tstbd_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            model.u_max,
            model.u_max,
            model.r_max,
            model.Tport_max,
            model.Tstbd_max,
        ]
    )'''
    '''ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array([0, 2])'''

    # set intial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    #ocp.solver_options.nlp_solver_type = "SQP_RTI"
    #ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    #ocp.solver_options.sim_method_num_stages = 4
    #ocp.solver_options.sim_method_num_steps = 3

    #ocp.solver_options.nlp_solver_max_iter = 200
    #ocp.solver_options.tol = 1e-4

    #ocp.solver_options.qp_solver_tol_stat = 1e-2
    #ocp.solver_options.qp_solver_tol_eq = 1e-2
    #ocp.solver_options.qp_solver_tol_ineq = 1e-2
    #ocp.solver_options.qp_solver_tol_comp = 1e-2

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return constraint, model, acados_solver