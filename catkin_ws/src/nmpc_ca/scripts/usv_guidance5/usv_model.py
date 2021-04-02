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

from casadi import *
#from tracks.readDataFcn import getTrack


def usv_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "usv_model_guidance5"

    # load track parameters
    #[s0, _, _, _, kapparef] = getTrack(track)
    #length = len(s0)
    #pathlength = s0[-1]
    # copy loop to beginning and end
    #s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    #kapparef = np.append(kapparef, kapparef[1:length])
    #s0 = np.append([-s0[length - 2] + s0[length - 81 : length - 2]], s0)
    #kapparef = np.append(kapparef[length - 80 : length - 1], kapparef)

    # compute spline interpolations
    #kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)

    #USV model coefficients
    T1 = 1.0

    ## CasADi Model
    # set up states & controls
    u = MX.sym("u")
    v = MX.sym("v")
    ye = MX.sym("ye")
    chie = MX.sym("chie")
    psied = MX.sym("psied")
    x = vertcat(u, v, ye, chie, psied)

    # controls
    Upsieddot = MX.sym("Upsieddot")
    U = vertcat(Upsieddot)

    # xdot
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    yedot = MX.sym("yedot")
    chiedot = MX.sym("chiedot")
    psieddot = MX.sym("psieddot")
    xdot = vertcat(udot, vdot, yedot, chiedot, psieddot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    #Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    #sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)

    beta = atan2(v, u+0.001)
    psie = chie - beta
    f_expl = vertcat(
      0,
      0,
      u*sin(psie) + v*cos(psie),
      (psied-psie)/T1,
      Upsieddot,
    )

    # constraint on forces
    #a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    #a_long = Fxd / m

    # Model bounds
    #model.psi_min = -pi
    #model.psi_max = pi

    # state bounds
    model.psied_min = -pi/2 # minimum desired angle
    model.psied_max = pi/2 # maximum desired angle


    # input bounds
    model.psieddot_min = -0.25 # minimum desired angle rate
    model.psieddot_max = 0.25 # maximum desired angle rate

    # nonlinear constraint
    #constraint.alat_min = -4  # maximum lateral force [m/s^2]
    #constraint.alat_max = 4  # maximum lateral force [m/s^1]

    #constraint.along_min = -4  # maximum lateral force [m/s^2]
    #constraint.along_max = 4  # maximum lateral force [m/s^2]

    # Define initial conditions
    #starting_angle = 0.00
    #x1 = 1.0
    #y1 = -1.0
    #x2 = 1.0
    #y2 = 3.8
    #ak = np.arctan2(y2-y1, x2-x1)
    #ye = 0.0
    #nedx = 0
    #nedy = 0
    #model.x0 = np.array([starting_angle, np.sin(starting_angle), np.cos(starting_angle), 0.001, 0.00, 0.00, ye, x1, y1, ak, nedx, nedy, 0.00, 0.00])
    #Start values
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


    # define constraints struct
    #constraint.alat = Function("a_lat", [x, u], [a_lat])
    #constraint.pathlength = pathlength
    #constraint.expr = vertcat(u,v,r,Tport,Tstbd)

    # Define model struct
    params = types.SimpleNamespace()
    params.T1 = T1

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.U = U
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params
    return model, constraint
