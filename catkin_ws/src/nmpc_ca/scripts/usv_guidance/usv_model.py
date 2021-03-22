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

    model_name = "usv_model_guidance"

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
    nedx = MX.sym("nedx")
    nedy = MX.sym("nedy")
    psi = MX.sym("psi")
    sinpsi = MX.sym("sinpsi")
    cospsi = MX.sym("cospsi")
    u = MX.sym("u")
    v = MX.sym("v")
    ye = MX.sym("ye")
    ak = MX.sym("ak")
    psid = MX.sym("psid")
    x = vertcat(nedx, nedy, psi, sinpsi, cospsi, u, v, ye, ak, psid)

    # controls
    Upsiddot = MX.sym("Upsiddot")
    U = vertcat(Upsiddot)

    # xdot
    nedxdot = MX.sym("nedxdot")
    nedydot = MX.sym("nedydot")
    psidot = MX.sym("psidot")
    sinpsidot = MX.sym("sinpsidot")
    cospsidot = MX.sym("cospsidot")
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    yedot = MX.sym("yedot")
    akdot = MX.sym("akdot")
    psiddot = MX.sym("psiddot")
    xdot = vertcat(nedxdot, nedydot, psidot, sinpsidot, cospsidot, udot, vdot, yedot, akdot, psiddot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    #Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    #sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)

    f_expl = vertcat(
      u*cos(psi) - v*sin(psi),
      u*sin(psi) + v*cos(psi),
      (psid - psi)/T1,
      cos(psi)*((psid - psi)/T1),
      -sin(psi)*((psid - psi)/T1),
      0,
      0,
      -(u*cos(psi) - v*sin(psi))*sin(ak) + (u*sin(psi) + v*cos(psi))*cos(ak),
      0,
      Upsiddot,
    )

    # constraint on forces
    #a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    #a_long = Fxd / m

    # Model bounds
    model.u_min = -2.0
    model.u_max = 2.0
    #model.psi_min = -pi
    #model.psi_max = pi

    # state bounds
    model.psid_min = -pi
    model.psid_max = pi

    # input bounds
    model.psiddot_min = -1.5 # minimum throttle change rate
    model.psiddot_max = 1.5 # maximum throttle change rate

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
    nedx = 0
    nedy = 0
    starting_angle = 0.00
    u = 0
    v = 0
    x1 = 2.0
    y1 = 2.0
    x2 = 6.0
    y2 = -15.0
    ak = np.math.atan2(y2-y1, x2-x1)
    ye = -(nedx-x1)*np.sin(ak)+(nedy-y1)*np.cos(ak)
    model.x0 = np.array([nedx, nedy, starting_angle, np.sin(starting_angle), np.cos(starting_angle), u,v, ye, ak, 0.0])


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
