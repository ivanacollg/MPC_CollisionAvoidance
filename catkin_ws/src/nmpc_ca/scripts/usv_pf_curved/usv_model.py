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

    model_name = "usv_model_pf_curved"

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
    X_u_dot = -2.25
    Y_v_dot = -23.13
    Y_r_dot = -1.31
    N_v_dot = -16.41
    N_r_dot = -2.79
    Yvv = -99.99
    Yvr = -5.49
    Yrv = -5.49
    Yrr = -8.8
    Nvv = -5.49
    Nvr = -8.8
    Nrv = -8.8
    Nrr = -3.49
    m = 30
    Iz = 4.1
    B = 0.41
    c = 1.0

    ## CasADi Model
    # set up states & controls
    psi = MX.sym("psi")
    sinpsi = MX.sym("sinpsi")
    cospsi = MX.sym("cospsi")
    u = MX.sym("u")
    v = MX.sym("v")
    r = MX.sym("r")
    ye = MX.sym("ye")
    x1 = MX.sym("x1")
    y1 = MX.sym("y1")
    ak = MX.sym("ak")
    nedx = MX.sym("nedx")
    nedy = MX.sym("nedy")
    Tport = MX.sym("Tport")
    Tstbd = MX.sym("Tstbd")
    x = vertcat(psi, sinpsi, cospsi, u, v, r, ye, x1, y1, ak, nedx, nedy, Tport, Tstbd)

    # controls
    UTportdot = MX.sym("UTportdot")
    UTstbddot = MX.sym("UTstbddot")
    U = vertcat(UTportdot, UTstbddot)

    # xdot
    psidot = MX.sym("psidot")
    sinpsidot = MX.sym("sinpsidot")
    cospsidot = MX.sym("cospsidot")
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    rdot = MX.sym("rdot")
    yedot = MX.sym("yedot")
    x1dot = MX.sym("x1dot")
    y1dot = MX.sym("y1dot")
    akdot = MX.sym("akdot")
    nedxdot = MX.sym("nedxdot")
    nedydot = MX.sym("nedydot")
    Tportdot = MX.sym("Tportdot")
    Tstbddot = MX.sym("Tstbddot")
    xdot = vertcat( psidot, sinpsidot, cospsidot, udot, vdot, rdot, yedot, x1dot, y1dot, akdot, nedxdot, nedydot, Tportdot, Tstbddot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    #Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    #sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)
    Xu = if_else(u > 1.25, 64.55, -25)
    Xuu = if_else(u > 1.25, -70.92, 0)
    Yv = 0.5*(-40*1000*fabs(v))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*((0.27/0.09)*(0.27/0.09)))
    Nr = (-0.52)*sqrt(u*u + v*v)
    Tu = Tport + c * Tstbd
    Tr = (Tport - c * Tstbd) * B / 2
    beta = atan2(v,u+.001)
    chi = psi + beta

    x = -1 : 0.1 : 1;
    y = randn(size(x));
    pp_casadi = casadi.interpolant('pp', 'bspline', {x}, y);

    f_expl = vertcat(
      r,
      cos(chi)*r,
      -sin(chi)*r,
      (Tu - (-m + 2 * Y_v_dot)*v - (Y_r_dot + N_v_dot)*r*r - (-Xu*u - Xuu*fabs(u)*u)) / (m - X_u_dot),
      (-(m - X_u_dot)*u*r - (- Yv - Yvv*fabs(v) - Yvr*fabs(r))*v) / (m - Y_v_dot),
      (Tr - (-2*Y_v_dot*u*v - (Y_r_dot + N_v_dot)*r*u + X_u_dot*u*r) - (-Nr*r - Nrv*fabs(v)*r - Nrr*fabs(r)*r)) / (Iz - N_r_dot),
      -(u*cos(psi) - v*sin(psi))*sin(ak) + (u*sin(psi) + v*cos(psi))*cos(ak),
      0,
      0,
      0,
      u*cos(psi) - v*sin(psi),
      u*sin(psi) + v*cos(psi),
      UTportdot,
      UTstbddot/c,
    )

    # constraint on forces
    #a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    #a_long = Fxd / m

    # Model bounds
    model.u_min = -2.0
    model.u_max = 2.0

    # state bounds
    model.Tport_min = -30
    model.Tstbd_min = -30
    model.Tport_max = 35
    model.Tstbd_max = 35

    model.r_min = -10.0 # minimum angular velocity [rad/s]
    model.r_max = 10.0  # maximum angular velocity [rad/s]

    # input bounds
    model.Tstbddot_min = -30 # minimum throttle change rate
    model.Tstbddot_max = 30 # maximum throttle change rate
    model.Tportdot_min = -30 # minimum throttle change rate
    model.Tportdot_max = 30 # maximum throttle change rate

    # nonlinear constraint
    #constraint.alat_min = -4  # maximum lateral force [m/s^2]
    #constraint.alat_max = 4  # maximum lateral force [m/s^1]

    #constraint.along_min = -4  # maximum lateral force [m/s^2]
    #constraint.along_max = 4  # maximum lateral force [m/s^2]

    # Define initial conditions
    starting_angle = 0.00
    x1 = 1.0
    y1 = -1.0
    x2 = 1.0
    y2 = 3.8
    ak = np.arctan2(y2-y1, x2-x1)
    ye = 0.0
    nedx = 0
    nedy = 0
    model.x0 = np.array([starting_angle, np.sin(starting_angle), np.cos(starting_angle), 0.001, 0.00, 0.00, ye, x1, y1, ak, nedx, nedy, 0.00, 0.00])

    # define constraints struct
    #constraint.alat = Function("a_lat", [x, u], [a_lat])
    #constraint.pathlength = pathlength
    #constraint.expr = vertcat(u,v,r,Tport,Tstbd)

    # Define model struct
    params = types.SimpleNamespace()
    params.X_u_dot = X_u_dot
    params.Y_v_dot = Y_v_dot
    params.Y_r_dot = Y_r_dot
    params.N_v_dot = N_v_dot
    params.N_r_dot = N_r_dot
    params.Yvv = Yvv
    params.Yvr = Yvr
    params.Yrv = Yrv
    params.Yrr = Yrr
    params.Nvv = Nvv
    params.Nvr = Nvr
    params.Nrv = Nrv
    params.m = m
    params.Iz = Iz
    params.B = B
    params.c = c

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
