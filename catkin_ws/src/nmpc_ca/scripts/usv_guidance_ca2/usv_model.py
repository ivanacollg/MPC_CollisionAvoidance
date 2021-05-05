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

    model_name = "usv_model_guidance_ca2"

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
    xned = MX.sym("xned")
    yned = MX.sym("yned")
    psi = MX.sym("psi")
    x = vertcat(u, v, ye, chie, psied, xned, yned, psi)

    # controls
    Upsieddot = MX.sym("Upsieddot")
    U = vertcat(Upsieddot)

    # xdot
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    yedot = MX.sym("yedot")
    chiedot = MX.sym("chiedot")
    psieddot = MX.sym("psieddot")
    xneddot = MX.sym("xneddot")
    yneddot = MX.sym("yneddot")
    psidot = MX.sym("psidot")
    xdot = vertcat(udot, vdot, yedot, chiedot, psieddot, xneddot, yneddot, psidot)

    # algebraic variables
    z = vertcat([])

    # parameters
    ox1 = MX.sym("ox1")
    oy1 = MX.sym("oy1")
    ox2 = MX.sym("ox2")
    oy2 = MX.sym("oy2")
    ox3 = MX.sym("ox3")
    oy3 = MX.sym("oy3")
    ox4 = MX.sym("ox4")
    oy4 = MX.sym("oy4")
    ox5 = MX.sym("ox5")
    oy5 = MX.sym("oy5")
    ox6 = MX.sym("ox6")
    oy6 = MX.sym("oy6")
    ox7 = MX.sym("ox7")
    oy7 = MX.sym("oy7")
    ox8 = MX.sym("ox8")
    oy8 = MX.sym("oy8")
    p = vertcat(ox1,oy1,ox2,oy2,ox3,oy3,ox4,oy4,ox5,oy5,ox6,oy6,ox7,oy7,ox8,oy8)
    #p = vertcat([])

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
      u*cos(psi) - v*sin(psi),
      u*sin(psi) + v*cos(psi),
      (psied-psie)/T1,
    )

    # constraint on forces
    #a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    #a_long = Fxd / m
    distance1 = sqrt((xned-ox1)*(xned-ox1) + (yned-oy1)*(yned-oy1))
    distance2 = sqrt((xned-ox2)*(xned-ox2) + (yned-oy2)*(yned-oy2))
    distance3 = sqrt((xned-ox3)*(xned-ox3) + (yned-oy3)*(yned-oy3))
    distance4 = sqrt((xned-ox4)*(xned-ox4) + (yned-oy4)*(yned-oy4))
    distance5 = sqrt((xned-ox5)*(xned-ox5) + (yned-oy5)*(yned-oy5))
    distance6 = sqrt((xned-ox6)*(xned-ox6) + (yned-oy6)*(yned-oy6))
    distance7 = sqrt((xned-ox7)*(xned-ox7) + (yned-oy7)*(yned-oy7))
    distance8 = sqrt((xned-ox8)*(xned-ox8) + (yned-oy8)*(yned-oy8))

    # Model bounds
    #model.psi_min = -pi
    #model.psi_max = pi

    # state bounds
    model.psied_min = -pi # minimum desired angle
    model.psied_max = pi # maximum desired angle

    # input bounds
    model.psieddot_min = -1#-0.25 # minimum desired angle rate
    model.psieddot_max = 1#0.25 # maximum desired angle rate

    model.Upsieddot_min = -1#-0.25 # minimum desired angle rate
    model.Upsieddot_max = 1#0.25 # maximum desired angle rate

    # nonlinear constraint
    #constraint.alat_min = -4  # maximum lateral force [m/s^2]
    #constraint.alat_max = 4  # maximum lateral force [m/s^1]
    constraint.distance_min = 0.5

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
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


    # define constraints struct
    #constraint.alat = Function("a_lat", [x, u], [a_lat])
    #constraint.pathlength = pathlength
    #constraint.expr = vertcat(u,v,r,Tport,Tstbd)
    constraint.expr = vertcat(distance1,distance2,distance3,distance4,distance5,distance6,distance7,distance8)

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
