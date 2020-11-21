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
def export_ode_model():

    model_name = 'usv_model'

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
    c = 0.78

    ## CasADi Model
    # set up states & controls
    u = MX.sym("u")
    v = MX.sym("v")
    r = MX.sym("r")
    Tport = MX.sym("Tport")
    Tstbd = MX.sym("Tstbd")
    x = vertcat(u, v, r, Tport, Tstbd)

    # controls
    UTportdot = MX.sym("UTportdot")
    UTstbddot = MX.sym("UTstbddot")
    U = vertcat(UTportdot, UTstbddot)

    # xdot
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    rdot = MX.sym("rdot")
    Tportdot = MX.sym("Tportdot")
    Tstbddot = MX.sym("Tstbddot")
    xdot = vertcat(udot, vdot, rdot, Tportdot, Tstbddot)


    # Model equations
    Xu = if_else(u > 1.25, 64.55, -25)
    Xuu = if_else(u > 1.25, -70.92, 0)
    Yv = 0.5*(-40*1000*fabs(v))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*((0.27/0.09)*(0.27/0.09)))
    Nr = (-0.52)*sqrt(u*u + v*v)
    Tu = Tport + c * Tstbd
    Tr = (Tport - c * Tstbd) * B / 2

    # Explicit and Implicit functions
    f_expl = vertcat(
      (Tu - (-m + 2 * Y_v_dot)*v - (Y_r_dot + N_v_dot)*r*r - (-Xu*u - Xuu*fabs(u)*u)) / (m - X_u_dot),
      (-(m - X_u_dot)*u*r - (- Yv - Yvv*fabs(v) - Yvr*fabs(r))*v) / (m - Y_v_dot),
      (Tr - (-2*Y_v_dot*u*v - (Y_r_dot + N_v_dot)*r*u + X_u_dot*u*r) - (-Nr*r - Nrv*fabs(v)*r - Nrr*fabs(r)*r)) / (Iz - N_r_dot),
      UTportdot,
      UTstbddot,
    )
    f_impl = xdot - f_expl

    # algebraic variables
    z = []

    # parameters
    p = []

    # dynamics
    #model = acados_dae()
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()

    #constraints
    # Model bounds
    model.u_min = -1.5
    model.u_max = 1.5

    # state bounds
    model.Tport_min = -30
    model.Tstbd_min = -30
    model.Tport_max = 35
    model.Tstbd_max = 35

    model.r_min = -1.0 # minimum angular velocity [rad/s]
    model.r_max = 1.0  # maximum angular velocity [rad/s]
    # input bounds
    model.Tstbddot_min = -10 # minimum change rate of stering angle [rad/s]
    model.Tstbddot_max = 10  # maximum change rate of steering angle [rad/s]
    model.Tportdot_min = -10 # -10.0  # minimum throttle change rate
    model.Tportdot_max = 10  # 10.0  # maximum throttle change rate

    constraint.expr = vertcat(u,Tport,Tstbd)

    # Define model struct
    params = types.SimpleNamespace()
    params.X_u_dot = X_u_dot
    params.Y_v_dot = Y_v_dot
    params.Y_r_dot = Y_r_dot
    params.N_v_dot = N_v_dot
    params.N_r_dot = N_r_dot
    #params.Xu = Xu
    #params.Xuu = Xuu
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

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = U
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    return model, constraint
