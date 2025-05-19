from acados_template import AcadosModel
import numpy as np
import casadi as ca


def export_diffdrive_model():
    model_name  = "basic_diffdrive_model"
    x1 = ca.MX.sym('x1')
    y1 = ca.MX.sym('y1')
    psi = ca.MX.sym('psi')
    theta= ca.MX.sym('theta') #track progress variable

    x = ca.vertcat(x1,y1,psi,theta)

    v = ca.MX.sym('v')
    omega = ca.MX.sym('omega')
    var = ca.MX.sym('var') 


    u = ca.vertcat(v,omega,var)

    x1_dot = ca.MX.sym('x_dot')
    y1_dot = ca.MX.sym('y_dot')
    psi_dot = ca.MX.sym('psi_dot')
    theta_dot = ca.MX.sym('theta_dot')
    
    x_dot = ca.vertcat(x1_dot,
                    y1_dot,
                    psi_dot,
                    theta_dot)
    
    f_expl = ca.vertcat(v*ca.cos(psi),
                     v*ca.sin(psi),
                     omega,
                     var)

    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.name = model_name

    model.t_label = "$t$ [s]"
    model.x_labels = ["$x$", "$y$", "$\\theta$"]
    model.u_labels = ["$v$", "$\\omega$"]


    return model


