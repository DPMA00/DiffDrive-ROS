from casadi import *
from acados_template import AcadosModel


def export_diffdrive_model():
    model_name  = "basic_diffdrive_model"



    x1 = SX.sym('x1')
    y1 = SX.sym('y1')
    theta = SX.sym('theta')

    x = vertcat(x1,y1,theta)

    v = SX.sym('v')
    omega = SX.sym('omega')

    u = vertcat(v,omega)

    x1_dot = SX.sym('x_dot')
    y1_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    
    x_dot = vertcat(x1_dot,
                    y1_dot,
                    theta_dot)
    
    f_expl = vertcat(v*cos(theta),
                     v*sin(theta),
                     -omega)

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



    