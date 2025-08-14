import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from acados_template import AcadosOcp, AcadosOcpSolver
from my_py_pkg.diffdrive_aug_model import export_diffdrive_model
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import MotorOdomInfo
from my_robot_interfaces.msg import CmdDriveVel
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import casadi as ca
from my_py_pkg.track import Track
import tf_transformations

class MPCCDiffDriveController(Node):
    def __init__(self):
        super().__init__("mpcc_diffdrive_controller_node")
        self.horizon_publisher = self.create_publisher(Path, "diffdrive/prediction_horizon", 10)
        self.odom_subscription = self.create_subscription(Odometry, "robot/odom", self.OdomCallback, 10)
        self.velocity_publisher = self.create_publisher(CmdDriveVel, "cmd_vel",10)
        self.track_publisher = self.create_publisher(Path, "diffdrive/track", 10)

        self.timer = self.create_timer(2/35, self.mpc_loop)
        self.next_theta = 0.0
        self.current_state = np.array([0.0, 0.0, 0.0,self.next_theta])
        self.wheel_radius = 0.05
        self.wheel_base = 0.29

        self.prev_L_enc = 0
        self.prev_R_enc = 0

        self.L_enc = 0
        self.R_enc = 0

        self.obstacle = np.array([1.0, 0.0])
        self.obstacle_R = 0.15
        self.safety_R = 0.25

        self.state_history = []
        self.control_history = []
        self.prediction_history = []
    
        self.ocp_solver, self.track= self.setup(N=35, tf=2.0)
        self.tracklength = self.track.L
        self.track_x, self.track_y = self.track.get_track_plot_params()




    def setup(self,N, tf):
        ocp = AcadosOcp()
        model = export_diffdrive_model()
        ocp.model = model
        ocp.solver_options.tf = tf
        ocp.solver_options.N_horizon = N

        q_c = 2e4
        q_l = 2.5e3
        q_var = 4e4
        q_psi = 2e3
        

        R_v = 1
        R_omega = 100
        R = np.diag([R_v, R_omega])

        
        x = ocp.model.x[0]
        y = ocp.model.x[1]
        psi = ocp.model.x[2]
        theta=ocp.model.x[3]

        u = ocp.model.u
        u_var = u[2]

        track = Track(data='/home/dpma/projects/DiffDrive-ROS/ros_ws/src/my_py_pkg/my_py_pkg/track_points2.csv')
        x_full, y_full, theta_full = track.create_grid(density=3000)

        x_lin = ca.interpolant('x', 'linear', [theta_full], x_full)
        y_lin = ca.interpolant('y', 'linear', [theta_full], y_full)
        
        x_ref = x_lin(theta)                
        y_ref = y_lin(theta)      
        
        dx_dtheta = ca.gradient(x_ref, theta)   
        dy_dtheta = ca.gradient(y_ref, theta)
        phi = ca.atan2(dy_dtheta, dx_dtheta)

        


        e_c = ca.sin(phi)*(x-x_ref) - ca.cos(phi)*(y-y_ref)
        e_l = -ca.cos(phi)*(x-x_ref) - ca.sin(phi)*(y-y_ref)
        
        #heading_error = psi-phi
        #heading_error = ca.atan2(ca.sin(heading_error), ca.cos(heading_error))
        
        velocity_error = u_var - 0.75

        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = ca.vertcat(e_c,
                                           e_l,
                                           #heading_error,
                                           velocity_error,
                                           u[:2])
        ocp.cost.yref = np.array([0,0,0,0,0])
        ocp.cost.W = ca.diagcat(q_c, q_l,q_var, R).full()
        
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr_e = ca.vertcat(e_c,
                                             e_l,
                                            )
                                            # heading_error)
        ocp.cost.yref_e = np.array([0,0])
        ocp.cost.W_e = ca.diagcat(q_c, q_l).full()
        
        v_max = 0.7
        omega_max = 4

        ocp.constraints.lbu = np.array([0, -omega_max, 0])
        ocp.constraints.ubu = np.array([v_max, omega_max, 1.0])
        ocp.constraints.idxbu = np.array([0, 1,2])

        
        ocp.constraints.x0 =  self.current_state

        ocp.constraints.constr_type = 'BGH'
        ocp.constraints.constr_type_0 = 'BGH'
        ocp.constraints.constr_type_e = 'BGH'

        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        
        
        ocp_solver = AcadosOcpSolver(ocp)

        

        return ocp_solver, track

    def mpc_loop(self):
        self.ocp_solver.set(0,"lbx", self.current_state)
        self.ocp_solver.set(0,"ubx", self.current_state)

        if self.next_theta  < 2*self.tracklength:
            status = self.ocp_solver.solve()
            if status !=0:
                self.get_logger().error("Solver failed with status: %d" % status)
                return
            v, w, var = self.ocp_solver.get(0,"u")
            controls = np.array([v,w])
        else:
            self.get_logger().warn(f'Theta: {self.next_theta:.3f}, Tracklength: {self.tracklength:.3f}')
            controls = np.array([0.0,0.0])
        

        #self.get_logger().info(f"MPC controls: v={controls[0]:.3f}, omega={controls[1]:.3f}, theta= {self.next_theta:.3f}")
        self.theta_prev = self.current_state[3]

        self.state_history.append(self.current_state)
        self.control_history.append(controls)
        self.publishPredictionHorizon()
        self.publishVelocity(controls)
        self.publishTrackPath()


   
    def publishVelocity(self, controls):
        msg = CmdDriveVel()

        msg.vx = controls[0]
        msg.omega = controls[1]

        self.velocity_publisher.publish(msg)

    
    def OdomCallback(self, msg: Odometry):
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x,
                      orientation_q.y,
                      orientation_q.z,
                      orientation_q._w)

        (_,_,yaw) = tf_transformations.euler_from_quaternion(quaternion)
        
        self.current_state[2] = yaw
        self.current_state[3] = self.track.project(self.current_state[0], self.current_state[1], self.theta_prev)        
        self.get_logger().info(f"x: {self.current_state[0]:.3f}, y: {self.current_state[1]:.3f}, theta:  {np.rad2deg(self.current_state[2]):.3f}")


    def publishPredictionHorizon(self):
        trajectory = []
        for i in range(self.ocp_solver.acados_ocp.solver_options.N_horizon + 1):
            trajectory.append(self.ocp_solver.get(i, "x"))
        prediction = Path()
        
        prediction.header.stamp = self.get_clock().now().to_msg()
        prediction.header.frame_id = "odom" 

        for (x, y, yaw, progress) in trajectory:
            pose_stamped = PoseStamped()
            
            pose_stamped.header.stamp = prediction.header.stamp
            pose_stamped.header.frame_id = prediction.header.frame_id

            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = np.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = np.cos(yaw / 2.0)

            

            prediction.poses.append(pose_stamped)

        self.horizon_publisher.publish(prediction)

    def publishTrackPath(self):
        traj = Path()
        
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "odom" 

        for (x, y) in zip(self.track_x, self.track_y):
            pose_stamped = PoseStamped()
            
            pose_stamped.header.stamp = traj.header.stamp
            pose_stamped.header.frame_id = traj.header.frame_id

            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = np.sin(0.0 / 2.0)
            pose_stamped.pose.orientation.w = np.cos(0.0 / 2.0)

            

            traj.poses.append(pose_stamped)

        self.track_publisher.publish(traj)
        

    def create_logs(self):
        import pandas as pd
        

        states = np.vstack(self.state_history)
        ctrls = np.vstack(self.control_history)

        DF = pd.DataFrame({
            "x": states[:,0],
            "y": states[:,1],
            "psi": states[:,2],
            "theta": states[:,3],
            "v": ctrls[:,0],
            "w": ctrls[:,1],
        })

        DF.to_csv('/tmp/data.csv', index=False)
        
    

        
        
def main(args=None):
    rclpy.init(args=args)
    node = MPCCDiffDriveController()
    #rclpy.get_default_context().on_shutdown(node.create_logs)

    try:
        rclpy.spin(node)
    finally:
        #node.create_logs()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()






