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


class MPCCDiffDriveController(Node):
    def __init__(self):
        super().__init__("mpcc_diffdrive_controller_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.horizon_publisher = self.create_publisher(Path, "diffdrive/prediction_horizon", 10)
        self.odom_info_subscription = self.create_subscription(MotorOdomInfo, "arduino/motor_odom_info", self.MotorOdomCallback, 10)
        self.velocity_publisher = self.create_publisher(CmdDriveVel, "arduino/cmd_vel",10)
        self.odometry_publisher = self.create_publisher(Odometry,"diffdrive/odometry",10)

        self.timer = self.create_timer(2/25, self.mpc_loop)
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
    
        self.ocp_solver, self.track= self.setup(N=25, tf=2.0)
        self.tracklength = self.track.L




    def setup(self,N, tf):
        ocp = AcadosOcp()
        model = export_diffdrive_model()
        ocp.model = model
        ocp.solver_options.tf = tf
        ocp.solver_options.N_horizon = N

        q_c = 1e4
        q_l = 3e4
        q_theta = 1e2
        q_psi = 3e3

        R_v = 1
        R_omega = 1
        R_var = 1
        
        x = ocp.model.x[0]
        y = ocp.model.x[1]
        psi = ocp.model.x[2]
        theta=ocp.model.x[3]

        u = ocp.model.u

        track = Track(data='/home/dpma/projects/DiffDrive-ROS/ros_ws/src/my_py_pkg/my_py_pkg/track_points.csv')
        x_full, y_full,theta_full = track.create_grid(density=1000)

        x_lin = ca.interpolant('x', 'linear', [theta_full], x_full)
        y_lin = ca.interpolant('y', 'linear', [theta_full], y_full)

        x_ref = x_lin(theta)                
        y_ref = y_lin(theta)

        dx_dtheta = ca.gradient(x_ref, theta)   
        dy_dtheta = ca.gradient(y_ref, theta)
        phi = ca.atan2(dy_dtheta, dx_dtheta)

        


        e_c = ca.sin(phi)*(x-x_ref) - ca.cos(phi)*(y-y_ref)
        e_l = -ca.cos(phi)*(x-x_ref) - ca.sin(phi)*(y-y_ref)
        heading_cost = q_psi * (psi-phi)**2
        ocp.model.cost_expr_ext_cost = (q_c * e_c**2 + q_l * e_l**2 + u[0]**2*R_v + u[1]**2*R_omega + u[2]**2*R_var - q_theta * theta) + heading_cost

        ocp.cost.cost_type = 'EXTERNAL'
        v_max = 1
        omega_max = 5

        ocp.constraints.lbu = np.array([-v_max/2, -omega_max, 0])
        ocp.constraints.ubu = np.array([v_max, omega_max, 1.0])
        ocp.constraints.idxbu = np.array([0, 1,2])

        
        ocp.constraints.x0 =  self.current_state

        ocp.constraints.constr_type = 'BGH'
        ocp.constraints.constr_type_0 = 'BGH'
        ocp.constraints.constr_type_e = 'BGH'

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'EXACT'
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
        

        self.get_logger().info(f"MPC controls: v={controls[0]:.3f}, omega={controls[1]:.3f}, theta= {self.next_theta:.3f}")
        self.state_history.append(self.current_state)
        self.control_history.append(controls)
        self.publishPredictionHorizon()
        self.publishVelocity(controls)

    def calcOdometry(self):
        delta_L_encoder = -(self.L_enc-self.prev_L_enc)
        delta_R_encoder = -(self.R_enc-self.prev_R_enc)

        self.prev_L_enc = self.L_enc
        self.prev_R_enc = self.R_enc

        d_left = self.wheel_radius*delta_L_encoder * 2*np.pi/360
        d_right = self.wheel_radius*delta_R_encoder * 2*np.pi/360

        d_avg = (d_left + d_right)/2
        delta_theta = (d_right-d_left)/self.wheel_base

        self.current_state[2] += delta_theta

        delta_x = d_avg * np.cos(self.current_state[2])
        delta_y = d_avg * np.sin(self.current_state[2])
        
        self.current_state[0] += delta_x
        self.current_state[1] += delta_y
        self.current_state[2] = np.atan2(np.sin(self.current_state[2]),np.cos(self.current_state[2]))
        self.next_theta = self.track.project(self.current_state[0], self.current_state[1])
        
        
        self.current_state[3] = self.next_theta

        self.publishOdometry()
        self.publishTF()

   
    def publishVelocity(self, controls):
        msg = CmdDriveVel()

        msg.vx = controls[0]
        msg.omega = controls[1]

        self.velocity_publisher.publish(msg)

    def publishOdometry(self):
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.current_state[0]
        odom_msg.pose.pose.position.y = self.current_state[1]
        odom_msg.pose.pose.position.z = 0.0  

        yaw = self.current_state[2]
        qx = 0.0
        qy = 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw


        self.odometry_publisher.publish(odom_msg)
    
    def MotorOdomCallback(self, msg):
        self.L_enc = msg.left_encoder
        self.R_enc = msg.right_encoder
        self.calcOdometry()

    
    def publishTF(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"            
        t.child_frame_id = "base_footprint"
        
        t.transform.translation.x = self.current_state[0]
        t.transform.translation.y = self.current_state[1]
        t.transform.translation.z = 0.0
        
        yaw = self.current_state[2]

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(yaw / 2.0)
        t.transform.rotation.w = np.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

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
    rclpy.get_default_context().on_shutdown(node.create_logs)

    try:
        rclpy.spin(node)
    finally:
        node.create_logs()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()






