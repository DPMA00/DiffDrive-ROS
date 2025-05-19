import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from acados_template import AcadosOcp, AcadosOcpSolver
from my_py_pkg.diffdrivemodel import export_diffdrive_model
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import CmdDriveVel
from tf2_ros import TransformBroadcaster
import tf_transformations
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import casadi as ca



class MPCDiffDriveController(Node):
    def __init__(self):
        super().__init__("mpc_diffdrive_controller_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.horizon_publisher = self.create_publisher(Path, "diffdrive/prediction_horizon", 10)
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.OdomCallback, 10)
        self.velocity_publisher = self.create_publisher(CmdDriveVel, "arduino/cmd_vel",10)
        
        self.target_subscriber = self.create_subscription(Vector3, "diffdrive/target_pos", self.TargetPosCallback,10)

        self.timer = self.create_timer(2/10, self.mpc_loop)
        
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.wheel_radius = 0.05
        self.wheel_base = 0.29

        self.prev_L_enc = 0
        self.prev_R_enc = 0

        self.L_enc = 0
        self.R_enc = 0

        self.obstacle = np.array([1.0, 0.0])
        self.obstacle_R = 0.15
        self.safety_R = 0.25

        self.target = np.array([0.0,0.0,0.0])

        self.pose_error = 0
        self.heading_error = 0
        self.setup_solver1()


    def setup_solver(self):
        self.ocp = AcadosOcp()
        model = export_diffdrive_model()
        self.ocp.model = model
        self.ocp.solver_options.tf = 2
        self.ocp.solver_options.N_horizon = 20
        Q = np.diag([2000, 2000, 2000])
        R = np.diag([1, 1])
        Q_avoid = np.diag([1])

        Q_e = np.diag([5000, 5000, 5000])

        x = self.ocp.model.x[0]
        y = self.ocp.model.x[1]

        self.h_expr = (self.obstacle_R+self.safety_R)**2 - ((x - self.obstacle[0])**2 +  (y-self.obstacle[1])**2)
        self.ocp.model.con_h_expr = self.h_expr

        dist_sq =  ((x - self.obstacle[0])**2 +  (y-self.obstacle[1])**2)
        self.avoidance_cost = 10/(dist_sq + 1e-2)

        self.ocp.cost.cost_type = 'NONLINEAR_LS'
        #self.ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
        self.ocp.cost.yref = np.array([self.target[0], self.target[1], self.target[2], 0, 0, 0])
        self.ocp.model.cost_y_expr = ca.vertcat(model.x, model.u, self.avoidance_cost) 
        self.ocp.cost.W = ca.diagcat(Q, R, Q_avoid).full()
        self.ocp.cost.cost_type_e = 'NONLINEAR_LS'
        self.ocp.cost.yref_e = self.target
        self.ocp.model.cost_y_expr_e = model.x
        self.ocp.cost.W_e = Q

        v_max = 0.3; omega_max = 3
        self.ocp.constraints.lbu = np.array([-v_max, -omega_max])
        self.ocp.constraints.ubu = np.array([v_max, omega_max])
        self.ocp.constraints.idxbu = np.array([0, 1])
        self.ocp.constraints.x0 = self.current_state

        self.ocp.constraints.uh = np.array([0.0])
        self.ocp.constraints.lh = np.array([-20000])

        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        self.ocp_solver = AcadosOcpSolver(self.ocp)
        
        
    def setup_solver1(self):
        self.ocp = AcadosOcp()
        model = export_diffdrive_model()
        self.ocp.model = model
        self.ocp.solver_options.tf = 2
        self.ocp.solver_options.N_horizon = 20
        Q = np.diag([2000, 2000, 2000, 1000, 1000])
        R = np.diag([1, 1])
        Q_avoid = np.diag([1])

        Q_e = np.diag([5000, 5000, 5000, 3000,3000])
        
        self.avoidance_cost = 0

        self.ocp.cost.cost_type = 'NONLINEAR_LS'
        self.ocp.cost.yref = np.array([self.target[0], self.target[1], self.target[2], 0, 0, 0])
        self.ocp.model.cost_y_expr = ca.vertcat(model.x, model.u, self.avoidance_cost) 
        self.ocp.cost.W = ca.diagcat(Q, R, Q_avoid).full()
        self.ocp.cost.cost_type_e = 'NONLINEAR_LS'
        self.ocp.cost.yref_e = self.target
        self.ocp.model.cost_y_expr_e = model.x
        self.ocp.cost.W_e = Q

        v_max = 0.3; omega_max = 3
        a_max = 1.0; alpha_max = 8
        
        self.ocp.constraints.lbx = np.array([-v_max, -omega_max])
        self.ocp.constraints.ubx = np.array([-v_max, -omega_max])
        self.ocp.constraints.idxbx = np.array([3,4])
        
        self.ocp.constraints.lbu = np.array([-a_max, -alpha_max])
        self.ocp.constraints.ubu = np.array([a_max, alpha_max])
        self.ocp.constraints.idxbu = np.array([0, 1])
        self.ocp.constraints.x0 = self.current_state


        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        self.ocp_solver = AcadosOcpSolver(self.ocp)
        

    def mpc_loop(self):
        
        for i in range(self.ocp.solver_options.N_horizon):
            self.ocp_solver.set(i, "yref", np.array([self.target[0], self.target[1], self.target[2], self.target[3], self.target[4], self.target[6], 0, 0, 0]))
        
        self.ocp_solver.set(self.ocp.solver_options.N_horizon,"yref",self.target)
        
        self.ocp_solver.set(0,"lbx", self.current_state)
        self.ocp_solver.set(0,"ubx", self.current_state)

        if self.pose_error >= 0.05 or abs(self.heading_error) >= np.deg2rad(1):
            status = self.ocp_solver.solve()
            if status !=0:
                self.get_logger().error("Solver failed with status: %d" % status)
                return
            controls = self.ocp_solver.get(0,"u")
        else:
            controls = np.array([0.0,0.0])
        

        self.get_logger().info(f"MPC controls: v={controls[0]:.3f}, omega={controls[1]:.3f}")
        self.publishPredictionHorizon()
        self.publishVelocity(controls)
    
    def CalcPoseErrors(self):
        self.pose_error = np.linalg.norm(self.target[0:2]-self.current_state[0:2])
        self.heading_error = self.angle_wrap(self.target[2] - self.current_state[2])     
                
        
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
        
        self.current_state[3] = msg.twist.twist.linear.x
        self.current_state[4] = msg.twist.twist.angular.z
        self.CalcPoseErrors()
        
        self.get_logger().info(f"x: {self.current_state[0]:.3f}, y: {self.current_state[1]:.3f}, theta:  {np.rad2deg(self.current_state[2]):.3f}")

    def TargetPosCallback(self, msg):
        self.target[0] = msg.x
        self.target[1] = msg.y
        self.target[2] = msg.z
    

    def publishPredictionHorizon(self):
        trajectory = []
        for i in range(self.ocp.solver_options.N_horizon + 1):
            trajectory.append(self.ocp_solver.get(i, "x"))
        prediction = Path()
        
        prediction.header.stamp = self.get_clock().now().to_msg()
        prediction.header.frame_id = "odom" 

        for (x, y, yaw) in trajectory:
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

    def angle_wrap(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi




        
        
def main(args=None):
    rclpy.init(args=args)
    node = MPCDiffDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()






