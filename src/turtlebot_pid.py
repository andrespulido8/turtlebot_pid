#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, PoseStamped
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from tf.transformations import quaternion_inverse, quaternion_multiply
import tf.transformations as trans
#from andres_turtlebot_pid.src import pidcfg

class turtlebot_pid_controller(object):
    def __init__(self):
        # Initialize gains and setup dynamic reconfigure  (currently not used)
        #self.last_config = None
        #self.reconfigure_server = DynamicReconfigureServer(
        #    pidcfg, self.reconfigure)

        self.is_sim = rospy.get_param('/is_sim', True)

        # Gains 
        if self.is_sim:
            self.kp = np.array([0.2,0.9]) # [x,yaw]
            self.kd = np.array([0.05,0.1])
            self.ki = np.array([0.02,0.01])
            self.saturation = np.array([0.7,1.6])  # [x,yaw]
            # amount of error to tolerate before stopping
            self.tolerance = np.array([0.05,0.01])
            self.is_debug = False
        else:
            # TODO: change this for hardware
            self.kp = np.array([0.2,0.5]) # [x,yaw]
            self.kd = np.array([0.1,0.2])
            self.ki = np.array([0.02,0.01])
            self.saturation = np.array([0.7,1.6])  # [x,yaw]
            # amount of error to tolerate before stopping
            self.tolerance = np.array([0.05,0.01])
            self.is_debug = False

        # Initialize state variables
        self.body_frame = None
        self.global_frame = None
        self.last_update = rospy.get_time()
        self.desired_position = np.array([0,0,0])
        self.desired_orientation = np.array([0,0,0,1])
        self.desired_twist_world = np.array([0,0,0,0,0,0])
        self.body_to_world = None
        self.world_to_body = None
        self.position = None
        self.orientation = None
        self.err_accumulation = np.array([0,0], dtype=np.float32)
        self.twist_world = None
        self.pose = None
        self.twist_world = None
        self.twist_body = None
        rospy.loginfo("Initializing turtlebot PID controller")

        # subcriber for desired state
        self.ref = rospy.Subscriber(
            '/goal_pose', Pose, self.goal_pose_cb, queue_size=3)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_cb, queue_size=3)
        self.command_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=3)
        if not self.is_sim:
            self.vel_sub = rospy.Subscriber('/pose_stamped', PoseStamped, self.pose_cb, queue_size=3)

    @staticmethod
    def parse_gains(gains):
        ret = [float(gain.strip()) for gain in gains.split(',')]
        if len(ret) != 6:
            raise ValueError('not 6')
        return ret

    def reconfigure(self, config, level):
        try:
            kp = self.parse_gains(config['kp'])
            kd = self.parse_gains(config['kd'])
            ki = self.parse_gains(config['ki'])
        except ValueError as e:
            rospy.logwarn('Invalid dynamic reconfigure: {}'.format(e))
            return self.last_config
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.ki = np.array(ki)
        self.last_config = config
        rospy.loginfo('Gains succesfully updated via dynamic reconfigure')
        return config

    @staticmethod
    def make_double_rotation(R):
        R2 = np.zeros((6, 6))
        R2[:3, :3] = R2[3:, 3:] = R
        return R2

    def computePID(self):
        command = np.array([0,0])
        now = rospy.get_time()
        dt  = np.array(now - self.last_update)

        # Calculate error in position, orientation, and twist (velocities)
        position_error_world = self.desired_position - self.position
        orientation_error_world = self.quat_to_rotvec(quaternion_multiply(
            self.desired_orientation, quaternion_inverse(self.orientation)))
        if self.is_sim:
            twist_error_world = self.desired_twist_world - self.twist_world
            err_twist_norm = np.linalg.norm(twist_error_world[:2])
            feedback_derivative = np.array([self.kd[0]*err_twist_norm, self.kd[1]*twist_error_world[-1]])
        
        # Compute distance error to goal
        err_pos_norm = np.linalg.norm(position_error_world[:2])

        # Desired orientation is looking at the goal
        yaw_error = np.arctan2(position_error_world[1], position_error_world[0]) - self.euler_from_quaternion(self.orientation)[2]
        if err_pos_norm < self.tolerance[0]:
            # use desired orientation when close to goal
            yaw_error = orientation_error_world[-1]
        if np.abs(yaw_error) > np.pi:
            # Wraps angle
            yaw_error = yaw_error - np.sign(yaw_error) * 2 * np.pi
        
        # Integral of error (and clip if too large)
        self.err_accumulation += np.array([err_pos_norm, yaw_error])*dt 
        self.err_accumulation[1] = np.clip(self.err_accumulation[1], -np.pi, np.pi)
        
        # Calculate feedback
        feedback_proportional = np.array([np.abs(self.kp[0]*err_pos_norm), self.kp[1]*yaw_error])
        feedback_integral = self.ki*self.err_accumulation
        if self.is_sim:
            feedback = feedback_proportional + feedback_derivative + feedback_integral
        else:
            feedback = feedback_proportional + feedback_integral
        command = np.clip(feedback, np.array([0,-self.saturation[1]]), self.saturation) 

        if err_pos_norm < self.tolerance[0]:
            # do not move if position error is small
            command[0] = 0
        elif np.abs(yaw_error) > np.pi/4:
            # rotate to position only and delete accumulated error if yaw error is large (> 90 deg)
            command[0] = 0
            self.err_accumulation[0] /= 1.5
        if np.abs(yaw_error) < self.tolerance[1]:
            # do not rotate if yaw error is small
            command[1] = 0

        if self.is_debug:
            rospy.loginfo("Feedback proportional: {}".format(feedback_proportional)) 
            if self.is_sim:
                rospy.loginfo("Feedback derivative: {}".format(feedback_derivative))
            rospy.loginfo("feedback integral: {}".format(feedback_integral))
            rospy.loginfo("command: {}".format(command))

        # Publish command
        self.command_pub.publish(self.make_command_msg(command))

        self.last_update = now

    @staticmethod
    def make_command_msg(command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0
        msg.angular.z = command[1]
        return msg

    @staticmethod
    def quat_to_rotvec(q):
        ''' Convert a quaternion to a rotation vector
        '''
        # For unit quaternion, return 0 0 0
        if np.all(np.isclose(q[0:3], 0)):
            return np.array([0., 0., 0.])
        if q[3] < 0:
            q = -q
        q = trans.unit_vector(q)
        angle = np.arccos(q[3]) * 2
        norm = np.linalg.norm(q)
        axis = q[0:3] / norm
        return axis * angle
    
    @staticmethod
    def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = q[0]; y = q[1]; z = q[2]; w = q[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        return np.array([roll_x, pitch_y, yaw_z])  # in radians

    @staticmethod
    def quaternion_matrix(q):
        mat_h = trans.quaternion_matrix(q)
        return mat_h[:3, :3] / mat_h[3, 3]

    def goal_pose_cb(self, msg):
        self.desired_position = np.array([msg.position.x,
                                          msg.position.y,
                                          msg.position.z])
        self.desired_orientation = np.array([msg.orientation.x,
                                                msg.orientation.y,
                                                msg.orientation.z,
                                                msg.orientation.w])
        self.desired_twist_world = np.array([0, 0, 0, 0, 0, 0])
    
    def odom_cb(self, msg):
        self.body_frame = msg.child_frame_id
        self.global_frame = msg.header.frame_id
        self.position = np.array([msg.pose.pose.position.x, 
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z])
        self.orientation = np.array([msg.pose.pose.orientation.x,
                                        msg.pose.pose.orientation.y,    
                                        msg.pose.pose.orientation.z,
                                        msg.pose.pose.orientation.w])
        linvel = np.array([msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z])
        angvel = np.array([msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z])
        self.body_to_world = self.make_double_rotation(self.quaternion_matrix(self.orientation))
        self.world_to_body = self.body_to_world.T  # currently not used
        self.twist_body = np.concatenate((linvel, angvel))
        self.twist_world = self.body_to_world.dot(self.twist_body)
        if self.is_sim:
            self.computePID()

    def pose_cb(self, msg):
        # callback for getting hardware pose
        self.body_frame = msg.child_frame_id
        self.global_frame = msg.header.frame_id
        self.position = np.array([msg.pose.position.x, 
                                    msg.pose.position.y,
                                    msg.pose.position.z])
        self.orientation = np.array([msg.pose.orientation.x,
                                        msg.pose.orientation.y,    
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.body_to_world = self.make_double_rotation(self.quaternion_matrix(self.orientation))
        self.computePID()


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_PID_controller', anonymous=True)
        pid = turtlebot_pid_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

