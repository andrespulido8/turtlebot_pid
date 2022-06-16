#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from tf.transformations import quaternion_inverse, quaternion_multiply
import tf.transformations as trans
#from andres_turtlebot_pid.src import pidcfg

class pid_controller(object):
    def __init__(self):
        # Initialize gains and setup dynamic reconfigure
        self.last_config = None
        #self.reconfigure_server = DynamicReconfigureServer(
        #    pidcfg, self.reconfigure)
        self.kp = np.array([0.4,0.7]) # [x,yaw]
        self.kd = np.array([0.02,0.02])
        self.ki = np.array([0.002,0.002])
        self.saturation = np.array([1,2])
        self.tolerance = np.array([0.05,0.02])

        # Initialize state variables
        self.body_frame = None
        self.global_frame = None
        self.last_update = rospy.get_time()
        self.desired_position = None
        self.desired_orientation = None
        self.desired_twist_world = None
        self.body_to_world = None
        self.world_to_body = None
        self.position = None
        self.orientation = None
        self.err_accumulation = np.array([0,0], dtype=np.float32)
        self.twist_world = None
        self.pose = None
        self.twist_world = None
        self.twist_body = None
        rospy.loginfo("Initializing PID controller")

        self.command_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=3)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_cb, queue_size=3)
        #self.ref = rospy.Subscriber(
        #    '/trajectory', Pose, self.trajectory_cb, queue_size=3)
        rate = rospy.Rate(10)  # 10hz

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

        # Calculate error in position, orientation, and twist
        position_error_world = self.desired_position - self.position
        orientation_error_world = self.quat_to_rotvec(quaternion_multiply(
            self.desired_orientation, quaternion_inverse(self.orientation)))
        pose_error_world = np.concatenate(
            (position_error_world, orientation_error_world))
        twist_error_world = self.desired_twist_world - self.twist_world
        twist_error_body = self.world_to_body.dot(twist_error_world)
        
        err_pos_norm = np.linalg.norm(position_error_world[:2])
        err_twist_norm = np.linalg.norm(twist_error_world[:2])
        self.err_accumulation += np.array([pose_error_world[0], pose_error_world[-1]])*dt 
        
        yaw_error = np.arctan2(position_error_world[1], position_error_world[0]) - self.euler_from_quaternion(self.orientation)[2]
        if np.abs(yaw_error) > np.pi:
            yaw_error = yaw_error - np.sign(yaw_error) * 2 * np.pi
        if err_pos_norm < self.tolerance[0]:
            yaw_error = orientation_error_world[-1]

        rospy.loginfo("pos norm error: {}".format(err_pos_norm))
        rospy.loginfo("yaw error: {}".format(yaw_error))

        # Calculate feedback
        feedback_proportional = np.array([np.abs(self.kp[0]*err_pos_norm), self.kp[1]*yaw_error])
        rospy.loginfo("Feedback proportional: {}".format(feedback_proportional)) 
        feedback_derivative = np.array([self.kd[0]*err_twist_norm, self.kd[1]*twist_error_world[-1]])
        rospy.loginfo("feedback derivative: {}".format(feedback_derivative))
        feedback_integral = self.ki*self.err_accumulation
        rospy.loginfo("feedback integral: {}".format(feedback_integral))
        feedback = feedback_proportional + feedback_derivative + feedback_integral

        if err_pos_norm < self.tolerance[0]:
            # do not move if position error is small
            command[0] = 0
        elif yaw_error > self.tolerance[1]:
            # rotate to position only and delete accumulated error if yaw error is large
            command[0] = 0
            self.err_accumulation[0] = 0
        if yaw_error < self.tolerance[1]:
            # do not rotate if yaw error is small
            command[1] = 0
        command = np.clip(feedback, np.array([0,-self.saturation[1]]), self.saturation) 
        rospy.loginfo("command: {}".format(command))
        print('\n')

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
        '''
        Convert a quaternion to a rotation vector
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

    def trajectory_cb(self, msg):
        self.desired_position = np.array([msg.pose.position.x,
                                          msg.pose.position.y,
                                          msg.pose.position.z])
        self.desired_orientation = np.array([msg.pose.orientation.x,
                                                msg.pose.orientation.y,
                                                msg.pose.orientation.z,
                                                msg.pose.orientation.w])
        self.desired_twist_world = np.array([msg.twist.linear.x,
                                                msg.twist.linear.y, 
                                                msg.twist.linear.z,
                                                msg.twist.angular.x,
                                                msg.twist.angular.y,
                                                msg.twist.angular.z])
        body_to_world = self.make_double_rotation(
            self.quaternion_matrix(self.desired_orientation))
        #self.desired_twist_world = body_to_world.dot(
        #    np.hstack(twist_to_numpy(msg.posetwist.twist)))
    
    def trajectory(self):
        z = 0
        qx = qy = 0
        k = 1  # Multiplier
        now = np.array(rospy.get_time()) 
        rospy.loginfo("Time: {}".format(now))
        # Initial point
        if now > 0 and now < 20:
            x = 0
            y = -1*k
            qw = 0.707  # -90 degrees
            qz = -0.707
        elif now > 20 and now < 40:
            # Second point
            x = 1*k
            y = -1*k
            qw = 1
            qz = 0
            rospy.loginfo("Time > 20: {}".format(now))
        elif now > 40 and now < 60:
            # Third point
            x = 1*k
            y = 1*k
            qw = qz = 0.707 # 90 degrees
            rospy.loginfo("Time > 40: {}".format(now))
        elif now > 60 and now < 80:
            # Fourth point
            y = 1*k
            x = 0*k
            qw = 0
            qz = 1  # 180 degrees
            rospy.loginfo("Time > 60: {}".format(now))
        else:
            rospy.loginfo("Time else: {}".format(now))
            x = y = 0
            qw = 1
            qz = 0
            rospy.loginfo("Time > 20: {}".format(now))
        self.desired_position = np.array([x, y, z])
        rospy.loginfo("Desired position: {}".format(self.desired_position))
        self.desired_orientation = np.array([qx, qy, qz, qw])
        self.desired_twist_world = np.array([0,0,0,0,0,0])  #([vx, vy, vz, wx, wy, wz])
        body_to_world = self.make_double_rotation(
            self.quaternion_matrix(self.desired_orientation))
        #self.desired_twist_world = body_to_world.dot(
        #    np.hstack(twist_to_numpy(msg.posetwist.twist)))

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
        #rospy.loginfo("Received odometry message: \n{}".format(msg))
        self.body_to_world = self.make_double_rotation(self.quaternion_matrix(self.orientation))
        self.world_to_body = self.body_to_world.T
        self.twist_body = np.concatenate((linvel, angvel))
        self.twist_world = self.body_to_world.dot(self.twist_body)
        self.trajectory()
        self.computePID()


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_PID_controller', anonymous=True)
        pid = pid_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

