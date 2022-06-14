#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from tf.transformations import quaternion_inverse, quaternion_multiply

class pid_controller(object):
    def __init__(self):
        # Initialize gains and setup dynamic reconfigure
        self.kp = np.zeros(3).T
        self.kd = np.zeros(3).T
        self.ki = np.zeros(3).T
        self.last_config = None
        self.reconfigure_server = DynamicReconfigureServer(PIDControllerConfig, self.reconfigure)
        self.wrench_pub = rospy.Publisher(
            '/wrench', WrenchStamped, queue_size=3)
        pub = rospy.Publisher('chatter', String, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_cb, queue_size=3)
        rate = rospy.Rate(10) # 10hz


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
            kg = self.parse_gains(config['kg'])
        except ValueError as e:
            rospy.logwarn('Invalid dynamic reconfigure: {}'.format(e))
            return self.last_config
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.ki = np.array(ki)
        self.last_config = config
        rospy.loginfo('Gains succesfully updated via dynamic reconfigure')
        return config

    def update(self):
        now = rospy.Time.now()

    def computePID(self):
        # Compute the PID error
        position_error_world = self.desired_position - self.position
        orientation_error_world = quat_to_rotvec(quaternion_multiply(
            self.desired_orientation, quaternion_inverse(self.orientation)))
        pose_error_world = np.concatenate(
            (position_error_world, orientation_error_world))
        twist_error_world = self.desired_twist_world - self.twist_world
        twist_error_body = self.world_to_body.dot(twist_error_world)

        self.error = error
        pub.publish(hello_str)
        

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        rate.sleep()


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


    def odom_cb(self, msg):
        self.body_frame = msg.child_frame_id
        self.global_frame = msg.header.frame_id
        (self.position, self.orientation), (linvel,
                                            angvel), _, _ = odometry_to_numpy(msg)
        self.body_to_world = self.make_double_rotation(
            quaternion_matrix(self.orientation))
        self.world_to_body = self.body_to_world.T
        self.twist_body = np.concatenate((linvel, angvel))
        self.twist_world = self.body_to_world.dot(self.twist_body)
        self.update()


if __name__ == '__main__':
        try:
                rospy.init_node('talker', anonymous=True)
                pid = pid_controller()
        except rospy.ROSInterruptException:
                pass
 

