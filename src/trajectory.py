#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


def publisher():
    pub = rospy.Publisher('trajectory', Pose, queue_size=1)
    rospy.init_node('des_pose_generator', anonymous=True)
    rate = rospy.Rate(2)  # Hz
    while not rospy.is_shutdown():
        p = Pose()
        p.position.x = 0.5
        p.position.y = -0.1
        p.position.z = 1.0
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        pub.publish(p)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
