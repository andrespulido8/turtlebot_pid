#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose


def publisher():
    rospy.init_node('goal_pose_node', anonymous=True)
    pub = rospy.Publisher('goal_pose', Pose, queue_size=1)
    rate = rospy.Rate(10)  # Hz
    global init_time 
    init_time = np.array(rospy.get_time())
    while not rospy.is_shutdown():
        (x, y, z), (qx, qy, qz, qw) = goal_pose_rectangle()
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        pub.publish(p)
        rate.sleep()
    
def goal_pose_rectangle():
    time_step = 5 
    z = 0
    qx = qy = 0
    k = 1  # Multiplier
    now = np.array(rospy.get_time()) - init_time
    rospy.loginfo("Time: {}".format(now))
    # Initial point
    if now > 0 and now < time_step*1:
        x = 0
        y = -1*k
        qw = 0.707  # -90 degrees
        qz = -0.707
    elif now > time_step*1 and now < time_step*2:
        # Second point
        x = 1*k
        y = -1*k
        qw = 1
        qz = 0
    elif now > time_step*2 and now < time_step*3:
        # Third point
        x = 1*k
        y = 1*k
        qw = qz = 0.707 # 90 degrees
    elif now > time_step*3 and now < time_step*4:
        # Fourth point
        y = 1*k
        x = 0*k
        qw = 0
        qz = 1  # 180 degrees
    else:
        x = y = 0
        qw = 1
        qz = 0
        #print("Done with rectangle!")
    return np.array([x, y, z]), np.array([qx, qy, qz, qw])

if __name__ == '__main__':
    ii = 0
    publisher()
