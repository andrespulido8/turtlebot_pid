/*
 *
 *  Author: Andres Pulido
 *  Date: June, 2022
 */

#include "turtlebot_pid.hpp"

turtlebot_pid_controller::turtlebot_pid_controller(ros::NodeHandle &node) {

    ROS_INFO("Initializing class");
    kp = {0.4, 0.65};
    kd = {0.1, 0.1};
    ki = {0.002, 0.002};
    saturation = {0.8, 1.8};
    tolerance = {0.05, 0.02};
    ROS_INFO("After variables");

    queueSize = 5;
    
    // register the first simulation time
    timeOfPreviousControl = ros::Time(0);
    ROS_INFO("After time");

    nodeSubscriberTurtlePose = node.subscribe("/odom", queueSize,
        &turtlebot_pid_controller::odom_cb, this);

    ROS_INFO("Subscribing to goal_pose");
    nodeSubscriberWayPointPose = node.subscribe(
        "goal_pose", queueSize, &turtlebot_pid_controller::goal_pose_cb, this);
    nodePublisherTurtleVelocity = node.advertise<geometry_msgs::Twist>(
        "cmd_vel", queueSize);

    //ros::Rate rate(10);
    //while (node.ok()) {
    //    computePID();
    //    nodePublisherTurtleVelocity.publish(command);
}

turtlebot_pid_controller::~turtlebot_pid_controller() {
    // TODO Auto-generated destructor stub
}

void turtlebot_pid_controller::goal_pose_cb(
    const geometry_msgs::Pose &msg) {
    // this function retrieve the goal position of the turtlebot
    goalPose.x = msg.position.x;
    goalPose.y = msg.position.y;
    goalPose.z = msg.position.z;
    goalYawOrientation = tf::getYaw(msg.orientation);
}

//void turtlebot_pid_controller::poseTransFormCallback(
//    const geometry_msgs::TransformStamped &msg) {
//    // this function register the current pose of the robot thanks to optitrack
//    robotPose.x = msg.transform.translation.x;
//    robotPose.y = msg.transform.translation.y;
//    robotTheta = tf::getYaw(msg.transform.rotation);
//    dt = (msg.header.stamp - timeOfPreviousControl).toSec();
//    timeOfPreviousControl = msg.header.stamp;
//    computePID();
//}

void turtlebot_pid_controller::odom_cb(const geometry_msgs::PoseStamped &msg) {

    // this function register the current pose of the robot thanks to gazebo
    //ROS_INFO("Received odometry message! now computing PID");
    robotPose.x = msg.pose.position.x;
    robotPose.y = msg.pose.position.y;
    robotTheta = tf::getYaw(msg.pose.orientation);
    dt = (msg.header.stamp - timeOfPreviousControl).toSec();
    timeOfPreviousControl = msg.header.stamp;
    computePID();
}

void turtlebot_pid_controller::computePID() {
    
    double command_x = kp[0] * sqrt(pow(goalPose.x - robotPose.x, 2) +
                                 pow(goalPose.y - robotPose.y, 2));

    // heading towards the goal    
    double theta = ((atan2(goalPose.y - robotPose.y, goalPose.x - robotPose.x) -
                     robotTheta));
    if (theta > 3.14) {
        theta = -(2 * 3.14 - theta);
    }
    if (theta < -3.14) {
        theta = theta + 2 * 3.14;
    }
    double feedback_angular = kp[1] * theta;

    // clipping command
    command.linear.x =
        (command_x > saturation[0]) ? saturation[0] : (command_x < 0) ? 0 : command_x;
    command.angular.z = (feedback_angular > saturation[1])
                            ? saturation[1] : (feedback_angular < -saturation[1]) ? -saturation[1]: feedback_angular;

    nodePublisherTurtleVelocity.publish(command);
    //ROS_INFO("Command: %f %f", command.linear.x, command.angular.z);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_pid_cpp_node");
  ROS_INFO("Turtlebot PID cpp node started");

  ros::NodeHandle node;
  turtlebot_pid_controller turtlebot_pid_controller(node);

  ros::spin();

  return 0;
}
