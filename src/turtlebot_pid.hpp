/*
 * turtlebot_pid.hpp
 *
 * Author: Andres Pulido 
 */

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
//#include <simple_pid.h>
#include <tf/tf.h>
#include <turtlesim/Pose.h>

/*!
@class turtlebot_pid_controller
@brief It is a ros c++ which compute a linear and an angular
velocity applied to a robot though a ros publisher. To do so, this class
subscribes through ros to the robot pose and to the robot goal pose
*/
class turtlebot_pid_controller {
  public:
    /*!
    @brief Constructor of a turtlebot_pid_controller instance 
    @param[in] _node  is the rosnode taken as argument
    @return Newly created turtlebot_pid_controller instance
    */
    turtlebot_pid_controller(ros::NodeHandle &node);
    
    /*!
    #@brief Callback function of the robot pose when the turtlebot_pid_controller class
    subscribes to a geometry_msgs::TransformStamped msgs
    #@param[in] _msg  is the pose of the robot retrieved through ros and is in a
    geometry_msgs::TransformStamped format
    */
    //void poseTransFormCallback(const geometry_msgs::TransformStamped &_msg);
    
    /*!
    @brief Callback function of the robot pose when the turtlebot_pid_controller class
    //subscribes to a nav_msgs::Odometry msgs
    @param[in] _msg  is the pose of the robot retrieved through ros and is in a
    //TransformStamped msg format
    */
    void odom_cb(const geometry_msgs::PoseStamped &_msg);

    /*!    
    @brief Callback function of the robot goal pose when the turtlebot_pid_controller
    class subscribes to a geometry_msgs::Pose &msg
    @param[in] _msg  is the goal pose of the robot retrieved through ros and is
    in a geometry_msgs::Pose msg format
     */
    void goal_pose_cb(const geometry_msgs::Pose &_msg);
    
    /*!
    @brief This function computes and publishs the velocities to apply to the
    robot depending on its current goal pose and pose. It publishs it as
    geometry_msgs/Twist msg.
    @param
    @return
    */
    void computePID();
    
    /*!
    @brief Destructor of the turtlebot_pid_controller_instance
    @param
    @return
    */
    virtual ~turtlebot_pid_controller();

  private:
    ros::Subscriber nodeSubscriberTurtlePose;
    ros::Subscriber nodeSubscriberWayPointPose;
    ros::Publisher nodePublisherTurtleVelocity;
    int queueSize;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> ki;
    std::vector<double> saturation;
    std::vector<double> tolerance;
    geometry_msgs::Twist command;
    geometry_msgs::Vector3 goalPose;
    geometry_msgs::Vector3 lastGoalPose;
    double goalYawOrientation = 0;
    double lastGoalYawOrientation = 0;
    double robotTheta = 0;
    geometry_msgs::Vector3 robotPose;
    geometry_msgs::Vector3 robotVel;
    ros::Time timeOfPreviousControl;
    double dt = 0;
};

