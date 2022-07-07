//
// Created by vlad on 05.07.2022.
//
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
//#include <tf2_msgs/TFMessage.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/LinearMath/Quaternion.h>


#define     t90c    "turn_90_clockwise"
#define     t180c   "turn_180_clockwise"
#define     t270c   "turn_270_clockwise"
#define     t360c   "turn_360_clockwise"
#define     t90cc   "turn_90_counterclockwise"
#define     t180cc  "turn_180_counterclockwise"
#define     t270cc  "turn_270_counterclockwise"
#define     t360cc  "turn_360_counterclockwise"
#define     fwd1    "forward_1"
#define     fwd2    "forward_2"
#define     bckwd1  "backward_1"
#define     bckwd2  "backward_2"


class Mover {
private:
    ros::NodeHandle     nh;
    ros::Subscriber     moveCommandSub;
    ros::Publisher      commandPub;

    std::string         command_topic;
    double              maxLinearVelocity;
    double              maxAngularVelocity;
    int                 publishRate;
public:
    Mover(ros::NodeHandle _nh);
    void applyMove(const std_msgs::String::ConstPtr &moveMsg);
};

Mover::Mover(ros::NodeHandle _nh) : nh(_nh){
//    ros::NodeHandle nh;

    nh.param<std::string>("basic_mover/cmd_topic", command_topic, "cmd_vel");
    nh.param<double>("basic_mover/max_linear_velocity", maxLinearVelocity, 0.1);
    nh.param<double>("basic_mover/max_angular_velocity", maxAngularVelocity, 0.1);
    nh.param<int>("basic_mover/publish_rate", publishRate, 10);

    moveCommandSub = nh.subscribe<std_msgs::String>("task",
                                                    50,
                                                    &Mover::applyMove,
                                                    this);

    commandPub     = nh.advertise<geometry_msgs::Twist>(command_topic, 10);

}


void Mover::applyMove(const std_msgs::String::ConstPtr &moveMsg) {

    geometry_msgs::Twist velocity;
    velocity.linear.x=0;
    velocity.angular.z=0;

    double timeLimit(0);

    if (moveMsg->data == t90c) {
        velocity.angular.z = -maxAngularVelocity;
        timeLimit = 6.28 / (-velocity.angular.z * 4);
    } else if (moveMsg->data == t180c) {
        velocity.angular.z = -maxAngularVelocity;
        timeLimit = 6.28 / (-velocity.angular.z * 2);
    } else if (moveMsg->data == t270c) {
        velocity.angular.z = -maxAngularVelocity;
        timeLimit = 6.28 / (-velocity.angular.z * 4 / 3);
    } else if (moveMsg->data == t360c) {
        velocity.angular.z = -maxAngularVelocity;
        timeLimit = 6.28 / (-velocity.angular.z * 1);
    } else if (moveMsg->data == t90cc) {
        velocity.angular.z = maxAngularVelocity;
        timeLimit = 6.28 / (velocity.angular.z * 4);
    } else if (moveMsg->data == t180cc) {
        velocity.angular.z = maxAngularVelocity;
        timeLimit = 6.28 / (velocity.angular.z * 2);
    } else if (moveMsg->data == t270cc) {
        velocity.angular.z = maxAngularVelocity;
        timeLimit = 6.28 / (velocity.angular.z * 4 / 3);
    } else if (moveMsg->data == t360cc) {
        velocity.angular.z = maxAngularVelocity;
        timeLimit = 6.28 / (velocity.angular.z * 1);
    } else if (moveMsg->data == fwd1) {
        velocity.linear.x = maxLinearVelocity;
        timeLimit = 1.0 / velocity.linear.x;
    } else if (moveMsg->data == fwd2) {
        velocity.linear.x = maxLinearVelocity;
        timeLimit = 2.0 / velocity.linear.x;
    } else if (moveMsg->data == bckwd1) {
        velocity.linear.x = -maxLinearVelocity;
        timeLimit = 1.0 / -velocity.linear.x;
    } else if (moveMsg->data == bckwd2) {
        velocity.linear.x = -maxLinearVelocity;
        timeLimit = 2.0 / -velocity.linear.x;
    } else
        ROS_WARN_STREAM("Unknown command \"" << moveMsg->data << "\"");


//    ROS_INFO_STREAM("Got command " << moveMsg->data <<
//                    "\nCurrent velocity:\n" << velocity <<
//                    "\nTime limit: " << timeLimit);

    if (timeLimit > 0){
        ros::Duration duration(timeLimit);
        ros::Time begin = ros::Time::now();
        ros::Rate rate(publishRate);
        while(ros::Time::now() - begin < duration){
            commandPub.publish(velocity);
            rate.sleep();
        }
    }

    velocity.linear.x = 0;
    velocity.angular.z = 0;
    commandPub.publish(velocity);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "basic_text_mover");
    ros::NodeHandle _nh;
    Mover BasicMover(_nh);

    ros::Rate r(5);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

