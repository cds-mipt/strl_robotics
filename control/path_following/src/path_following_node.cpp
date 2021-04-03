/*
This is a path following controller for a differential drive mobile robot 
*/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <math.h>


// declare error parameters
double distance_error=0 , angle_error=0 , old_angle_err=0;

// declare path parameters
double x_planning[100] , y_planning[100];
uint32_t size_path=0 , old_size_path=0;
bool path_changed=false, path_stored=false;
int count_path=0;

// declare condition parametrs
double  distance_stop=0.25 , d_stop=0 , a_stop=0;

// declare odometry parameters
double odom_x, odom_y, odom_theta;

// declare control parameters
double v_current=0 , v_old=0 , w_current=0;

// declare integers
int j=0 , k_v=0 , rate_loop=10;

//declare sample time
double dt=0.1;

// declare states 
bool motion_mode=false, stop_mode_ind = false;


// declare Publishers and Subscribers
ros::Publisher   desired_traj_pub;
ros::Publisher   cmd_publish;
ros::Publisher   errors_pub;
ros::Subscriber  odom_subscriber;
ros::Subscriber  traje_subscriber;
ros::Subscriber  number_path;

//declare messages
geometry_msgs::Pose2D qd[100],err;
geometry_msgs::Twist tw_msg;

// declare topics
std::string path_topic;
std::string cmd_topic;
std::string global_frame;
std::string base_frame;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;

// declare functions
double sat_angular_speed(double max , double min , double w_ref);
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);
void check_path();
void get_errors();
void stop_mode();
void lon_control();
void lat_control();
double get_yaw(geometry_msgs::Quaternion q);


// Callback to get the Path from Planning node
void traj_back(const geometry_msgs::PoseArray msg) 
{
   size_path = msg.poses.size();
   
   if(size_path!=0)
   {
       path_changed=true;
       for (j=0; j<size_path ; j++)
       {   
           x_planning[j]=msg.poses[j].position.x;
           y_planning[j]=msg.poses[j].position.y;
           qd[j].x=x_planning[j];
           qd[j].y=y_planning[j];
        //   ROS_INFO("j,  x , y [%i ,%f , %f]" , j , qd[j].x, qd[j].y);
       }
       path_stored=true;
       old_size_path=size_path;
   }
   else
   {
      // ROS_INFO("Size _path [%i]", size_path);
       ROS_WARN("There is no Path");
   }
}


// Initilize the Path
void check_path()
{
    if(path_changed)
    {
        count_path++;
        k_v=0;
        ROS_INFO("MOTION MODE - Path CHANGED [%i]", count_path);
        path_changed=false;
        stop_mode_ind = false;
    }
}

// Get the errors between the robot and path
void get_errors()
{
          err.x = (qd[k_v].x-odom_x) ; 
	      err.y =  (qd[k_v].y-odom_y) ;
          distance_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
          double angle_path=atan2(err.y,err.x);
          err.theta =  angle_path - odom_theta;
          angle_error = atan2(sin(err.theta ),cos(err.theta ));
}

// Linear Speed Controller
void lon_control()
{
   // ROS_INFO("Motion Mode - Errors Angle and Distance [%i , %i, , %f , %f]" ,k_v , old_size_path, angle_error , distance_error);
    if(path_stored && k_v<old_size_path)
    {
        if(abs(angle_error)<=0.2)
        {
            v_current = sat_linear_velocity(max_v,min_v,acc_v ,distance_error, v_old);
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }
        else if (abs(angle_error)>0.2 && k_v==0)
        {
            v_current = 0;
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }
        else if (abs(angle_error)>0.2 && k_v>0)
        {
            v_current = sat_linear_velocity(0.15,min_v,acc_v ,distance_error, v_old);;
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }
        if (distance_error<distance_stop && k_v <old_size_path-1)
        {
            k_v++;
        }
    }
}

// Angular Speed Controller
void lat_control()
{
    if(path_stored  && k_v<old_size_path)
    {
        w_current=sat_angular_speed(max_w,0,angle_error); 
        tw_msg.angular.z=w_current;
        old_angle_err = angle_error; 
    }
}

// Stop Mode
void stop_mode()
{
    if (old_size_path > 0)
    {
        d_stop = sqrt(pow((qd[old_size_path-1].x-odom_x), 2) + pow((qd[old_size_path-1].y-odom_y), 2));
    if (d_stop<0.2 && !stop_mode_ind)
    {
        stop_mode_ind = true;
        tw_msg.linear.x=0;
        tw_msg.angular.z=0;
        count_path =0;
        ROS_INFO("STOP MODE");
    }
    }
    
}


// Saturation Angular Speed Function
double sat_angular_speed(double max , double min , double w_ref)
{
    double w_err=w_ref;
    if(abs(w_err)>=1 && v_current>=0.6)
    {
        w_err=4*w_err;
    }
    else if (abs(w_err)>=1 && v_current<0.6)
    {
        w_err=2.8*w_err;
    }
    else if (abs(w_err)<1 && abs(w_err)>=0.5 && v_current>=0.6)
    {
        w_err=2.8*w_err;
    }
    else if (abs(w_err)<1 && abs(w_err)>=0.5 && v_current<0.6)
    {
        w_err=2.3*w_err;
    }
    else if (abs(w_err)<0.5 && v_current>=0.6)
    {
        w_err=2.9*w_err;
    }
    else if (abs(w_err)<0.5 && v_current<0.6)
    {
        w_err=1.8*w_err;
    }

    if (signbit(w_err)==1)
    {
        if (abs(w_err)<min)
        {return -min;}
        else if (abs(w_err)>max)
        {return -max;}
        else
        {return w_err;}
    }
    else if (signbit(w_err)==0)
    {
        if (w_err<min)
        {return min;}
        else if (w_err>max)
        {return max;}
        else
        {return w_err;}
    }
}

// Saturation Linear Speed Function
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity)
{
    double v_cmd = v_ref - old_velocity;
    if(v_cmd>0)
    {
        v_cmd = old_velocity +accel*dt;
    }
    else if(v_cmd<0)
    {
        v_cmd = old_velocity -0.2*dt;
    }
    else
    {
        v_cmd = old_velocity;
    }

    if (v_cmd<min)
    {
        return min;
    }
    else if (v_cmd>max)
    {
        return max;
    }
    else
    {
        return v_cmd;
    }
}

// Initialize config parameters
void Initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_topic", path_topic);
    n.getParam("cmd_topic" ,  cmd_topic);
    n.getParam("global_frame" ,  global_frame);
    n.getParam("base_frame" ,  base_frame);

    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
}


// Main Function
int main(int argc, char **argv) {
 
    ros::init(argc, argv, "path_following_node");
    ros::NodeHandle n;

    Initialize_parameters(n);


    // Define the publishers and sunscribers
    cmd_publish         = n.advertise<geometry_msgs::Twist>    (cmd_topic, 1);
    traje_subscriber    = n.subscribe                          (path_topic,1 , traj_back);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(rate_loop);

    

    ROS_INFO("Start Path Following Node");

    while (ros::ok()) {

    //transform the path into world coordinate
    geometry_msgs::TransformStamped transformStamped;
    try{
        
      transformStamped = tfBuffer.lookupTransform(global_frame, base_frame,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    odom_x = transformStamped.transform.translation.x;
    odom_y = transformStamped.transform.translation.y;
    odom_theta = get_yaw(transformStamped.transform.rotation);


    check_path();

    get_errors();

    if (!stop_mode_ind)
    {
       lon_control();
 
       lat_control();
    }

    stop_mode();

    if (size_path!=0)
    {
        cmd_publish.publish(tw_msg);
    }
    else
    {
        tw_msg.linear.x=0;
        tw_msg.angular.z=0;
        cmd_publish.publish(tw_msg);

    }
    
    
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}

// Get Theta of the Path
double get_yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
