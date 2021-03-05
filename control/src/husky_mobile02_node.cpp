
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>

double x_trans , y_trans;
double distance_linear_error=0, distance_angular_error=0,last_dist_err_i = 0 , dist_err_i=0, e_dif=0;
double old_angle_angular_err=0,  angle_angular_error=0 , angle_linear_error=0, distance_stop=0.3;
double x_planning[1000] , y_planning[1000] , theta_path[1000] ;
double odom_x, odom_y, odom_theta;
double err_value=0 , old_err_value=0;
double v_current=0 , v_old=0 , w_current=0 , w_old=0;
double sec=0 , old_sec=0 , dt=0;
double yaw, pitch, roll;

int j=0 , k=0, k_v=0 , k_w=0;
uint32_t size_path=0 , old_size_path=0 ;

bool  path_changed=false, path_stored=false;

ros::Publisher desired_traj_pub;
ros::Publisher cmd_publish;

geometry_msgs::Pose2D qd[1000],err;
geometry_msgs::Twist tw_msg;

tf::Point Odom_pos; 

double sat_angular_speed(double max , double min , double w_ref , double old_w);
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);
void check_path();
void get_errors();
void stop_mode();
void lon_control();
void lat_control();

void odom_back(const nav_msgs::Odometry msg) {

    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    odom_theta = tf::getYaw(msg.pose.pose.orientation);
    odom_x=Odom_pos.x();
    odom_y=Odom_pos.y();
}

void traj_back(const geometry_msgs::PoseArray msg) {

 //Odom_xx=Odom_pos.x();
  // int h=msg.header.seq;
   ROS_INFO("frame path: [%s]", msg.header.frame_id.c_str());
  // ROS_INFO("I heard  [%i]", h);
   size_path = msg.poses.size();
   ROS_INFO("size [%i]", size_path);
   if(size_path!=0 && old_size_path!=size_path)
   {
       path_changed=true;
       ROS_INFO("Path CHANGED [%i]", 0);
       ROS_INFO("  THE VALUES [%i]", 0);
       for (j=0; j<size_path ; j++)
       {   
           x_planning[j]=msg.poses[j].position.x;
           y_planning[j]=msg.poses[j].position.y;
           qd[j].x=x_planning[j];
           qd[j].y=y_planning[j];
           ROS_INFO("j,  x , y [%i ,%f , %f]" , j , qd[j].x, qd[j].y);
       }
       path_stored=true;
       ROS_INFO("Path Stored [%i]", 0);
       old_size_path=size_path;

   }
}

void check_path()
{
    if(path_changed)
    {
        k=0;
        k_v=0;
        k_w=0;
        ROS_INFO("K=0: [ %f  ]",  0.0);
        path_changed=false;

    }

    if(path_stored && k<old_size_path)
    {
       //ROS_INFO("PLOTTING: [ %f  ]",  0.0);
       desired_traj_pub.publish(qd[k]);
       k=k+1;
    }
}

void get_errors()
{
          //get the error between the odom and the next point in path
          err.x = (qd[k_v].x-odom_x) ; 
	      err.y =  (qd[k_v].y-odom_y) ;
          //ROS_INFO("angle: [ %f , %f  ]",  qd[k].x , qd[k].y);
          distance_linear_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
          double angle_linear_loop=atan2(err.y,err.x);
          err.x = (qd[k_w].x-odom_x) ; 
	      err.y =  (qd[k_w].y-odom_y) ;
          distance_angular_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
         // ROS_INFO("angle: [ %f  ]",  0.1);
          
          double angle_angular_loop=atan2((qd[k_w].y-odom_y),(qd[k_w].x-odom_x));
          //ROS_INFO("angle: [ %f  ]",  angle);
          err.theta =  angle_angular_loop - odom_theta;
         // ROS_INFO("angle error: [ %f  ]",  0.1);
          angle_angular_error = atan2(sin(err.theta),cos(err.theta)); // to make sure err.theta in [-pi,pi]
          err.theta =  angle_linear_loop - odom_theta;
          angle_linear_error = atan2(sin(err.theta),cos(err.theta)); // to make sure err.theta in [-pi,pi]
         // ROS_INFO("k_v, k_w ,  dist_er , angl_er: [%i,%i, %f , %f , %f , %f ]", k_v , k_w, distance_linear_error, distance_angular_error ,angle_linear_error, angle_angular_error);
}

void lon_control()
{
    if(path_stored && k_v<old_size_path)
    {
     
        if(abs(angle_angular_error)<0.5)
        {
          //  if(distance_linear_error>4)
            v_current = sat_linear_velocity(0.5,0.25,0.25 ,distance_linear_error ,  v_old);
          //  else if (distance_linear_error<= 4 &&  distance_linear_error >2 )
          //  v_current = sat_linear_velocity(1,0.25,-0.25 , v_old);
           // ROS_INFO("v_current: [ %f  ]",  v_current);
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }
      /*  else if (abs(angle_angular_error)<0.25 && abs(angle_angular_error)>=0.15)
        {
            //v_current = 0.05;
            v_current = sat_linear_velocity(1,0.1,0.25 ,distance_linear_error ,  v_old);
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }*/
        else if (abs(angle_angular_error)>0.5)
        {
            v_current = 0;
            tw_msg.linear.x=v_current;
            v_old=v_current;
        }
        
        if (distance_linear_error<distance_stop && k_v <old_size_path-1 || abs(angle_linear_error) <0.15 && k_v <old_size_path-1 )
        {
            k_v++;
        }
        
    }
}

void lat_control()
{
    if(path_stored && k_w<old_size_path)
    {
     e_dif = (angle_angular_error-old_angle_angular_err)/dt; 
     w_current=sat_angular_speed(1,0,angle_angular_error , w_old); 
        tw_msg.angular.z=w_current;// + 5*e_dif);
        w_old=0;
        if (distance_angular_error<0.5 && k_w <old_size_path-1)
        {
            k_w++;
            e_dif=0;
        }
        old_angle_angular_err = angle_angular_error;
        
    }
}

void stop_mode()
{
    if (path_stored && k_w==old_size_path-1 && k_v==old_size_path-1 && distance_linear_error < distance_stop)
    {
      //  ROS_INFO("STOP: [ %f  ]",  0.0);
        tw_msg.angular.z=0;
        tw_msg.linear.x=0;
    }
}

double sat_angular_speed(double max , double min , double w_ref , double old_w)
{
    double w_err=(w_ref-old_w);
    if(abs(w_err)>1 && v_current>0.6)
    {
        w_err=4*w_err;
    }
    else if (abs(w_err)>1 && v_current<=0.6)
    {
        w_err=2.8*w_err;
    }
    else if (abs(w_err)<=1 && abs(w_err)>0.5 && v_current>0.6)
    {w_err=2.8*w_err;}
    else if (abs(w_err)<=1 && abs(w_err)>0.5 && v_current<=0.6)
    {w_err=2.3*w_err;}
    else if (abs(w_err)<=0.5 && v_current>0.6)
    {
        w_err=2.3*w_err;
    }
    else if (abs(w_err)<=0.5 && v_current<0.6)
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

double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity)
{
    err_value = 0.25*(v_ref - old_velocity);
    if(old_velocity>err_value)
    err_value = old_velocity -0.25*dt;
    else
    err_value = old_velocity +0.25*dt;
    if (err_value<min)
    {
        return min;
    }
    else if (err_value>max)
    {
        return max;
    }
    else
    {
        return err_value;
    }
}

int main(int argc, char **argv) {
 
    ros::init(argc, argv, "control_husky");
    ros::NodeHandle n;

    cmd_publish= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber odom_subscriber = n.subscribe("odometry", 5, odom_back);
    ros::Subscriber traje_subscriber = n.subscribe("planning_node/trajectory",5 , traj_back);
    desired_traj_pub = n.advertise<geometry_msgs::Pose2D>("/desired_traj_pub",5);
    ros::Publisher errors_pub = n.advertise<geometry_msgs::Pose2D>("/errors_pub",1000);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(20); // ros spins 20 frames per second

    ROS_INFO("Start [%f , %f]" , 0.1 , 0.1);
    old_sec =ros::Time::now().toSec();

    while (ros::ok()) {

    sec =ros::Time::now().toSec();
    dt=sec-old_sec;
    old_sec=sec;
   // ROS_INFO("Time [%f]", dt);

    check_path();

    get_errors();

    lon_control();

    lat_control();

    stop_mode();
        
  //  ROS_INFO("old_size_path: [ %i  ]",  old_size_path);
    
    cmd_publish.publish(tw_msg);
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}




