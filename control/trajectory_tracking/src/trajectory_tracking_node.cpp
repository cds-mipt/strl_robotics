#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rtp_msgs/PathStamped.h>
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
#include "visualization_msgs/Marker.h"

// declare path parameters
double x_planning[100], y_planning[100] ;
double x_path[100], y_path[100];
double d[100] , interval_times[100] , time_path=0;
uint32_t size_path=0 , size=0 ;
bool  path_changed=false, path_stored=false;

// declare odometry parametrs
double odom_x, odom_y, odom_theta;
double init_x , init_y;

// declare controllers parameters
double ud1=0 , ud2=0 , u1=0 , u2=0 , u1_=0 , u2_=0;
double z1=0 , z2=0 , z3=0 , zd1=0 , zd2=0, zd3=0 , z1_=0 , z2_=0, z3_=0;
int j=0 , k=0, k1=-3 , k2=-6 , k3=-13;
double kx,ky,kz;

// time length parameter for every segment , time parameter and sample parameter
double dt=0 , t=0 ,ddt=0.1;

// declare desired velocities
double vd=0.5,wd=0;

// declare trajectory parameters
double xd,yd,xdd,ydd;
geometry_msgs::Pose2D qd , err;

// declare cmd value parameters
double omega = 0.0 , omega_past=0;
double speed = 0.0 , speed_past=0.1;

// declare different parameters
int h=0 , k_h=0;
int check=0;
bool test_1=true , test_2 = true;

// declare message parameters
geometry_msgs::Twist tw_msg;
visualization_msgs::Marker line_strip;


// declare Publishers and Subscribers
ros::Publisher   cmd_publish;
ros::Publisher   marker_pub;
ros::Subscriber  traje_subscriber;

// declare topics
std::string odom_topic;
std::string path_topic;
std::string cmd_topic;
std::string error_topic;
std::string draw_topic;
std::string global_frame;
std::string base_frame;

// declare config parameters
int control_method;

// declare functions
double sat_angular_speed(double max , double min , double w_ref , double old_w);
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);
void check_path();
void get_errors();
void get_data();
void stop_mode();
void first_controller();
void second_controller();
void third_controller();
double get_yaw(geometry_msgs::Quaternion q);
void Initialize_parameters(ros::NodeHandle n);


void traj_back(const geometry_msgs::PoseArray msg) {

   size_path = msg.poses.size();
   if(size_path!=0)
   {
       size = size_path;
       path_changed=true;
       for (j=0; j<size_path ; j++)
       {   
           x_planning[j]=msg.poses[j].position.x;
           y_planning[j]=msg.poses[j].position.y;
           x_path[j]=x_planning[j];
           y_path[j]=y_planning[j];
           ROS_INFO("j,  x , y [%i ,%f , %f]" , j , x_path[j], y_path[j]);
       }
       path_stored=true;
   }
}

// Initilize the new path
void check_path()
{
    if(path_changed && path_stored)
    {
        get_data();
        path_changed=false;
        t=0;
    }
}

// get the needed data from the path (length and time for every segment)
void get_data()
{
    test_1 = true;
    line_strip.points.clear();
    double stime=0;
    for (int i=0;i<size;i++)
{
    if (i==0)
    {
        d[i]=sqrt(pow((x_path[i]-odom_x),2)+pow((y_path[i]-odom_y),2));
    }
    else
    {
        d[i]=sqrt(pow((x_path[i]-x_path[i-1]),2)+pow((y_path[i]-y_path[i-1]),2));
    }
    if (i==0)
    {
        stime=d[i]/vd;
    }
    else
    {
        stime=(d[i]/vd) + stime;
    }
    // the time for every segment
    interval_times[i]=stime;
    // the time of the whole path
    time_path=stime;

    ROS_INFO("data: i , x , y , theta ,  d , time_coordinate [%i , %f , %f , %f, %f , %f]" , i , x_path[i] , y_path[i] , d[i] , d[i]/vd , time_path );

    init_x = odom_x;
    init_y = odom_y;

}

test_1 =false;

}

// to correct the initial angle
void correct_angle()
{
    double first_angles_path=atan2((y_path[1]-init_y),(x_path[1]-init_x));
    first_angles_path=atan2(sin(first_angles_path) , cos(first_angles_path));
    err.theta = first_angles_path - odom_theta;
	err.theta = atan2(sin(err.theta),cos(err.theta));
    if(abs(err.theta)>0.2)
    {
        tw_msg.linear.x=0;
        tw_msg.angular.z=0.25;
        cmd_publish.publish(tw_msg);
    }
    else 
    {
        tw_msg.linear.x=0;
        tw_msg.angular.z=0;
        cmd_publish.publish(tw_msg);
        test_2=false;
        t=0;
    }
}

// to make equation for every segment in path
void get_trajectory()
{
    if (t< interval_times[0])
    {
        k=0;
        dt=d[0]/vd;
        qd.x=((x_path[k]-init_x)/dt)*t+init_x;
        xd=(x_path[k]-init_x)/dt;
        qd.y=((y_path[k]-init_y)/dt)*t+init_y;
        yd=(y_path[k]-init_y)/dt;
        qd.theta = atan2(yd,xd);
     //   ROS_INFO("Coordinates and Error in Postion [%i %i %f %f %f %f %f %f ]" , 0 , size, qd.x , qd.y , odom_x , odom_y ,  t , interval_times[0]);
    }
    else
    {
       for (int l=0 ; l<size ; l++ )
       {
        if (t>=interval_times[l] && t<interval_times[l+1])
        {
            h=l+1;
            break;
        }
       }
        k=h;
        dt=d[h]/vd;
        qd.x=((x_path[k]-x_path[k-1])/dt)*(t-interval_times[h-1])+x_path[k-1];
        xd=(x_path[k]-x_path[k-1])/dt;
        qd.y=((y_path[k]-y_path[k-1])/dt)*(t-interval_times[h-1])+y_path[k-1];
        yd=(y_path[k]-y_path[k-1])/dt;
        qd.theta = atan2(yd,xd);
     //   ROS_INFO("Coordinates and Error in Postion [%i %i %f %f %f %f]" ,h , size ,qd.x , qd.y, t ,interval_times[h]);
    }
        geometry_msgs::Point p;
        p.x = qd.x;
        p.y = qd.y;

        line_strip.points.push_back(p);
        marker_pub.publish(line_strip);
    
}

// calculate Tracking errors between the robot and desired trajectory coordinates
void get_errors()
{
		err.x = (qd.x-odom_x) * cos(odom_theta ) + (qd.y-odom_y) * sin(odom_theta );
		err.y = -(qd.x-odom_x) * sin(odom_theta ) + (qd.y-odom_y) * cos(odom_theta );
		err.theta = (qd.theta - odom_theta );
		err.theta = atan2(sin(err.theta),cos(err.theta)); // Normalize theta_e between -pi and oi
}

void first_controller()
{
    ROS_INFO("First_controller");
    kx = 6; ky =10;
	kz = 10;
	speed = vd*cos(err.theta) + kx*err.x;
    omega =  wd + vd*ky*err.y + kz*sin(err.theta);

    if (vd==0.5)
    {
        if(speed>=0.5)
        {
           speed=0.5;
        }
        else if (speed<=0)
        {
           speed=0;
      
        }
    }
    /* if(speed>speed_past)
    {
        speed=speed_past+0.25*ddt;
    }
    else if (speed<speed_past)
    {
        speed=speed_past-0.25*ddt;
    }*/

    if (omega>=0.5)
    {
        omega=0.5;
    }
    else if (omega<=-0.5)
    {
        omega=-0.5;
    }  
    speed_past=speed;
    omega_past=omega;
}

void second_controller()
{
    ROS_INFO("Second_controller");
    speed = (xd+1*sin(atan(1*err.x)))*cos(odom_theta)+(yd+1*sin(atan(1*err.y)))*sin(odom_theta);
    omega = (xd+1*sin(atan(1*err.x)))*(-(1/1)*sin(odom_theta))+(yd+1*sin(atan(1*err.y)))*((1/1)*cos(odom_theta));
    if (vd==0.5)
    {
        if(speed>=0.5)
        {
           speed=0.5;
        }
        else if (speed<=0)
        {
           speed=0;
      
        }
    }
    /* if(speed>speed_past)
    {
        speed=speed_past+0.25*ddt;
    }
    else if (speed<speed_past)
    {
        speed=speed_past-0.25*ddt;
    }*/

    if (omega>=0.5)
    {
        omega=0.5;
    }
    else if (omega<=-0.5)
    {
        omega=-0.5;
    }  
    speed_past=speed;
    omega_past=omega;
}

void third_controller()
{
    ROS_INFO("Third_controller");
    ud1= xd;
    ud2=0;
    zd1=qd.x;
    zd2=yd/xd;
    zd3=qd.y;
    z1=odom_x;
    z2=tan(odom_theta);
    z3=odom_y;
    z1_=z1-zd1;
    z2_=z2-zd2;
    z3_=z3-zd3;
    u1_ = k1*z1_;
    u2_ = k2*z2_ + (k3/ud1)*z3_;
    speed = (u1_+ud1)/cos(odom_theta);
    omega= (u2_+ud2)/(1+pow(tan(odom_theta),2));
    if (vd==0.5)
    {
        if(speed>=0.5)
        {
           speed=0.5;
        }
        else if (speed<=0)
        {
           speed=0;
      
        }
    }
    /* if(speed>speed_past)
    {
        speed=speed_past+0.25*ddt;
    }
    else if (speed<speed_past)
    {
        speed=speed_past-0.25*ddt;
    }*/

    if (omega>=0.5)
    {
        omega=0.5;
    }
    else if (omega<=-0.5)
    {
        omega=-0.5;
    }  
    speed_past=speed;
    omega_past=omega;
}

void publish_cmd()
{
    tw_msg.linear.x=speed;
    tw_msg.angular.z=omega;
    cmd_publish.publish(tw_msg);
}

void stop_cmd()
{
    tw_msg.linear.x=0;
    tw_msg.angular.z=0;
    cmd_publish.publish(tw_msg);
}


// Initialize config parameters
void Initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_topic",    path_topic);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);

    n.getParam("control_method", control_method);
    ROS_INFO("[%i]" , control_method);
}


int main(int argc, char **argv) {
 
    ros::init(argc, argv, "trajectory_tracking_node");
    ros::NodeHandle n;

    Initialize_parameters(n);


    // Define the publishers and sunscribers
    cmd_publish         = n.advertise<geometry_msgs::Twist>    (cmd_topic, 1);
    traje_subscriber    = n.subscribe                          (path_topic,1 , traj_back);
    marker_pub          = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(10); // ros spins 20 frames per second

     line_strip.header.frame_id = global_frame;
     line_strip.header.stamp = ros::Time::now();
     line_strip.ns = "points_and_lines";
     line_strip.action = 0;
     line_strip.pose.orientation.w = 1.0;
     line_strip.type = 4;
     line_strip.scale.x = 0.2;
     line_strip.scale.y = 0.2;

         // Points are blue
     line_strip.color.b = 1.0f;
     line_strip.color.a = 1.0;

    while (t<=time_path &&ros::ok()) {

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
       if (!path_changed && path_stored && !test_1)
       {
           //if(test_2)
   /* {
        correct_angle();  // correct the angle of robot at the begining of motion
    }
    else
    {*/
        get_trajectory(); // realize the trajectory equations and get the coordinate of trajectory at time t.

        get_errors(); // get the erros at time t

        

        switch (control_method)
        {
        case 1:
            first_controller();
            break;
        case 2:
            second_controller();
            break;
        case 3:
            third_controller();
            break;
        default:
            break;
        }

        publish_cmd();

        t=t+ddt;
        
  //  }
       }
       
    ros::spinOnce();
    loop_rate.sleep();
    }
    stop_cmd();
    return 0;
}

double get_yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
