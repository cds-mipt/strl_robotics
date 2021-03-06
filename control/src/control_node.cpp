
/*
This is a control node for controlling the motion of differential drive mobile robot. The goal here is to track a path 
that is produced by a planning node. The planning node gives only the Cartesian coordinates in a local coordinate system.
The control node contains three different algorithms:
1- path following method which provides tracking the path regardless the velocities
2- trajectory tracking method which provides tracking the path tacking in account the time and velocities of the trajectory
3- combined control method which cobines the last two methods
*/
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include "visualization_msgs/Marker.h"
#include <random>
#include <stdio.h>
#include <time.h>
#include <fstream>

using namespace std;

// declare error parameters
double distance_error=0 , angle_error=0 , d_error=0 , a_error = 0 , next_engle_error=0;

// declare condition parametrs
double  distance_stop=1;

// declare control parameters
double v_cmd=0 , w_cmd=0;

// declare velocities mesurements
double v_robot=0,w_robot=0;

// declare integers k_v is index depends on position and h is index depends on time
int k_v=0 , h;

double d_stop=0 , dist_to_path=0;


double average_distance=0 , sum=0 , max_dist=0;
int index2=0;


// declare path parameters
double d[100] , interval_times[100] , time_path=0;
double taken_time=0;
uint32_t size_path=0 , size=0;
bool  path_stored=false;

// declare odometry parametrs
double odom_x, odom_y, odom_theta;
double init_x , init_y;
std_msgs::ColorRGBA c;

double old_angle_path=0;

// time length parameter for every segment , time parameter and sample parameter
double length_time_seg=0 , t=0 ,dt=0.1;

// declare trajectory parameters
double xd,yd;
geometry_msgs::Pose2D qt , qp[100] , err;

// declare cmd value parameters
double omega_past=0;
double speed_past=0;

double max_v_local=0;

double angle_lift=0;

bool to_lift=false;

tf::Point Odom_pos; 


// declare states 
bool request_point_sent  = false , written_to_file = true , request_replan_sent=false , start_motion=true;

ofstream outFile;

// declare message parameters
geometry_msgs::Twist tw_msg;
visualization_msgs::Marker traject_path;
visualization_msgs::Marker robot_path;
geometry_msgs::Point f;
geometry_msgs::PoseStamped goal_point_msg;
std_msgs::Bool request_point_msg;
std_msgs::Bool request_replan_msg;
std_msgs::Float32 v1, v2;
std_msgs::Float32 x_tf, y_tf;

std_msgs::Float32 v_robot_msg, w_robot_msg;

// declare Publishers and Subscribers
ros::Publisher   v_path_pub;
ros::Publisher   v_traj_pub;  
ros::Publisher   cmd_pub;
ros::Publisher   marker_pub;
ros::Publisher   marker_path_pub;
ros::Publisher   request_point_pub;
ros::Publisher   request_replan_pub;
ros::Subscriber  traj_sub;
ros::Subscriber  angle_lift_sub;
ros::Subscriber  odom_sub;
ros::Publisher   odom_tf_pub_x;
ros::Publisher   odom_tf_pub_y;


// declare topics
std::string path_topic;
std::string cmd_topic;
std::string velocities_topic;
std::string angle_lift_topic;
std::string global_frame;
std::string base_frame;
std::string odometry_topic;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;
double kx,ky,kz;
bool real_or_sim;
double method;
double vd;
double wd;
// declare functions
double sat_angular_speed(double max , double min , double w_ref);
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);
void stop_mode();
void lon_control();
void lat_control();
void get_path_errors();
void get_trajectory_errors();
void get_trajectory();
void get_data();
void trajectory_tracking_controller();
void cal_average_max(int g);
void request_point();
void reset_parameters();
double distance_to_path(double x1, double x2 , double y1 , double y2);
void Initialize_parameters(ros::NodeHandle n);
void request_replanning();
void check_needed_to_replan();
void publish_marker_odom();
void rotation_to_lift();


// Callback to get the Path from Planning node
void traj_back(const geometry_msgs::PoseArray msg) 
{
   size = msg.poses.size();
   if(size!=0)
   {
       ROS_INFO("New Path");
       for (int j=0; j<size ; j++)
       {   
           qp[j].x=msg.poses[j].position.x;
           qp[j].y=msg.poses[j].position.y;
           ROS_INFO("x , y %f %f", qp[j].x , qp[j].y );
       }
       size_path = size;
        path_stored = true;
        init_x = odom_x;                              // is needed to calculate the dist_to_path
        init_y = odom_y;                              // is needed to calculate the dist_to_path
        get_data();
        get_path_errors();
        stop_mode();
        request_point_sent = false;
        written_to_file = false;
        request_replan_sent=false;
        to_lift = false;
 
   }
   else
   {
       ROS_WARN("There is no Path");
       tw_msg.linear.x=0;
       tw_msg.angular.z=0;
       cmd_pub.publish(tw_msg);
       path_stored = false;
       request_point_sent = false;
       request_replan_sent=false;
       v_cmd = 0;
       w_cmd = 0;
   }
}

void odom_back(const nav_msgs::Odometry msg) {

    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    odom_theta = tf::getYaw(msg.pose.pose.orientation);
    odom_x=Odom_pos.x();
    odom_y=Odom_pos.y();


               
}

void angle_lift_back(const std_msgs::Float32 msg)
{
    angle_lift=msg.data;
}

// calculate average and max of the distance to path
void cal_average_max(int g)
{
    if(path_stored)
    {
        if (g==0)
    {
        dist_to_path = distance_to_path(init_x,qp[g].x,init_y,qp[g].y);
    }
    else if (g<size_path)
    {
        dist_to_path = distance_to_path(qp[g-1].x,qp[g].x,qp[g-1].y,qp[g].y);
    }

    if(dist_to_path>=0)
    {
        index2 = index2+1;
        sum = sum+dist_to_path;
        average_distance = sum / index2;
        if(dist_to_path>max_dist)
        {
            max_dist = dist_to_path;
        }
    }
    }
    
}

////////////////////// PATH FOLLOWING FUNCTIONS ///////////////


void path_following_controller()
{
    lon_control();
    lat_control();
}

// Get the errors between the robot and path
void get_path_errors()
{
    if(path_stored)
    {
        //Calculate the erros between the robot and the current segement
          err.x = (qp[k_v].x-odom_x) ; 
	      err.y =  (qp[k_v].y-odom_y) ;
          distance_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
          double angle_path=atan2(err.y,err.x);
          err.theta =  angle_path - odom_theta;
          angle_error = atan2(sin(err.theta ),cos(err.theta ));

    // accordingly to these errors move to the next segment      
          if (distance_error<distance_stop && k_v <size_path-1)
          {
            k_v++;
          }

          
    //Calculate the erros between the robot and the next segement
          err.x = (qp[k_v+1].x-qp[k_v].x) ; 
	      err.y =  (qp[k_v+1].y-qp[k_v].y) ;
          double next_angle_path=atan2(err.y,err.x);
          err.theta =  next_angle_path - odom_theta;
          next_engle_error = atan2(sin(err.theta ),cos(err.theta ));
    }  
   
}

// Linear Speed Controller
void lon_control()
{
    if(path_stored && k_v<size_path)
    {
        if (real_or_sim)
        {
            if(abs(angle_error)<=0.25)                    
            {
                v_cmd = sat_linear_velocity(max_v,min_v,acc_v ,distance_error, v_robot);
            }
            else if (abs(angle_error)>0.25 && v_cmd <= 0.2 && k_v==0)     
            {
                v_cmd = 0;
            }
            else if (abs(angle_error)>0.25 && k_v>0)      
            {
                max_v_local=v_cmd;
                if (max_v_local>0.3)
                {
                    max_v_local=max_v_local-0.02;
                }
                v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,distance_error, v_robot);
            }
        }
        else
        {
            //saturations when decreasing speed at the last segment
            ROS_INFO("%i %i %f %f ", size_path , k_v , distance_error , angle_error);
            if (k_v == size_path-1 && distance_error<1.5 && abs(angle_error)<=0.25 && v_cmd>0.3)
            {
                max_v_local=v_cmd;
                if (max_v_local>0.3)
                {
                    max_v_local=max_v_local-0.02;
                }
                v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,distance_error, v_cmd);
            }
            else if(abs(angle_error)<=0.25)                    
            {
                v_cmd = sat_linear_velocity(max_v,min_v,acc_v ,distance_error, v_cmd);
            }
            else if (abs(angle_error)>0.25 && v_cmd <= 0.2 && k_v==0)     
            {
                v_cmd = sat_linear_velocity(0.05,min_v,acc_v ,distance_error, v_cmd);
            }
            else if (abs(angle_error)>0.25 && k_v>0)      
            {
                max_v_local=v_cmd;
                if (max_v_local>0.3)
                {
                    max_v_local=max_v_local-0.02;
                }
                v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,distance_error, v_cmd);
            }
        }   

        tw_msg.linear.x=v_cmd;
        v1.data=v_cmd;
        v2.data=0;
        v_path_pub.publish(v1);
        v_traj_pub.publish(v2);
    }
}

// Angular Speed Controller
void lat_control()
{
    if(path_stored && k_v<size_path)
    {
        w_cmd=sat_angular_speed(max_w,0,angle_error); 
        tw_msg.angular.z=w_cmd;
    }
}

// Stop Mode
void stop_mode()
{
    d_stop = sqrt(pow((qp[size_path-1].x-odom_x), 2) + pow((qp[size_path-1].y-odom_y), 2));
    if (d_stop<0.2)
    {
       path_stored=false;
    }
}

void request_point()
{
    request_point_msg.data = true;
    request_point_pub.publish(request_point_msg);
}

void request_replanning()
{
    request_replan_msg.data = true;
    request_replan_pub.publish(request_replan_msg);
    ROS_INFO("Request Replanning");
}

void reset_parameters()
{
    index2=0;
    sum=0;
    average_distance=0;
    max_dist=0;
    v_cmd = 0;
    w_cmd = 0;
    taken_time = 0;
    start_motion = true;
}

void check_needed_to_replan()
{
    if (err.x > 0.5 || err.y>0.5)
    {
            request_replanning();
            request_replan_sent=true;
            ROS_INFO("Request Replanning");
               
    }
}

void publish_marker_odom()
{
    f.x=odom_x;
    f.y=odom_y;
    robot_path.colors.push_back(c);
    robot_path.points.push_back(f);
    marker_path_pub.publish(robot_path);
}

void publish_odom_tf()
{
    x_tf.data = odom_x;
    y_tf.data = odom_y;
    odom_tf_pub_x.publish(x_tf);
    odom_tf_pub_y.publish(y_tf);
}

void rotation_to_lift()
{

    err.theta =  0 - odom_theta;
    angle_error = atan2(sin(err.theta ),cos(err.theta ));

    if (abs(err.theta)>=0.05 && !to_lift)
    {
        w_cmd=sat_angular_speed(max_w,0,angle_error); 
        tw_msg.angular.z=w_cmd;
        ROS_INFO("theta %f ", odom_theta);
    }
    else if (abs(err.theta)<0.05 && !to_lift)
    {
        path_stored = false;
        tw_msg.angular.z=0;
        to_lift = true;
    }
    

}


// Saturation Angular Speed Function
double sat_angular_speed(double max , double min , double w_ref)
{
    double w_err=w_ref;
    if(abs(w_err)>=1 && v_cmd>=0.6)
    {
        w_err=3*w_err;
    }
    else if (abs(w_err)>=1 && v_cmd<0.6)
    {
        w_err=3*w_err;
    }
    else if (abs(w_err)<1 && abs(w_err)>=0.35 && v_cmd>=0.6)
    {
        w_err=3*w_err;
    }
    else if (abs(w_err)<1 && abs(w_err)>=0.35 && v_cmd<0.6)
    {
        w_err=3*w_err;
    }
    else if (abs(w_err)<0.35 && v_cmd>=0.6)
    {
        w_err=3*w_err;
    }
    else if (abs(w_err)<0.35 && v_cmd<0.6)
    {
        w_err=1*w_err;
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
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real)
{
    double v_diff = v_ref - v_real;
    // saturations on accelerations
    if(v_diff>=0)
    {
        v_diff = v_real +accel*dt;
    }
    else if(v_diff<0)
    {
        v_diff = v_real -accel*dt;
    }

    if (v_diff<min)
    {
        return min;
    }
    else if (v_diff>max)
    {
        return max;
    }
    else
    {
        return v_diff;
    }
}

double distance_to_path(double x1, double x2 , double y1 , double y2)
{

    double dist=abs((x2-x1)*(y1-odom_y)-(x1-odom_x)*(y2-y1))/sqrt(pow((x2-x1),2)+pow((y2-y1),2));
    return dist;

}

////////////////////// TRAJECTORY TRACKING FUNCTIONS /////////////////

void get_data()
{
    traject_path.points.clear();
    double seg_time=0;
    h=0;
    t=0;
    k_v=0;


    // get the times for every segment  and for all the path // it is called one time when the path is updated
    for (int i=0;i<size_path;i++)
{
    if (i==0)
    {
        d[i]=sqrt(pow((qp[i].x-odom_x),2)+pow((qp[i].y-odom_y),2));
    }
    else
    {
        d[i]=sqrt(pow((qp[i].x-qp[i-1].x),2)+pow((qp[i].y-qp[i-1].y),2));
    }
    if (i==0)
    {
        seg_time=d[i]/vd;
    }
    else
    {
        seg_time=(d[i]/vd) + seg_time;
    }
    // the time for every segment
    interval_times[i]=seg_time;
    // the time of the whole path
    time_path=seg_time;

    init_x = odom_x;
    init_y = odom_y;


}
}


// calculate x and y of the trajectory at every moment of time  // called every cycle of the loop
void get_trajectory()
{
    if (t< interval_times[0])
    {
        h=0;
        length_time_seg=d[0]/vd;
        qt.x=((qp[h].x-init_x)/length_time_seg)*t+init_x;
        xd=(qp[h].x-init_x)/length_time_seg;
        qt.y=((qp[h].y-init_y)/length_time_seg)*t+init_y;
        yd=(qp[h].y-init_y)/length_time_seg;
        qt.theta = atan2(yd,xd);
    }
    else
    {
       for (int l=0 ; l<size_path ; l++ )
       {
        if (t>=interval_times[l] && t<interval_times[l+1])
        {
            h=l+1;
            break;
        }
       }
        length_time_seg=d[h]/vd;
        qt.x=((qp[h].x-qp[h-1].x)/length_time_seg)*(t-interval_times[h-1])+qp[h-1].x;
        xd=(qp[h].x-qp[h-1].x)/length_time_seg;
        qt.y=((qp[h].y-qp[h-1].y)/length_time_seg)*(t-interval_times[h-1])+qp[h-1].y;
        yd=(qp[h].y-qp[h-1].y)/length_time_seg;
        qt.theta = atan2(yd,xd);
       
    }

        geometry_msgs::Point p;
        p.x = qt.x;
        p.y = qt.y;

        traject_path.points.push_back(p);
        marker_pub.publish(traject_path);
   
}


// calculate Tracking errors between the robot and desired trajectory coordinates
void get_trajectory_errors()
{
		err.x = (qt.x-odom_x) * cos(odom_theta ) + (qt.y-odom_y) * sin(odom_theta );
		err.y = -(qt.x-odom_x) * sin(odom_theta ) + (qt.y-odom_y) * cos(odom_theta );
		err.theta = (qt.theta - odom_theta );
		err.theta = atan2(sin(err.theta),cos(err.theta)); // Normalize theta_e between -pi and oi
        a_error = err.theta;
}

void trajectory_tracking_controller()
{
    speed_past=v_cmd;
    omega_past=w_cmd;
	  wd=(qt.theta - old_angle_path)/dt;
    old_angle_path = qt.theta;

	  v_cmd = vd*cos(err.theta) + kx*err.x;
    w_cmd =  wd + vd*ky*err.y + kz*sin(err.theta);

    
    if(v_cmd>speed_past)
    {
        v_cmd=speed_past+acc_v*dt;
    }
    else if (v_cmd<speed_past)
    {
        v_cmd=speed_past-acc_v*dt;
    }

  // saturations
    if (v_cmd >max_v)
    {
        v_cmd =max_v;
    }
    else if (v_cmd<=0)
    {
       v_cmd=0;
    }

    if (w_cmd>=max_w)
    {
        w_cmd=max_w;
    }
    else if (w_cmd<=-max_w)
    {
        w_cmd=-max_w;
    }
    
    v1.data=0;
    v2.data=v_cmd;
    v_path_pub.publish(v1);
    v_traj_pub.publish(v2);
    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
}

// Initialize config parameters
void Initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_topic",    path_topic);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("velocities_topic" ,  velocities_topic);
    n.getParam("angle_lift_topic" ,  angle_lift_topic);
    n.getParam("odometry_topic" ,  odometry_topic);

    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
    n.getParam("kx", kx);
    n.getParam("ky", ky);
    n.getParam("kz", kz);
    n.getParam("real_or_sim", real_or_sim);
    n.getParam("method", method);
    n.getParam("vd", vd);
    n.getParam("wd", wd);
}


int main(int argc, char **argv) {
 
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    Initialize_parameters(n);


    // Define the publishers and sunscribers
    cmd_pub             = n.advertise<geometry_msgs::Twist>         (cmd_topic, 1);
    traj_sub            = n.subscribe                               (path_topic,10 , traj_back);
    marker_pub          = n.advertise<visualization_msgs::Marker>   ("marker_traj", 1);
    marker_path_pub     = n.advertise<visualization_msgs::Marker>   ("marker_real", 1);
    v_path_pub          = n.advertise<std_msgs::Float32>            ("v_path_topic",1);
    v_traj_pub          = n.advertise<std_msgs::Float32>            ("v_traj_topic",1);
    odom_tf_pub_x       = n.advertise<std_msgs::Float32>            ("odom_x_tf",1);
    odom_tf_pub_y       = n.advertise<std_msgs::Float32>            ("odom_y_tf",1);
    angle_lift_sub      = n.subscribe                               (angle_lift_topic,1 , angle_lift_back);
    request_point_pub   = n.advertise<std_msgs::Bool>               ("next_point_topic", 1);
    request_replan_pub  = n.advertise<std_msgs::Bool>               ("replan", 10);
    odom_sub            = n.subscribe                               (odometry_topic, 5, odom_back);



    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(10); // ros spins 20 frames per second

    //outFile.open(("/home/cont/catkin_ws/src/control/generator_point/src/dat.txt"));

     traject_path.header.frame_id = global_frame;
     traject_path.header.stamp = ros::Time::now();
     traject_path.ns = "points_and_lines";
     traject_path.action = 0;
     traject_path.pose.orientation.w = 1.0;
     traject_path.type = 4;
     traject_path.scale.x = 0.2;
     traject_path.scale.y = 0.2;

     traject_path.color.r = 1.0f;
     traject_path.color.a = 1.0;

     robot_path.header.frame_id = global_frame;
     robot_path.header.stamp = ros::Time();
     robot_path.ns = "points_and_lines";
     robot_path.action = visualization_msgs::Marker::ADD;
     robot_path.pose.orientation.w = 1.0;
     robot_path.type = 4;
     robot_path.scale.x = 0.2;
     robot_path.scale.y = 0.2;

     robot_path.color.a = 1.0;
     robot_path.color.r = 0.0;
     robot_path.color.g = 1.0;
     robot_path.color.b = 0.0;
     robot_path.points.clear();

    while (ros::ok()) {

      if (path_stored) 
    {
        if(start_motion)
        {
            ROS_INFO("Start Motion");
            ROS_INFO("Position Robot %f %f", odom_x , odom_y);
           // outFile << odom_x <<" , "<< odom_y<< " , "<< sum <<" , "<< index2<< " , "<<average_distance<<" , "<<max_dist<<" , "<<t<<endl;
            start_motion=false;
        }
        if (method==1)
        {
            get_path_errors();
            path_following_controller();
            cal_average_max(k_v);
            c.r = 0.0;
            c.b = 1;
            c.g = 0;
            publish_marker_odom();
            taken_time = taken_time+dt;
        }
        else if (method==2 && t<= time_path)
        {
            get_path_errors();
            get_trajectory();
            get_trajectory_errors(); // get the erros at time t
            cal_average_max(k_v);
            d_error = sqrt(pow((qt.x-odom_x),2)+pow((qt.y-odom_y),2));

            if (distance_error<0.6 && abs(next_engle_error) >0.25)
            {
                c.r = 0.0;
                c.b = 1;
                c.g = 0;
                path_following_controller();
            }
            else if (abs(a_error)>0.3)
            {
                c.r = 0.75;
                c.b = 0.66;
                c.g = 0;
                path_following_controller();
            }
            else if (d_error>0.3 && abs(a_error)>0.12)
            {
                c.r = 0.75;
                c.b = 0.0;
                c.g = 0.5;
                path_following_controller();
            }
            else
            {
                c.g = 1;
                c.r = 0;
                c.b = 0;
                trajectory_tracking_controller();
            }
            publish_marker_odom();
            check_needed_to_replan();
            t=t+dt;
            taken_time = taken_time+dt;
        }
        else if (method==3 && t<= time_path)
        {
            get_trajectory();
            get_trajectory_errors(); // get the erros at time t
            cal_average_max(h);
            c.g = 1;
            c.r = 0;
            c.b = 0;
            trajectory_tracking_controller();
            publish_marker_odom();
            check_needed_to_replan();
            t=t+dt;
            taken_time = taken_time+dt;
        }
        else
        {
            path_stored = false;
        }
        stop_mode();
        ROS_INFO("%f , %f", w_cmd , odom_theta);
        
    }
    else
        {
            tw_msg.linear.x=0;
            tw_msg.angular.z=0;
            cmd_pub.publish(tw_msg);
            if(!written_to_file)
            {
               // outFile << qp[size_path-1].x<<" , "<<qp[size_path-1].y<< " , "<< odom_x<<" , "<< odom_y<< " , "<<average_distance<<" , "<<max_dist<<" , "<<taken_time<<endl;
                ROS_INFO("Complete path");
                ROS_INFO("odometry %f , %f , %f" , odom_x ,odom_y , odom_theta);
                ROS_INFO("x y(local_lidar_map) avarage max time %f , %f , %f , %f , %f", qp[size_path-1].x,qp[size_path-1].y, average_distance , max_dist , taken_time);
                written_to_file = true;
                
            } 
            if (!request_point_sent)
            {
                request_point();
                request_point_sent = true;
            }
            reset_parameters();
        }
        
        cmd_pub.publish(tw_msg); 
      
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

