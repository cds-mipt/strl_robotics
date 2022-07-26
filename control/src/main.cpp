/*
This is a control node for controlling the motion of differential drive mobile robot. The goal here is to track a path 
that is produced by a planning node. The planning node gives only the Cartesian coordinates in a local coordinate system.
The control node depends on path following method which provides tracking the path regardless the velocities
*/
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <stdio.h>
#include <time.h>
#include <fstream>

using namespace std;

// controller parameters
double dist_error = 0;
double angle_error = 0;
double dist_limit = 1.05;
double dist_stop = 0;
double max_v_local = 0;
int k_v = 0;

// velocities paramters
double v_cmd = 0;
double w_cmd = 0;
double v_robot = 0;
double w_robot = 0;

// motion feature paramters
double j_cost = 0;
double taken_time = 0;
double average_distance = 0;
double sum = 0;
double max_dist = 0;
double dist_to_path = 0;
double dist_to_path_current_segment = 0;
double dist_to_path_past_segment = 0;
double dist_to_path_pre_past_segment = 0;
int counts = 0;
int index2 = 0;

// intersection point parameters
double x_inter = 0;
double y_inter = 0;
double d_inter = 0;

// path parameters
uint32_t size_path = 0 , size = 0;
double final_orientation = 0;

// odometry parametrs
double odom_x;
double odom_y;
double odom_theta;
double init_x;
double init_y;

// desired point paramters
double x_desired =0;
double y_desired=0;
double xg = 0;
double yg =0 ;

// rate time parameter
double dt = 0.1;

// boolean paramters
bool show_info = true;
bool to_lift = false;
bool path_stored = false;
bool path_changed = false;
bool robot_reached = false;
bool written_to_file = true;
bool modify_desired_point = true;
bool robot_orientation_corrected = false;

// writing file paramter
ofstream outFile;

// declare message parameters
tf::Point odom_pos;
visualization_msgs::Marker  robot_mk;
visualization_msgs::Marker  dist_mk, desired_mk;
geometry_msgs::Point        robot_pt, end_pt , desired_pt;
geometry_msgs::Twist        tw_msg;
geometry_msgs::Pose2D       qp[100] , new_qp[100], err;
std_msgs::String            status_msg;

// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   robot_mk_pub;
ros::Publisher   dist_mk_pub;
ros::Publisher   desired_mk_pub;
ros::Publisher   status_control_pub;
ros::Subscriber  path_sub_1;
ros::Subscriber  path_sub_2;
ros::Subscriber  odom_sub;

// declare topics
std::string path_topic1;
std::string path_topic2;
std::string cmd_topic;
std::string global_frame;
std::string base_frame;
std::string odometry_topic;
std::string status_topic;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;
bool correct_orient_at_last_point;

// declare functions
// to check stop condition 
// will stop the robot when it is close to the goal
void stop_mode();
// calculate the errors between the robot and the path
void get_path_errors();
// to reset the paramters when there is no path or path completed
void reset_parameters();
// to rotate the robot at the last point to the desired orientation
void rotation_to_lift();
// to calculate the intersection point between the robot and path
// it is used only for marker
void cal_inter_point();
// to calculate the taken time and the cost of the motion
void cal_time_and_cost();
// to calculate the deviations of the robot from the path
void cal_average_max(int g);
// to initialize the markers
void initialize_markers();
// to publish three markers: robot position, desired point and
// the distance from robot to path
void publish_marker_odom();
// to write the motion results to a txt file
void write_result_to_file();
// the path following algorithm
void path_following_controller();
// to define subscribers and publishers
void define_sub_pub(ros::NodeHandle n);
// to initialize config paramters
void initialize_parameters(ros::NodeHandle n);
// to calculate the intersection point too
void get_line_equation(double x1, double x2 , double y1 , double y2, double odmx , double odmy);
// to get the orientation of yaw for the local frame and the last segment
double get_yaw(geometry_msgs::Quaternion q);
// to calculate the distance from the robot to the path
double distance_to_path(double x1, double x2 , double y1 , double y2);
// to set saturation on the linear velocity of the robot
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);

int ind_control = 0;

void path_back1(const geometry_msgs::PoseArray msg) 
{
   size = msg.poses.size();
   if(size!=0)
   {         
       if (size != size_path)
       {
           path_changed = true;
       }
       else
       {
           for (int j=0; j<size ; j++)
       {   
           path_changed = false;           
           new_qp[j].x=msg.poses[j].position.x;
           new_qp[j].y=msg.poses[j].position.y;
           if (abs(new_qp[j].x - qp[j].x)>0.1 || abs(new_qp[j].y - qp[j].y)>0.1)
           {
                path_changed = true;
                break;
           }
       }
       }
       

       if (path_changed)
       {
           ROS_INFO("New Path, it contains: %i points" , size);
           for (int j=0; j<size ; j++)
            { 
                qp[j].x=msg.poses[j].position.x;
                qp[j].y=msg.poses[j].position.y;
                ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                final_orientation = get_yaw(msg.poses[size-1].orientation);
            }
       
        size_path = size;
        path_stored = true;
        robot_reached = false;
        init_x = odom_x;                              // is needed to calculate the dist_to_path
        init_y = odom_y;                              // is needed to calculate the dist_to_path
        k_v=0;
        get_path_errors();
        stop_mode();
        written_to_file = false;
        to_lift = false;
       }
 
   }
   else
   {
       path_stored = false;
       v_cmd = 0;
       w_cmd = 0;
   }
}

void path_back2(const nav_msgs::Path msg) 
{
   size = msg.poses.size();
   if(size!=0)
   {
             
       if (size != size_path)
       {
           path_changed = true;
       }
       else
       {
           for (int j=0; j<size ; j++)
       {   
           path_changed = false;           
           new_qp[j].x=msg.poses[j].pose.position.x;
           new_qp[j].y=msg.poses[j].pose.position.y;
           if (abs(new_qp[j].x - qp[j].x)>0.1 || abs(new_qp[j].y - qp[j].y)>0.1)
           {
                path_changed = true;
                break;
           }
       }
       }
       if (path_changed)
       {
           ROS_INFO("Odometry Robot %f , %f" , odom_x , odom_y);
           ROS_INFO("New Path, it contains: %i points" , size);
           for (int j=0; j<size ; j++)
            { 
                qp[j].x=msg.poses[j].pose.position.x;
                qp[j].y=msg.poses[j].pose.position.y;
                ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                final_orientation = get_yaw(msg.poses[size-1].pose.orientation);
            }
       
        size_path = size;
        path_stored = true;
        robot_reached = false;
        init_x = odom_x;                              // is needed to calculate the dist_to_path
        init_y = odom_y;                              // is needed to calculate the dist_to_path
        k_v=0;
        get_path_errors();
        stop_mode();
        written_to_file = false;
        to_lift = false;
       }
       show_info = true;
 
   }
   else
   {
       path_stored = false;
       if(show_info)
       {
           ROS_INFO("Odometry Robot %f , %f" , odom_x , odom_y);
           ROS_INFO("NO PATH");
           show_info = false;
       }
       v_cmd = 0;
       w_cmd = 0;
   }
}

void get_line_equation(double x1, double x2 , double y1 , double y2, double odmx , double odmy)
{
    // slop of the segment
    double m = (y2-y1)/(x2-x1);
    // prependicular slop
    double m_p = -1/m;

    // coordinate of the intersection point
    x_inter = (m * x1 - m_p * odmx - y1 + odmy)/(m-m_p);
    y_inter = m*(x_inter-x1) + y1;

    // d_inter must be equal to dist_to_path
    d_inter = sqrt((odmx-x_inter)*(odmx-x_inter)+(odmy-y_inter)*(odmy-y_inter));
}

void odom_back(const nav_msgs::Odometry msg) {


  /*  tf::pointMsgToTF(msg.pose.pose.position, odom_pos);
    odom_theta = tf::getYaw(msg.pose.pose.orientation);
    odom_x=odom_pos.x();
    odom_y=odom_pos.y();*/
  //  double theta = tf::getYaw(msg.pose.pose.orientation);

   // v_robot = msg.twist.twist.linear.x * cos(odom_theta) + msg.twist.twist.linear.y * sin(odom_theta);
    
    v_robot = abs(msg.twist.twist.linear.x);
   // ROS_INFO("%f" , v_robot);
  //  w_robot = msg.twist.twist.angular.z;

               
}

void cal_average_max(int g)
{
    if(path_stored)
    {
        if (g==0)
    {
        dist_to_path = distance_to_path(init_x,qp[g].x,init_y,qp[g].y);
        dist_to_path_current_segment = dist_to_path;
        dist_to_path_past_segment = 1000;
        dist_to_path_pre_past_segment = 1000;
    }
    else if (g==1)
    {
        dist_to_path_current_segment = distance_to_path(qp[g-1].x,qp[g].x,qp[g-1].y,qp[g].y);
        dist_to_path_past_segment = distance_to_path(init_x,qp[g-1].x,init_y,qp[g-1].y);
        dist_to_path_pre_past_segment = 1000;
        dist_to_path = std::min(dist_to_path_current_segment,dist_to_path_past_segment);
    }
    else if (g==2)
    {
        dist_to_path_current_segment = distance_to_path(qp[g-1].x,qp[g].x,qp[g-1].y,qp[g].y);
        dist_to_path_past_segment = distance_to_path(qp[g-2].x,qp[g-1].x,qp[g-2].y,qp[g-1].y);
        dist_to_path_pre_past_segment = distance_to_path(init_x,qp[g-2].x,init_y,qp[g-2].y);
        if (dist_to_path_current_segment<=dist_to_path_past_segment && dist_to_path_current_segment<=dist_to_path_pre_past_segment)
        {
            dist_to_path = dist_to_path_current_segment;
        }
        else if (dist_to_path_past_segment<=dist_to_path_current_segment && dist_to_path_past_segment<=dist_to_path_pre_past_segment)
        {
            dist_to_path = dist_to_path_past_segment;
        }
        else if (dist_to_path_pre_past_segment<=dist_to_path_current_segment && dist_to_path_pre_past_segment<=dist_to_path_past_segment)
        {
            dist_to_path = dist_to_path_pre_past_segment;
        }
    }
    else if (g>2)
    {
        dist_to_path_current_segment = distance_to_path(qp[g-1].x,qp[g].x,qp[g-1].y,qp[g].y);
        dist_to_path_past_segment = distance_to_path(qp[g-2].x,qp[g-1].x,qp[g-2].y,qp[g-1].y);
        dist_to_path_pre_past_segment = distance_to_path(qp[g-3].x,qp[g-2].x,qp[g-3].y,qp[g-2].y);
        if (dist_to_path_current_segment<=dist_to_path_past_segment && dist_to_path_current_segment<=dist_to_path_pre_past_segment)
        {
            dist_to_path = dist_to_path_current_segment;
        }
        else if (dist_to_path_past_segment<=dist_to_path_current_segment && dist_to_path_past_segment<=dist_to_path_pre_past_segment)
        {
            dist_to_path = dist_to_path_past_segment;
        }
        else if (dist_to_path_pre_past_segment<=dist_to_path_current_segment && dist_to_path_pre_past_segment<=dist_to_path_past_segment)
        {
            dist_to_path = dist_to_path_pre_past_segment;
        }
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

void path_following_controller()
{

    ROS_INFO("kv siz ind errors %i %i %i %f %f %f %f" , k_v , size_path , ind_control, dist_error , angle_error , v_cmd , v_robot);

    //  DECREASING
    if (k_v == size_path-1 && dist_error<1.5 && v_cmd>0.3)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        max_v_local=v_cmd;
        if (max_v_local>0.3)
        {
            max_v_local=max_v_local-0.04;
        }
        v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,dist_error, v_cmd);
        ind_control = 1;
    }
    else
    {
        // Motion 
        if(abs(angle_error)<0.1)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(max_v,0,acc_v,dist_error , v_cmd);
            w_cmd = angle_error;
            ind_control = 2;
        }
        else if(abs(angle_error)>=0.1 && abs(angle_error)<0.4)  // case error angle small
        {
            v_cmd = sat_linear_velocity(max_v-0.1,0,acc_v,dist_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 3;
        }
        else if(abs(angle_error)>=0.4 && abs(angle_error)<0.75)  // case error angle medium
        {
            v_cmd = sat_linear_velocity(max_v-0.2,0,acc_v,dist_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 4;
        }
        else if(abs(angle_error)>=0.75 && abs(angle_error)<1.1)  // case error angle big
        {
            v_cmd = sat_linear_velocity(max_v-0.3,0,acc_v,dist_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 5;
        }
        else if(abs(angle_error)>=1.1 && abs(angle_error)<1.4)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(max_v-0.45,0,acc_v,dist_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 6;   
        }
        else if(abs(angle_error)>=1.4)
        {
            v_cmd = 0;
            w_cmd = 1*angle_error;
            ind_control = 7;
        }
        
    }

 //   ROS_INFO("vel 1 %f" , v_cmd);
    
    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v==0 && size_path > 1) // v_cmd instead of v_robot for simulation
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
        ind_control = 8;
    }
    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}
    if(v_cmd>max_v)
    {v_cmd = max_v;}
    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
    
  
}

void get_path_errors()
{
    double angle_path2=0;
    if(path_stored)
    {
        //Calculate the erros between the robot and the current segement
          err.x = (qp[k_v].x-odom_x) ; 
	      err.y =  (qp[k_v].y-odom_y) ;
          dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
          
          double angle_path1=atan2(err.y,err.x);
          err.theta =  angle_path1 - odom_theta;
          angle_error = atan2(sin(err.theta ),cos(err.theta ));

        //Calculate the erros between the robot and the next segement
        if(k_v<(size_path-1))
        {
          err.x = (qp[k_v+1].x-qp[k_v].x);
	      err.y =  (qp[k_v+1].y-qp[k_v].y);
          double next_angle_path=atan2(err.y,err.x);
          err.theta =  next_angle_path - odom_theta;
        }
        // accordingly to these errors move to the next segment      
        if (dist_error<dist_limit && k_v <size_path-1)
        {
        k_v++;
        }

        // generate the desired point for the first segment
        if (dist_error > 1.5 && k_v==0)
        {
            angle_path2 = atan2(qp[k_v].y-init_y,qp[k_v].x-init_x);
            xg = (dist_error - 1.5) * cos(angle_path2);
            yg = (dist_error - 1.5) * sin(angle_path2);

        
            if ( qp[k_v].x >= init_x)
            {
                x_desired = qp[k_v].x - abs(xg);
            }
            else
            {
                x_desired = qp[k_v].x + abs(xg);
            }

            if (qp[k_v].y >=init_y)
            {
                y_desired = qp[k_v].y - abs(yg);
            }
            else
            {
                y_desired = qp[k_v].y + abs(yg);
            }
            angle_path2=atan2(y_desired - odom_y,x_desired - odom_x);
            err.theta =  angle_path2 - odom_theta;
            angle_error = atan2(sin(err.theta ),cos(err.theta ));
        }
        else if (dist_error > 1.5 && k_v>0)
        {
            angle_path2 = atan2(qp[k_v].y-qp[k_v-1].y,qp[k_v].x-qp[k_v-1].x);
            xg = (dist_error - 1.5) * cos(angle_path2);
            yg = (dist_error - 1.5) * sin(angle_path2);
            if (qp[k_v].x >= qp[k_v-1].x)
            {
                x_desired = qp[k_v].x - abs(xg);
            }
            else
            {
                x_desired = qp[k_v].x + abs(xg);
            }

            if (qp[k_v].y >= qp[k_v-1].y)
            {
                y_desired = qp[k_v].y - abs(yg);
            }
            else
            {
                y_desired = qp[k_v].y + abs(yg);
            }
   
            angle_path2=atan2(y_desired - odom_y,x_desired - odom_x);
            err.theta =  angle_path2 - odom_theta;
            angle_error = atan2(sin(err.theta ),cos(err.theta ));
        }
        else
        {
            x_desired = qp[k_v].x;
            y_desired = qp[k_v].y;
        } 
    }  
}

void stop_mode()
{
    dist_stop = sqrt(pow((qp[size_path-1].x-odom_x), 2) + pow((qp[size_path-1].y-odom_y), 2));
    if (dist_stop<=0.5)
    {
        path_stored=false;
        robot_reached=true;
        ROS_INFO("Robot Reached to the goal");
        status_msg.data="reached";
    }
    else
    {
        status_msg.data="not reached";
        robot_reached = false;
    }
    status_control_pub.publish(status_msg);
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
}

void publish_marker_odom()
{
    robot_pt.x=odom_x;
    robot_pt.y=odom_y;

    desired_pt.x = x_desired;
    desired_pt.y = y_desired;

    end_pt.x = x_inter;
    end_pt.y = y_inter;

    robot_mk.points.push_back(robot_pt);
    robot_mk_pub.publish(robot_mk);
    desired_mk.points.push_back(desired_pt);
    dist_mk.points.push_back(robot_pt);
    dist_mk.points.push_back(end_pt);
    dist_mk_pub.publish(dist_mk);
    desired_mk_pub.publish(desired_mk);
}

void rotation_to_lift()
{

    err.theta =  final_orientation - odom_theta;
    angle_error = atan2(sin(err.theta ),cos(err.theta ));
    if (abs(err.theta)>=0.05 && !to_lift && robot_reached)
    {
        w_cmd = angle_error;
        if (w_cmd > max_w)
        {
            w_cmd = max_w;
        }
        else if(w_cmd < -max_w)
        {
            w_cmd = -max_w;
        }
        tw_msg.angular.z=w_cmd;
    }
    else if (abs(err.theta)<0.05 && !to_lift && robot_reached)
    {
        path_stored = false;
        tw_msg.angular.z=0;
        to_lift = true;
        ROS_INFO("The ORIENTATION IS CORRECTED");
    }

}

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

void cal_inter_point()
{
    if (k_v==0)
    {
        get_line_equation(init_x , qp[k_v].x , init_y , qp[k_v].y, odom_x , odom_y);
    }
    else if (k_v==1)
    {
        if (dist_to_path_current_segment<=dist_to_path_past_segment)
        {
            get_line_equation(qp[k_v-1].x , qp[k_v].x , qp[k_v-1].y , qp[k_v].y, odom_x , odom_y);
        }
        else
        {
            get_line_equation(init_x , qp[k_v-1].x , init_y , qp[k_v-1].y, odom_x , odom_y);
        }
    
    }
    else if (k_v==2)
    {
        if (dist_to_path_current_segment<=dist_to_path_past_segment && dist_to_path_current_segment<=dist_to_path_pre_past_segment)
        {
            get_line_equation(qp[k_v-1].x , qp[k_v].x , qp[k_v-1].y , qp[k_v].y, odom_x , odom_y);
        }
        else if (dist_to_path_past_segment<=dist_to_path_current_segment && dist_to_path_past_segment<=dist_to_path_pre_past_segment)
        {
            get_line_equation(qp[k_v-2].x , qp[k_v-1].x , qp[k_v-2].y , qp[k_v-1].y, odom_x , odom_y);
        }
        else if (dist_to_path_pre_past_segment<=dist_to_path_current_segment && dist_to_path_pre_past_segment<=dist_to_path_past_segment)
        {
            get_line_equation(init_x , qp[k_v-2].x , init_y , qp[k_v-2].y, odom_x , odom_y);
        }
    
    }
    else if (k_v>2)
    {
        if (dist_to_path_current_segment<=dist_to_path_past_segment && dist_to_path_current_segment<=dist_to_path_pre_past_segment)
        {
            get_line_equation(qp[k_v-1].x , qp[k_v].x , qp[k_v-1].y , qp[k_v].y, odom_x , odom_y);
        }
        else if (dist_to_path_past_segment<=dist_to_path_current_segment && dist_to_path_past_segment<=dist_to_path_pre_past_segment)
        {
            get_line_equation(qp[k_v-2].x , qp[k_v-1].x , qp[k_v-2].y , qp[k_v-1].y, odom_x , odom_y);
        }
        else if (dist_to_path_pre_past_segment<=dist_to_path_current_segment && dist_to_path_pre_past_segment<=dist_to_path_past_segment)
        {
            get_line_equation(qp[k_v-3].x , qp[k_v-2].x , qp[k_v-3].y , qp[k_v-2].y, odom_x , odom_y);
        }
    
    }
}
void write_result_to_file()
{
    if(!written_to_file)
    {
        outFile << qp[size_path-1].x<<" , "<<qp[size_path-1].y<< " , "<< odom_x <<" , "<< odom_y<< " , "<<average_distance<<" , "<<max_dist<<" , "<<taken_time<< " , "<<j_cost/counts <<endl;
        
        counts = 0;
        j_cost = 0;
        written_to_file = true;
    } 
}

void cal_time_and_cost()
{
    taken_time = taken_time + dt;
    counts++;
    j_cost = j_cost + pow(v_cmd,2)+pow(w_cmd,2);
}

void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_topic1",   path_topic1);
    n.getParam("path_topic2",   path_topic2);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("odometry_topic" ,  odometry_topic);
    n.getParam("status_topic" , status_topic);
    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
    n.getParam("correct_orient_at_last_point" , correct_orient_at_last_point);
}

void define_sub_pub(ros::NodeHandle n)
{
    // Define the publishers and sunscribers
    path_sub_1          = n.subscribe                              (path_topic1,10 , path_back1);
    path_sub_2          = n.subscribe                              (path_topic2,10 , path_back2);
    odom_sub            = n.subscribe                              (odometry_topic, 5, odom_back);
    cmd_pub             = n.advertise<geometry_msgs::Twist>        (cmd_topic, 1);
    robot_mk_pub        = n.advertise<visualization_msgs::Marker>  ("marker_real", 1);
    dist_mk_pub         = n.advertise<visualization_msgs::Marker>  ("marker_dist", 1);
    desired_mk_pub      = n.advertise<visualization_msgs::Marker>  ("marker_desired_point", 1);
    status_control_pub  = n.advertise<std_msgs::String>            (status_topic, 10);
}

void initialize_markers()
{
    dist_mk.header.frame_id = global_frame;
    dist_mk.header.stamp = ros::Time();
    dist_mk.ns = "points_and_lines";
    dist_mk.action = visualization_msgs::Marker::ADD;
    dist_mk.pose.orientation.w = 1.0;
    dist_mk.type = 5;
    dist_mk.scale.x = 0.2;
    dist_mk.scale.y = 0.2;
    dist_mk.color.a = 1.0;
    dist_mk.color.r = 1.0;
    dist_mk.color.g = 0.0;
    dist_mk.color.b = 0.0;
    dist_mk.points.clear();

    desired_mk.header.frame_id = global_frame;
    desired_mk.header.stamp = ros::Time();
    desired_mk.ns = "points_and_lines";
    desired_mk.action = visualization_msgs::Marker::ADD;
    desired_mk.pose.orientation.w = 1.0;
    desired_mk.type = 8;
    desired_mk.scale.x = 0.1;
    desired_mk.scale.y = 0.1;
    desired_mk.color.a = 1.0;
    desired_mk.color.r = 0.0;
    desired_mk.color.g = 0.0;
    desired_mk.color.b = 1.0;
    desired_mk.points.clear();

    robot_mk.header.frame_id = global_frame;
    robot_mk.header.stamp = ros::Time();
    robot_mk.ns = "points_and_lines";
    robot_mk.action = visualization_msgs::Marker::ADD;
    robot_mk.pose.orientation.w = 1.0;
    robot_mk.type = 4;
    robot_mk.scale.x = 0.2;
    robot_mk.scale.y = 0.2;
    robot_mk.color.a = 1.0;
    robot_mk.color.r = 0.0;
    robot_mk.color.g = 0.0;
    robot_mk.color.b = 1.0;
    robot_mk.points.clear();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
    initialize_parameters(n);

    initialize_markers();

    define_sub_pub(n);


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf::StampedTransform inverse;

    ros::Rate loop_rate(10); // ros spins 20 frames per second

    outFile.open(("/home/cds-notebook/mu_ws/src/strl_robotics/generator_stored_point/src/dat.txt"));



    while (ros::ok()) 
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {    
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


        if (path_stored) 
        {
            get_path_errors();
            path_following_controller();
            publish_marker_odom();
            cal_average_max(k_v);
            cal_inter_point();
            cal_time_and_cost();
            stop_mode(); 
        }
        else
        {
            tw_msg.linear.x=0;
            tw_msg.angular.z=0;
           // ROS_INFO("PATH COMPLETED. Set a new path!");
            if (correct_orient_at_last_point) 
            {
                rotation_to_lift();
            }
            write_result_to_file();
            reset_parameters();
        }

        // to start smoothly in case emergency stop
        if (abs(v_robot)<0.06 && abs(v_cmd-v_robot)>0.4)
        {
            v_cmd = 0;
            tw_msg.linear.x = v_cmd;
            ind_control = 9;
        }

        cmd_pub.publish(tw_msg); 
      
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

double get_yaw(geometry_msgs::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
