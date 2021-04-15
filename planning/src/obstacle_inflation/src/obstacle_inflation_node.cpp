//
// Created by vlad on 16.02.2021.
//


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <chrono>
#include "map.h"


double yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class ObstInf{
private:

    std::string rawGridTopic;
    ros::Subscriber rawGridSub;
    std::string gridTopic;
    ros::Publisher gridPub;
    std::string costmapTopic;
    ros::Publisher costmapPub;

    tf2_ros::Buffer&                tfBuffer;


    Map map = Map(600, 600);
    nav_msgs::OccupancyGrid grid;
    nav_msgs::OccupancyGrid costmap;

    int gridSize;

    double robot_size;
    double safe_size;

    std::string globalFrame;
    std::string baseFrame;

public:
    ObstInf(tf2_ros::Buffer& _tfBuffer);

    int log_level;

    void inflate(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
    void publish();
};

ObstInf::ObstInf(tf2_ros::Buffer& _tfBuffer): tfBuffer(_tfBuffer) {
    ros::NodeHandle nh;

    nh.param<std::string>("/inflation_params/grid_topic", gridTopic, "some_grid");
    nh.param<std::string>("/inflation_params/raw_grid_topic", rawGridTopic, "some_raw_grid");
    nh.param<std::string>("/inflation_params/costmap_topic", costmapTopic, "some_costmap");


    nh.param<std::string>("/inflation_params/global_frame", globalFrame, "map");
    nh.param<std::string>("/inflation_params/base_frame", baseFrame, "base_link");

    nh.param<double>("/inflation_params/robot_size", robot_size, 1.0);
    nh.param<double>("/inflation_params/safe_size", safe_size, 1.0);
    nh.param<int>("/inflation_params/log_level", log_level, 1);
    nh.param<int>("/inflation_params/grid_size", gridSize, 500);


    rawGridSub   =      nh.subscribe<nav_msgs::OccupancyGrid>       (rawGridTopic,
                                                                    50,
                                                                    &ObstInf::inflate,
                                                                    this);

    gridPub      =      nh.advertise<nav_msgs::OccupancyGrid>       (gridTopic,
                                                                    50);
    
    costmapPub   =      nh.advertise<nav_msgs::OccupancyGrid>       (costmapTopic,
                                                                    50);
}



void ObstInf::inflate(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {

    int minSize = int(std::min(gridMsg->info.width, gridMsg->info.height));

    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::TransformStamped transform;
    try {
        transform = tfBuffer.lookupTransform(gridMsg->header.frame_id, baseFrame, ros::Time(0));
    }catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
    robot_pose.header.frame_id = gridMsg->header.frame_id;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;


    robot_pose.pose.position.x -= gridMsg->info.origin.position.x;
    robot_pose.pose.position.y -= gridMsg->info.origin.position.y;
    robot_pose.pose.position.x = int(robot_pose.pose.position.x / gridMsg->info.resolution);
    robot_pose.pose.position.y = int(robot_pose.pose.position.y / gridMsg->info.resolution);

    this->grid = *gridMsg;
    this->costmap = grid;

    map.setWidth(minSize);
    map.setHeight(minSize);

    if(log_level){
        ROS_INFO_STREAM("Received grid. Inflating");
        ROS_INFO_STREAM("Width: " << gridMsg->info.width << "\t Height: " << gridMsg->info.height);
        ROS_INFO_STREAM("X: " << robot_pose.pose.position.x << "\t Y: " << robot_pose.pose.position.y);
        ROS_INFO_STREAM("X start: " << std::max(int(robot_pose.pose.position.x - minSize/2), 0) << "\t X end: " << std::min(int(robot_pose.pose.position.x + minSize/2), int(grid.info.width)));
        ROS_INFO_STREAM("Y start: " << std::max(int(robot_pose.pose.position.y - minSize/2), 0) << "\t Y end: " << std::min(int(robot_pose.pose.position.y + minSize/2), int(grid.info.height)));
    }

    int _x = 0, _y = 0;
    for (int y = std::max(int(robot_pose.pose.position.y - minSize/2), 0); y < std::min(int(robot_pose.pose.position.y + minSize/2), int(grid.info.height)); ++y){
        for (int x = std::max(int(robot_pose.pose.position.x - minSize/2), 0); x < std::min(int(robot_pose.pose.position.x + minSize/2), int(grid.info.width)); ++x){
           map.setCell(_x, _y, gridMsg->data[y * grid.info.width + x] > 0);
           ++_x;
        }
        _x = 0;
        ++_y;
    }


    map.computeDistances();

    double robotCellSize = robot_size / grid.info.resolution;
    if(log_level) ROS_INFO_STREAM("Robot size in cells:" << robotCellSize);
    double safeCellSize = safe_size / grid.info.resolution;
    //!Here we create gird for planner to plan on

    _x = 0; _y = 0;
    for (int y = std::max(int(robot_pose.pose.position.y - minSize/2), 0); y < std::min(int(robot_pose.pose.position.y + minSize/2), int(grid.info.height)); ++y){
        for (int x = std::max(int(robot_pose.pose.position.x - minSize/2), 0); x < std::min(int(robot_pose.pose.position.x + minSize/2), int(grid.info.width)); ++x){
            if (map.getDistance(_x, _y) < robotCellSize) grid.data[y * grid.info.width + x] = 100;
            ++_x;
        }
        _x = 0;
        ++_y;
    }



    _x = 0; _y = 0;
    for (int y = std::max(int(robot_pose.pose.position.y - minSize/2), 0); y < std::min(int(robot_pose.pose.position.y + minSize/2), int(grid.info.height)); ++y){
        for (int x = std::max(int(robot_pose.pose.position.x - minSize/2), 0); x < std::min(int(robot_pose.pose.position.x + minSize/2), int(grid.info.width)); ++x){
            if (map.getDistance(_x, _y) < robotCellSize && map.getDistance(_x, _y) > 0) {
                costmap.data[y * grid.info.width + x] = int(100 - map.getDistance(_x, _y) * 20 / robotCellSize);
            }else if (map.getDistance(_x, _y) < safeCellSize && map.getDistance(_x, _y) > robotCellSize){
                costmap.data[y * grid.info.width + x] = int(80 - (map.getDistance(_x, _y) - robotCellSize) * 30 / (robotCellSize - safeCellSize));

            }else if(map.getDistance(_x, _y) > safeCellSize){
                costmap.data[y * grid.info.width + x] = int(50*exp(-(map.getDistance(_x, _y) - safeCellSize)));
            }
            ++_x;
        }
        _x = 0;
        ++_y;
    }

    publish();
}

void ObstInf::publish(){
    gridPub.publish(grid);
    costmapPub.publish(costmap);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "inflator");

    tf2_ros::Buffer tfBuffer(ros::Duration(5)); //todo: remake with pointers
    tf2_ros::TransformListener tfListener(tfBuffer);

    ObstInf Inflator(tfBuffer);
    ros::Rate r(50);
    
    while(ros::ok()){
        ros::spinOnce();
        if(Inflator.log_level) ROS_WARN_STREAM("No grid received. Waiting");
        r.sleep();
    }
}


