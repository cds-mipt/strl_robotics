//
// Created by vlad on 16.02.2021.
//


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <chrono>
#include "map.h"

class ObstInf{
private:

    std::string rawGridTopic;
    ros::Subscriber rawGridSub;
    std::string gridTopic;
    ros::Publisher gridPub;




    Map map = Map(600, 600);
    nav_msgs::OccupancyGrid grid;

    int occupancy_threshold = 40;
    double robot_size;
public:
    ObstInf();
    void inflate(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
    void inflate_comm(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
};

ObstInf::ObstInf() {
    ros::NodeHandle nh;

    nh.param<std::string>("/node_params/grid_topic", gridTopic, "some_grid");
    nh.param<std::string>("/node_params/raw_grid_topic", rawGridTopic, "some_raw_grid");

    nh.param<double>("/node_params/robot_size", robot_size, 1.0);

    rawGridSub   =      nh.subscribe<nav_msgs::OccupancyGrid>       (rawGridTopic,
                                                                    50,
                                                                    &ObstInf::inflate,
                                                                    this);

    gridPub      =      nh.advertise<nav_msgs::OccupancyGrid>       (gridTopic,
                                                                    50);
}

void ObstInf::inflate_comm(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
    this->grid = *gridMsg;

//    if ((map.getWidth() != gridMsg->info.width) || (map.getHeight() != gridMsg->info.height)){
//        map.setWidth(gridMsg->info.width);
//        map.setHeight(gridMsg->info.height);
//    }

    map.setWidth(gridMsg->info.width);
    map.setHeight(gridMsg->info.height);
    for (int i = 0; i < gridMsg->data.size(); i++){
        ROS_INFO_STREAM(int(i/gridMsg->info.height));
        map.setCell(int(i%gridMsg->info.width),int(i/gridMsg->info.width), gridMsg->data[i]==100);
    }

//std::chrono::time_point<std::chrono::system_clock> start, finish;
//start = std::chrono::system_clock::now();
    map.computeDistances();
//finish = std::chrono::system_clock::now();
//double time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count()) / 1000000000;
//std::cout << "Distance time: " << time << std::endl;
    ROS_INFO_STREAM(grid.data.size());
    ROS_INFO_STREAM(map.getHeight());
    ROS_INFO_STREAM(map.getWidth());
    double robotCellSize = robot_size/grid.info.resolution;
    for (int y = 0 ; y < map.getHeight(); ++y)
    {
        for (int x = 0 ; x < map.getWidth(); ++x)
        {
//            ROS_INFO_STREAM(map.getDistance(x, y));
            if (map.getDistance(x, y) < robotCellSize && map.getDistance(x, y) > 0){
//                ROS_INFO_STREAM(map.getDistance(x, y));
                grid.data[y*grid.info.width + x] = 50;
            }

        }
    }
    ROS_INFO_STREAM("I'm in!");
    gridPub.publish(grid);
}


void ObstInf::inflate(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
    this->grid = *gridMsg;
    ROS_INFO_STREAM("Received grid. Inflating");
    map.setWidth(gridMsg->info.width);
    map.setHeight(gridMsg->info.height);
    for (int i = 0; i < gridMsg->data.size(); i++) {
        map.setCell(int(i % gridMsg->info.width), int(i / gridMsg->info.width), gridMsg->data[i] > 50);
    }

    map.computeDistances();

    double robotCellSize = robot_size / grid.info.resolution;
    for (int y = 0; y < map.getHeight(); ++y) {
        for (int x = 0; x < map.getWidth(); ++x) {
            if (map.getDistance(x, y) < robotCellSize && map.getDistance(x, y) > 0) {
                grid.data[y * grid.info.width + x] = 50;
            }

        }
    }

    gridPub.publish(grid);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "inflator");


    ObstInf Inflator;
    ros::Rate r(50);

    
    while(ros::ok()){
        ros::spinOnce();
        ROS_WARN_STREAM("No grid received. Waiting");
        r.sleep();
    }
}


