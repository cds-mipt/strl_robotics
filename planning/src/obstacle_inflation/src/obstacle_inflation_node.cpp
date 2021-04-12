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
    std::string costmapTopic;
    ros::Publisher costmapPub;




    Map map = Map(600, 600);
    nav_msgs::OccupancyGrid grid;
    nav_msgs::OccupancyGrid costmap;

    int occupancy_threshold = 40;
    double robot_size;
    double safe_size;

public:
    ObstInf();

    int log_level;

    void inflate(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
    void publish();
};

ObstInf::ObstInf() {
    ros::NodeHandle nh;

    nh.param<std::string>("/inflation_params/grid_topic", gridTopic, "some_grid");
    nh.param<std::string>("/inflation_params/raw_grid_topic", rawGridTopic, "some_raw_grid");
    nh.param<std::string>("/inflation_params/costmap_topic", costmapTopic, "some_costmap");

    nh.param<double>("/inflation_params/robot_size", robot_size, 1.0);
    nh.param<double>("/inflation_params/safe_size", safe_size, 1.0);
    nh.param<int>("/inflation_params/log_level", log_level, 1);

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
    this->grid = *gridMsg;
    this->costmap = grid;

    if(log_level) ROS_INFO_STREAM("Received grid. Inflating");
    map.setWidth(gridMsg->info.width);
    map.setHeight(gridMsg->info.height);
    std::cout << "Width: " << gridMsg->info.width << "\t Height: " << gridMsg->info.height << "\n";
    for (int i = 0; i < gridMsg->data.size(); i++) {
        map.setCell(int(i % gridMsg->info.width), int(i / gridMsg->info.width), gridMsg->data[i] > 0);
    }

    map.computeDistances();

    double robotCellSize = robot_size / grid.info.resolution;
    double safeCellSize = safe_size / grid.info.resolution;
    //!Here we create gird for planner to plan on
    for (int y = 0; y < map.getHeight(); ++y) {
        for (int x = 0; x < map.getWidth(); ++x) {
            if (map.getDistance(x, y) < robotCellSize && map.getDistance(x, y) > 0) {
                grid.data[y * map.getWidth() + x] = 100;
            }
        }
    }
    //!Here we create costmap for visualization and debug
    for (int y = 0; y < map.getHeight(); ++y) {
        for (int x = 0; x < map.getWidth(); ++x) {
            if (map.getDistance(x, y) < robotCellSize && map.getDistance(x, y) > 0) {
                costmap.data[y * grid.info.width + x] = char(100 - map.getDistance(x, y) * 20 / robotCellSize);

            }else if (map.getDistance(x, y) < safeCellSize && map.getDistance(x, y) > robotCellSize){
                costmap.data[y * grid.info.width + x] = char(80 - (map.getDistance(x, y) - robotCellSize) * 30 / (robotCellSize - safeCellSize));

            }else if(map.getDistance(x, y) > safeCellSize){
                costmap.data[y * grid.info.width + x] = char(50*exp(-(map.getDistance(x, y) - safeCellSize)));
            }
        }
    }

    publish();
}

void ObstInf::publish(){
    gridPub.publish(grid);
    costmapPub.publish(costmap);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "inflator");

    ObstInf Inflator;
    ros::Rate r(50);
    
    while(ros::ok()){
        ros::spinOnce();
        if(Inflator.log_level) ROS_WARN_STREAM("No grid received. Waiting");
        r.sleep();
    }
}


