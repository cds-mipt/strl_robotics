//
// Created by vlad on 16.02.2021.
//


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <distance_map_msgs/DistanceMap.h>

#include <fstream>
#include <chrono>
#include <exception>


class ObstInf{
private:
    //!The idea is as follows:
    //!We receive raw grid from some map creator
    std::string rawGridTopic;
    ros::Subscriber rawGridSub;
    //!Then we send it to a distance map creator
    std::string distanceMapTopicIn;
    ros::Publisher distanceMapPub;
    //!Next we receive distance map as it is
    std::string distanceMapTopicOut;
    ros::Subscriber distanceMapSub;
    //!And we modify this distance map to be either inflated grid
    std::string gridTopic;
    ros::Publisher gridPub;
    //!Or regular costmap i.e. not modify at all
    std::string costmapTopic;
    ros::Publisher costmapPub;


    nav_msgs::OccupancyGrid grid;
    nav_msgs::OccupancyGrid costmap;

    int gridSize;

    double robot_size;
    double safe_size;

    bool inflating = false;


public:
    ObstInf();

    int log_level = 1;

    void computeDistances(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
    void inflate(const distance_map_msgs::DistanceMap::ConstPtr& distMapMsg);
    void publish();
};

ObstInf::ObstInf(){
    ros::NodeHandle nh;

    nh.param<std::string>("/inflation_params/grid_topic", gridTopic, "some_grid");
    nh.param<std::string>("/inflation_params/distmap_topic_in", distanceMapTopicIn, "/dist_map_in");
    nh.param<std::string>("/inflation_params/raw_grid_topic", rawGridTopic, "some_raw_grid");
    nh.param<std::string>("/inflation_params/costmap_topic", costmapTopic, "some_costmap");
    nh.param<std::string>("/inflation_params/distmap_topic_out", distanceMapTopicOut, "/distance_map_node/distance_field_obstacles");


    nh.param<double>("/inflation_params/robot_size", robot_size, 1.0);
    nh.param<double>("/inflation_params/safe_size", safe_size, 1.0);
    nh.param<int>("/inflation_params/log_level", log_level, 1);


    rawGridSub      =      nh.subscribe<nav_msgs::OccupancyGrid>            (rawGridTopic,
                                                                            50,
                                                                            &ObstInf::computeDistances,
                                                                            this);

    distanceMapPub  =      nh.advertise<nav_msgs::OccupancyGrid>            (distanceMapTopicIn,
                                                                            50);


    distanceMapSub  =      nh.subscribe<distance_map_msgs::DistanceMap>     (distanceMapTopicOut,
                                                                            50,
                                                                            &ObstInf::inflate,
                                                                            this);

    gridPub         =      nh.advertise<nav_msgs::OccupancyGrid>            (gridTopic,
                                                                            50);
    
    costmapPub      =      nh.advertise<nav_msgs::OccupancyGrid>            (costmapTopic,
                                                                            50);
}



void ObstInf::computeDistances(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
    if(log_level) ROS_INFO_STREAM("Received grid");


    if(!inflating){
        grid.data.clear();
        costmap.data.clear();

        this->grid = *gridMsg;
        this->costmap = *gridMsg;

        if(log_level) ROS_INFO_STREAM("Inflating");

        //!Here we modify our grid to make unknown cells traversable
        auto modifiedGrid = grid;
        for (int y = 0; y < modifiedGrid.info.height; ++y){
            for (int x = 0; x < modifiedGrid.info.width; ++x){
                if (modifiedGrid.data[y * modifiedGrid.info.width + x] == -1) modifiedGrid.data[y * modifiedGrid.info.width + x] = 0;
            }
        }
        inflating = true;
        distanceMapPub.publish(modifiedGrid);
        //! Publish raw grid to distance map package
        //! Receive distance map
    }







}



void ObstInf::inflate(const distance_map_msgs::DistanceMap::ConstPtr& distMapMsg) {
    if(log_level) ROS_INFO_STREAM("Received distance map");

    if(!grid.info.resolution) return;
    double robotCellSize = robot_size / grid.info.resolution;
    double safeCellSize = safe_size / grid.info.resolution;
    //!Here we create gird for planner to plan on

    if(log_level) ROS_INFO_STREAM("Transforming previous grid");

    if(distMapMsg->info.height != grid.info.height || distMapMsg->info.width != grid.info.width) ROS_WARN_STREAM("Heights or widths do not correspond!");

    auto y_max = std::min(grid.info.height, distMapMsg->info.height);
    auto x_max = std::min(grid.info.width, distMapMsg->info.width);
//
//    ROS_INFO_STREAM(grid.info);

    for (int y = 0; y < grid.info.height; ++y){
        for (int x = 0; x < grid.info.width; ++x){
            if (distMapMsg->data[y * grid.info.width + x] < robotCellSize &&
                grid.data[(grid.info.height-y-1) * grid.info.width + x] != -1) grid.data[(grid.info.height-y-1) * grid.info.width + x] = 100;

            if (distMapMsg->data[y * grid.info.width + x] > robotCellSize &&
                distMapMsg->data[y * grid.info.width + x] < safeCellSize &&
                grid.data[(grid.info.height-y-1) * grid.info.width + x] != -1) grid.data[(grid.info.height-y-1) * grid.info.width + x] = 60;

            costmap.data[(grid.info.height-y-1) * grid.info.width + x] = int(distMapMsg->data[y * distMapMsg->info.width + x]);
        }
    }

    inflating = false;

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
//        if(Inflator.log_level) ROS_WARN_STREAM("No grid received. Waiting");
        r.sleep();
    }
}


