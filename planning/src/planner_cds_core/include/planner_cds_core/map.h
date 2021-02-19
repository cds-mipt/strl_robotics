#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "gl_const.h"
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
class Map
{
    public:
        Map();
        Map(const Map& orig);
        ~Map();

        bool getMap(const char *FileName);

        bool CellIsTraversable (int i, int j) const;
        bool CellOnGrid (int i, int j) const;
        bool CellIsObstacle(int i, int j) const;
        int  getValue(int i, int j) const;

        bool getMap(const nav_msgs::OccupancyGrid::ConstPtr& grid);
        void setStart(int x_st, int y_st);
        void setGoal(int x_gl, int y_gl);
        int     height, width;
        int     start_i, start_j;
        int     goal_i, goal_j;
        double  cellSize;
        int**   Grid;
};

#endif

