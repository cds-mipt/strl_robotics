#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "isearch.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "jp_search.h"
#include "astar.h"
#include "bfs.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"
#include <nav_msgs/OccupancyGrid.h>

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool createLog();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

        bool getMap(const nav_msgs::OccupancyGrid::ConstPtr& grid);
        void createEnvironmentOptions(bool AS, bool AD, bool CC);
        void createSearch(std::string searchType);
        void setStart(int x_st, int y_st);
        void setGoal(int x_gl, int y_gl);
        std::list<Node> getLPPath();
    private:
        const char* getAlgorithmName();

        Map                     map;
        Config                  config;
        EnvironmentOptions      options;
        ISearch*                search;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

