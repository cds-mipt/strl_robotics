#include "mission.h"
#include "astar.h"
#include "bfs.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"
#include "gl_const.h"

Mission::Mission()
{
    logger = nullptr;
    search = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
    search = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
    if (search)
        delete search;
}

bool Mission::getMap()
{
    return map.getMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != NULL) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS || config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC]);
    else
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch()
{
    if (search)
        delete search;
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        search = new BFS();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        search = new Dijkstra();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        search = new JP_Search(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        search = new Theta(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
}

void Mission::startSearch()
{
    sr = search->startSearch(logger, map, options);
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.cellSize);
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        return CNS_SP_ST_DIJK;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        return CNS_SP_ST_BFS;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        return CNS_SP_ST_JP_SEARCH;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}

bool Mission::getMap(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    return map.getMap(grid);
}

void Mission::createEnvironmentOptions(bool AS, bool AD, bool CC) {
    options = EnvironmentOptions(AS,AD,CC);
}

void Mission::createSearch(std::string searchType)
{   //todo: parameterize
    if (search)
        delete search;
    if (searchType == "bfs")
        search = new BFS();
    else if (searchType  == "dijkstra")
        search = new Dijkstra();
    else if (searchType  == "astar")
        search = new Astar(1, 1);
    else if (searchType  == "jp_search")
        search = new JP_Search(1, 1);
    else if (searchType  == "theta")
        search = new Theta(1, 1);
}

void Mission::setStart(int x_st, int y_st){
    map.setStart(x_st, y_st);
}
void Mission::setGoal(int x_gl, int y_gl){
    map.setGoal(x_gl, y_gl);
}

std::list<Node> Mission::getLPPath(){
    return *sr.lppath;
}