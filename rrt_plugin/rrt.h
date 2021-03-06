#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/openrave.h>
#include <math.h>
#include <boost/bind.hpp>
#include <openrave/planningutils.h>
#include <chrono>
using namespace OpenRAVE;
using namespace std;
using  s = chrono::seconds;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock ;

class NODE{

public:

    std::vector<double> _configuration;
    NODE* parentNode;

    //constructors
    NODE(); // Empty CONSTRUCTOR
    NODE(std::vector<double> config); //CONSTRUCTOR - configuration provided
    NODE(std::vector<double> config, NODE* parent ); //CONSTRUCTOR - configuration, parent node provided
    ~NODE(); // DESTRUCTOR

    //Functions
    std::vector<double> getConfig(); //returns the configuration of the current node
    NODE* getParent();
};


class NODETREE{

public:

    std::vector<NODE*> _nodes;
    NODE* startNode;
    //constructors
    NODETREE();
    NODETREE(NODE* init); //Empty constructor
    ~NODETREE();

    //Functions
    void addNode(NODE* node); //adds a node
    vector<NODE*> getNodes(); //returns the vector of nodes
    NODE* lastNode(); //get the last node

};


class RRT{

public:

    std::vector<OpenRAVE::RobotBasePtr> robots;
    OpenRAVE::RobotBasePtr robot;
    std::vector<double> lower, upper;
    std::vector<double> initial;
    vector<GraphHandlePtr> handles;
    int i,j,min,max;

    RRT();
    ~RRT();

    vector<double> randConfiguration();
    vector<double > vectorAdd(vector<double > v1,vector<double > v2);
    double  euclidianDistance(vector<double> config1, vector<double> config2);
    double weightedDistance(vector<double> config1, vector<double> config2);
    NODE* nearestNeighbhor (vector<double> config, NODETREE& tree );
    void RRTconnect(OpenRAVE::EnvironmentBasePtr env, NODETREE& t, NODE* nearest,vector<double > config );
    std::vector<NODE*> RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<double > goal, double goalBias);
    std::vector<NODE*> shortCutSmooth(OpenRAVE::EnvironmentBasePtr env,vector<NODE*> path, int iterations,vector<double> goalConfig, double goalBias);
    bool isNotInlimits(vector<double> config);
    std::vector<NODE*> BiRRTPlanner(OpenRAVE::EnvironmentBasePtr env,vector<double> startConfig,vector<double> goalConfig);
   // std::vector<NODE*> shortCutSmooth(vector<Node*>path);
};




