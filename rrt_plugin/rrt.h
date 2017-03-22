#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/openrave.h>
#include <math.h>
#include <boost/bind.hpp>


using namespace OpenRAVE;
using namespace std;

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
    NODETREE(NODE* init); //Empty constructor
    ~NODETREE();

    //Functions
    void addNode(NODE* node); //adds a node
    vector<NODE*> getNodes(); //returns the vector of nodes

};



vector<double> Rand();
vector<double > vectorAdd(vector<double > v1,vector<double > v2);
double  euclidianDistance(vector<double> config1, vector<double> config2);
NODE* nearestNeighbhor (vector<double> config, NODETREE& tree );
void RRTconnect(NODETREE& t, NODE* nearest,vector<double > config );
std::vector<NODE*> RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<double> initial, vector<double > goal, double goalBias);
std::vector<OpenRAVE::RobotBasePtr> robots;
OpenRAVE::RobotBasePtr robot;
std::vector<dReal> Lower, Upper;
std::vector<double> lower, upper;
