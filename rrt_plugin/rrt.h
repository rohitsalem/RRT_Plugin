#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/plugin.h>
#include <math.h>

using namespace OpenRAVE;
using namespace std;

class NODE{

public:

  std::vector<double> _configuration;
  NODE* parentNode;

  //constructors
  NODE(); // Empty CONSTRUCTOR
  NODE(std::vector<float> config); //CONSTRUCTOR - configuration provided
  NODE(std::vector<float> config, NODE* parent ); //CONSTRUCTOR - configuration, parent node provided
  ~NODE(); // DESTRUCTOR

  //Functions
  std::vector<float> getConfig(); //returns the configuration of the current node
};


class NODETREE{

public:

  std::vector<NODE*> _nodes;

  //constructors
  NODETREE(); //Empty constructor
  ~NODETREE();

  //Functions
  void addNode(NODE* node); //adds a node
  vector<NODE*> getNodes(); //returns the vector of nodes

};


bool collision(vector<float > config);
vector<double> Rand();
vector<float > vectorAdd(vector<float > v1,vector<float > v2);
float  euclidianDistance(vector<float> config1, vector<float> config2);
NODE* nearestNeighbhor (vector<float> config, NODETREE& tree );
void RRTconnect(NODETREE& t, NODE* nearest,vector<float > config );
std::vector<NODE*> RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<float> initial, vector<float > goal, float goalBias);

