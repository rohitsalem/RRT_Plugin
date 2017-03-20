#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/plugin.h>

using namespace OpenRAVE;
using namespace std;

class NODE{

public:
  std::vector<double> _configuration;
  NODE* parentNode;

  //constructors
  NODE(); //CONSTRUCTOR
  NODE(std::vector<float> config); //CONSTRUCTOR - configuration provided
  NODE(std::vector<float> config, NODE* parent ) //CONSTRUCTOR - configuration, parent node provided
  ~NODE(); // DESTRUCTOR

  //Functions
  std::vector<float> getConfig(); //returns the configuration of the current node
};
