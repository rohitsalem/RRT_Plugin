#include "rrt.h"

using namespace std;

// Constructors
// Constructors when no arguments are passed
RRTNodes::RRTNodes()
  {
    parentNode=NULL;
  }

//Constructor when configuration is given
RRTNodes::RRTNodes(vector<float > config)
  {
    parentNode=NULL;
    _configuration=config; //assigning the value of the configuration to public variable
  }

//Constructor when configuration and parent node is given
RRTNodes::RRTNodes(vector<float > config,RRTNodes* parent)
  {
    parentNode=parent;  //assigning the value of the configuration and parent node to public variables
    _configuration =config;
  }

//Destructor
RRTNodes::~RRTNodes()
  {
    parentNode=NULL;
    _configuration.clear();
  }
