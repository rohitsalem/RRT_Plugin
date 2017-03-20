#include "rrt.h"

using namespace std;

// Constructors
// Constructors when no arguments are passed
NODE::NODE()
  {
    parentNode=NULL;
  }

//Constructor when configuration is given
NODE::NODE(vector<float > config)
  {
    parentNode=NULL;
    _configuration=config; //assigning the value of the configuration to public variable
  }

//Constructor when configuration and parent node is given
NODE::NODE(vector<float > config,NODE* parent)
  {
    parentNode=parent;  //assigning the value of the configuration and parent node to public variables
    _configuration =config;
  }

//Destructor
NODE::~NODE()
  {
    parentNode=NULL;
    _configuration.clear();
  }

//Functions
//returns the parent node
NODE* NODE::getparent()
  {
    return parentNode;
  }

//returns the configuration
vector<float > NODE::getconfig()
  {
  return _configuration;
  }

//function to add a node
void NODETREE::addNode(NODETREE* Node)
  {
    _nodes.push_back(Node);
  }

//function to return the vector of nodes
vector<NODETREE*> NODETREE::getNodes()
  {
    return _nodes;
  }
