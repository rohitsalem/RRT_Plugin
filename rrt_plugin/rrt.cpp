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
NODE::NODE(vector<float > config, NODE* parent)
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
vector<float > NODE::getConfig()
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

// Functions
//Function to calculate Euclidian distance
float  euclidianDistance(vector<float> config1, vector<float> config2)
{
    float  distance;
    int n=config1.size();
    for(int i=0;i<n;i++)
    {
        int x=(config1[i]-config2[i])*(config1[i]-config2[i]); //difference between the squares
        distance+= x;
    }
    return sqrt(distance);
}

// function for NearestNeighbor
NODE* nearestNeighbhor (vector<float> config, NODETREE& tree )
{

    float  temp=1000000;
    float  len=0;
    int index=0;
    int n= tree.NODETREE::getNodes().size(); // stores the length of the vector

    for(int i =0; i<n;i++)
    {
        len=euclidianDistance(config,tree.NODETREE::getNodes()[i]->getConfig());
        if (len<temp)
        {
            temp=len;
            index=i;
        }
    }
    return tree.NODETREE::getNodes()[index];
}

vector<float > vectorAdd(vector<float > v1,vector<float > v2)
{
    vector<float > v3;
    for(int i=0; i<v1.size();i++)
    {
        v3.push_back(v1[i]+v2[i]);
    }
    return v3;
}

bool collision(vector<float> config)
{
    return true;
}

vector<float> Rand()
{

    vector<float> randConfig;

    for (unsigned int j = 0;j<lower.size();j++)
        randConfig.push_back(  ((float)rand()/RAND_MAX) * (upper[j]-lower[j]) +lower[j]);
    return randConfig;

}

void RRTconnect(NODETREE& t, NODE* nearest,vector<float > config )
{
    vector<float > unitvector;
    NODE* step;
    step=nearest;
    float  Dist;
    Dist = euclidianDistance(config,nearest->getConfig());
    for(int i=0; i<config.size();i++)
    {
        unitvector.push_back(0.75*((config[i]-nearest->getConfig()[i])/Dist));
    }

    do
    {
        if(collision(vectorAdd(step->getConfig(),unitvector))==true)
        {
            step=new NODE(vectorAdd(step->getConfig(),unitvector),step);
            t.NODETREE::addNode(step);
        }
    }while(euclidianDistance(config,step->getConfig())>=0.25);

}


std::vector<NODE*> RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<float> initial, vector<float> goal, float goalBias)
{

    vector<RobotBasePtr> bodies;
    env->GetRobots(bodies);
    RobotBasePtr robot;
    robot=bodies[0];

    robot->GetActiveDOFValues(initial);

    robot->GetActiveDOFLimits(lower,upper);

    NODE* start;
    vector<float > qrand;
    start=new NODE (initial);
    NODETREE t;
    t.NODETREE::addNode(start);
    NODE* Nearest;


    do{
        if((float) rand()/(RAND_MAX)> goalBias)
        {
            qrand=Rand();
        }
        else qrand=goal;
        Nearest=nearestNeighbhor (qrand,t);
        RRTconnect(t,Nearest,qrand);

    }while(euclidianDistance(t.NODETREE::getNodes()[t.NODETREE::getNodes().size()-1]->getConfig(),goal)>=0.25);

}
