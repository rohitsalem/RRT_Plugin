#include "rrt.h"

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
NODE* NODE::getParent()
{
    return parentNode;
}

//returns the configuration
vector<float > NODE::getConfig()
{
    return _configuration;
}

//function to add a node
void NODETREE::addNode(NODE* Node)
{
    _nodes.push_back(Node);
}

//function to return the vector of nodes
vector<NODE*> NODETREE::getNodes()
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
    for(unsigned int i=0; i<v1.size();i++)
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

NODETREE::NODETREE(NODE *init)
{
    start=init;
}

void RRTconnect(OpenRAVE::EnvironmentBasePtr env, NODETREE& t, NODE* nearest,vector<float > config )
{

    robots = std::vector<OpenRAVE::RobotBasePtr> ();
    env->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot;
    robot=robots.at(0);

    vector<float > unitvector;
    vector<float> v;
    NODE* step;
    step=nearest;
    float  Dist;
    Dist = euclidianDistance(config,nearest->getConfig());
    for(unsigned int i=0; i<config.size();i++)
    {
        unitvector.push_back((config[i]-nearest->getConfig()[i])/Dist);
    }

    do
    {   v= vectorAdd(step->getConfig(),unitvector);

        step=new NODE(v,step);
        t.NODETREE::addNode(step);

    }while((euclidianDistance(config,step->getConfig())>=0.2) && (env->CheckCollision(robot)||robot->CheckSelfCollision())!=true );
    NODE* qrand;
    qrand =new NODE(config,step);
    t.NODETREE::addNode(qrand);
}


std::vector<NODE*> RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<float> initial, vector<float> goal, float goalBias)
{

//    robots = std::vector<OpenRAVE::RobotBasePtr> ();
//    env->GetRobots(robots);
//    OpenRAVE::RobotBasePtr robot;
//    robot=robots.at(0);


    //robot->GetActiveDOFValues(initial);

    robot->GetActiveDOFLimits(Lower,Upper);

    vector<float> lower=float(Lower);
     vector<float> upper=float(Upper);

    vector<float > qrand;
    NODE* start;
    NODETREE t(start);
    NODE* Nearest;

    start=new NODE (initial);
    t.NODETREE::addNode(start);

    do{
        if((float) rand()/(RAND_MAX)> goalBias)
        {
            qrand=Rand();
        }

        else qrand=goal;
        Nearest=nearestNeighbhor (qrand,t);
        RRTconnect(env,t,Nearest,qrand);
    }while(euclidianDistance(nearestNeighbhor(goal,t)->getConfig(),goal)>=0.25);
    // }while(euclidianDistance(t.NODETREE::getNodes()[t.NODETREE::getNodes().size()-1]->getConfig(),goal)>=0.25);
    std::vector<NODE*> path;
    NODE* goalNode;
    goalNode =new NODE(goal,nearestNeighbhor(goal,t));
    while(goalNode->getParent()!=NULL)
    {
       path.push_back(goalNode);
       goalNode=goalNode->getParent();
    }
    return path ; // should change this
}
