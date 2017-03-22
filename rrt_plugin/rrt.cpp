#include "rrt.h"

// Constructors
// Constructors when no arguments are passed
NODE::NODE()
{
    parentNode=NULL;
}

//Constructor when configuration is given
NODE::NODE(vector<double > config)
{
    parentNode=NULL;
    _configuration=config; //assigning the value of the configuration to public variable
}

//Constructor when configuration and parent node is given
NODE::NODE(vector<double > config, NODE* parent)
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
vector<double > NODE::getConfig()
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
double  RRT::euclidianDistance(vector<double> config1, vector<double> config2)
{
    double  distance;
    int n=config1.size();
    for(int i=0;i<n;i++)
    {
        int x=(config1[i]-config2[i])*(config1[i]-config2[i]); //difference between the squares
        distance+= x;
    }
    return sqrt(distance);
}

// function for NearestNeighbor
NODE* RRT::nearestNeighbhor (vector<double> config, NODETREE& tree )
{

    double  temp=1000000;
    double  len=0;
    int index=0;
    int n= tree.NODETREE::getNodes().size(); // stores the length of the vector

    for(int i =0; i<n;i++)
    {
        len=RRT::euclidianDistance(config,tree.NODETREE::getNodes()[i]->getConfig());
        if (len<temp)
        {
            temp=len;
            index=i;
        }
    }
    return tree.NODETREE::getNodes()[index];
}

vector<double > RRT::vectorAdd(vector<double > v1,vector<double > v2)
{
    vector<double > v3;
    for(unsigned int i=0; i<v1.size();i++)
    {
        v3.push_back(v1[i]+v2[i]);
    }
    return v3;
}

vector<double> RRT::Rand()
{

    vector<double> randConfig;
    for (unsigned int j = 0;j<lower.size();j++)
        randConfig.push_back(  ((double)rand()/RAND_MAX) * (upper[j]-lower[j]) +lower[j]);
    return randConfig;

}

NODETREE::NODETREE(NODE *init)
{
    startNode=init;
    _nodes.push_back(startNode);
}

void RRT::RRTconnect(OpenRAVE::EnvironmentBasePtr env, NODETREE& t, NODE* nearest,vector<double > config )
{

    //    robots = std::vector<OpenRAVE::RobotBasePtr> ();
    //    env->GetRobots(robots);
    //    OpenRAVE::RobotBasePtr robot;
    //    robot=robots.at(0);

    vector<double> unitvector;
    vector<double> v;
    NODE* step;
    step=nearest;
    double  Dist;
    Dist = RRT::euclidianDistance(config,nearest->getConfig());
    for(unsigned int i=0; i<config.size();i++)
    {
        unitvector.push_back((config[i]-nearest->getConfig()[i])/Dist);
    }

    do
    {   v= RRT::vectorAdd(step->getConfig(),unitvector);

        step=new NODE(v,step);
        t.NODETREE::addNode(step);

    }while((RRT::euclidianDistance(config,step->getConfig())>=0.2) && (env->CheckCollision(robot)||robot->CheckSelfCollision())!=true );
    NODE* qrand;
    qrand =new NODE(config,step);
    t.NODETREE::addNode(qrand);
}


std::vector<NODE*> RRT::RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<double> initial, vector<double> goal, double goalBias)
{

//    robots = std::vector<OpenRAVE::RobotBasePtr> ();
//    env->GetRobots(robots);
//    OpenRAVE::RobotBasePtr robot;
//    robot=robots.at(0);

//    robot->GetActiveDOFValues(initial);

    robot->GetActiveDOFLimits(Lower,Upper);

    vector<double> lower =vector<double>(Lower);
    vector<double> upper =vector<double>(Upper);

    vector<double > qrand;
    NODE* Nearest;
    NODE* start=new NODE (initial);
    NODETREE t(start);
    //t.addNode(start);

    do{
        if((double) rand()/(RAND_MAX)> goalBias)
        {
            qrand=RRT::Rand();
        }

        else qrand=goal;
        Nearest=RRT::nearestNeighbhor (qrand,t);
        RRT::RRTconnect(env,t,Nearest,qrand);
    }while(RRT::euclidianDistance(RRT::nearestNeighbhor(goal,t)->getConfig(),goal)>=0.25);

    // }while(RRT::euclidianDistance(t.NODETREE::getNodes()[t.NODETREE::getNodes().size()-1]->getConfig(),goal)>=0.25);

    std::vector<NODE*> path;
    NODE* goalNode;
    goalNode =new NODE(goal,RRT::nearestNeighbhor(goal,t));
    while(goalNode->getParent()!=NULL)
    {
        path.push_back(goalNode);
        goalNode=goalNode->getParent();
    }
    return path ;
}
