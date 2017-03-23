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
        double x=(config1[i]-config2[i])*(config1[i]-config2[i]); //difference between the squares
        distance+= x;
    }
    return sqrt(distance);
}
double RRT::weightedDistance(vector<double> config1, vector<double> config2)
{
    double  distance;
    int n=config1.size();

    vector<double> weights={3.17104,2.75674,2.2325,1.78948, 1.42903, 0.809013,0.593084};

    for(int i=0;i<n;i++)
    {
        double x=(config1[i]-config2[i])*(config1[i]-config2[i])*(weights[i]*weights[i]) ; //difference between the squares
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

    for(int i =0; i<n;++i)
    {
        len=RRT::weightedDistance(config,tree.NODETREE::getNodes()[i]->getConfig());
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

RRT::RRT()
{

}

RRT::~RRT()
{

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

NODETREE::~NODETREE()
{
    _nodes.clear();
}
NODE* NODETREE::lastNode()
{
    return _nodes.back();
}

bool RRT::isNotInlimits(vector<double> config)
{   bool x;
    for(unsigned int i; i<config.size(); ++i)
    {
        if((lower[i]>config[i])||(config[i]>upper[i]))
            x=true;
        else
            x=false;
    }
    return x;
}
void RRT::RRTconnect(OpenRAVE::EnvironmentBasePtr env, NODETREE& t, NODE* nearest,vector<double > config )
{

    vector<double> unitvector;
    vector<double> v;

    bool collision;
    double  Dist,distance;
    Dist = RRT::euclidianDistance(config,nearest->getConfig());

    for(unsigned int i=0; i<config.size();i++)
    {
        unitvector.push_back(0.2*(config[i]-nearest->getConfig()[i])/Dist);
    }
    //cout<<unitvector[0]<<" "<<unitvector[1]<<" "<<unitvector[2]<<" "<<unitvector[3]<<" "<<unitvector[4]<<" "<<unitvector[5]<<" "<<unitvector[6]<<endl;

    distance=Dist;

    while(distance>0.2)
    {
        v= RRT::vectorAdd(nearest->getConfig(),unitvector);
        robot->SetActiveDOFValues(v);
        // cout<<"inside rrt Connect \n" <<endl;

        if(!(env->CheckCollision(robot))&& !(robot->CheckSelfCollision()) && !isNotInlimits(v))
        {
            nearest=new NODE(v,nearest);
            t.NODETREE::addNode(nearest);
            distance=euclidianDistance(v,config);
           // cout<<"Distance between nearest and random config: "<< distance<< endl;
            //   cout << "Added a Step to the tree: " << j << "\n" <<nearest->getConfig()[0]<<" "<<nearest->getConfig()[1]<<" "<<nearest->getConfig()[2]<<" "<< nearest->getConfig()[3]<<" "<< nearest->getConfig()[4]<<" "<< nearest->getConfig()[5]<<" "<< nearest->getConfig()[6]<<endl ;
        }

        else
        {
            collision=true;
            break;
        }
    }

    if(!collision)
    {
        robot->SetActiveDOFValues(config);
        if(!(env->CheckCollision(robot)) && !(robot->CheckSelfCollision()) && !isNotInlimits(config))
        {
            NODE* qrand;
            qrand =new NODE(config,nearest);
            t.NODETREE::addNode(qrand);
            //  cout <<"added the random node to the tree with Config: " << config[0]<<" "<< config[1]<<" "<< config[2]<<" "<< config[3]<<" "<< config[4]<<" "<< config[5]<<" "<< config[6]<<endl;
        }
    }

}


std::vector<NODE*> RRT::RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<double> goalConfig, double goalBias)
{
    i=0;
    srand(time(NULL));
    env->GetRobots(robots);
    robot=robots.at(0);

    vector<double> initial= {-0.15,0.075,-1.008,-0.11,0,-0.11,0};

    robot->GetActiveDOFLimits(lower,upper);
    double l=0;
  //      double l= M_PI;
    lower[4] = -l;
    lower[6] = -l;  //because they are revolute joints
    upper[4] = l;
    upper[6] = l;

    vector<double > qrand;
    NODE* Nearest;
    NODE* start=new NODE (initial);
    NODETREE t(start);
    double distance;
    cout << "Start Node Config" << start->getConfig()[0]<<" "<<start->getConfig()[1]<<" "<<start->getConfig()[2]<<" "<<start->getConfig()[3]<<" "<<start->getConfig()[4]<<" "<< start->getConfig()[5]<<" "<<start->getConfig()[6]<< endl;


    for(int i=0; i<10000; ++i)
    {
        NODE* endNode;
        endNode= t.getNodes().back();
        distance= euclidianDistance(endNode->getConfig(), goalConfig);

        if(distance>=0.01)
        {

            if(rand()/(double)(RAND_MAX)> goalBias)
            {
                qrand=RRT::Rand();
           //     cout<<" Taking random Node \n";
            }
            else
            {
                qrand=goalConfig;
             //   cout<< "Goal taken as random node \n"<< endl;
            }

            Nearest=RRT::nearestNeighbhor (qrand,t);

            //cout << "qrand : " << i <<"\n" <<qrand[0]<<" "<<qrand[1]<<" "<<qrand[2]<<" "<< qrand[3]<<" "<<qrand[4]<<" "<< qrand[5]<<" "<<qrand[6]<<endl ;
            if(i%100==0)
            cout <<"Number of nodes explored :" << i <<endl;
            RRT::RRTconnect(env,t,Nearest,qrand);

        }
        else
        {
            cout<<"Reached goal"<< endl;
            for(i=0;i<7;++i)
            {
                cout<<endNode->getConfig()[i]<<" , ";
            }
            break;
        }

    }

// Generating the path
    std::vector<NODE*> path;
    NODE* goalNode;
    goalNode =t.lastNode();
    while(goalNode->getParent()!=NULL)
    {
        goalNode=goalNode->getParent();
        path.push_back(goalNode);
    }

// Generating the trajectory
    vector<vector<double>> trajectoryConfig;
    TrajectoryBasePtr trajectory = RaveCreateTrajectory(env,"");
    trajectory->Init(robot->GetActiveConfigurationSpecification());

    for(unsigned int i=0; i<path.size();++i)
    {
        trajectoryConfig.push_back( path[i]->getConfig());
    }

    unsigned int n=trajectoryConfig.size();

    for (unsigned i = 0; i <n ; ++i)
    {
        trajectory->Insert(i,trajectoryConfig.at(n-1-i),true);
    }
    planningutils::RetimeActiveDOFTrajectory(trajectory,robot);
    robot->GetController()->SetPath(trajectory);
    cout<<"Trajectory Executed \n";

    return path ;
}


//std::vector<NODE*> RRT::shortCutSmooth(vector<Node*> path){

//    srand(time(NULL));
//    min=rand() % path.size();
//    max=rand() % path.size();
//    int temp;
//    double dist;
//    vector<double> unitvector;
//    if(min>max)
//    {
//        temp=max;
//        max=min;
//        min=temp;
//    }
//    dist=weightedDistance(path[min]->getConfig(),path[max]->getConfig());

//    for(unsigned int i=0; i<config.size();i++)
//    {
//        unitvector.push_back(0.2*(path[max]->getConfig()->getConfig()[i])/Dist);
//    }


//}

