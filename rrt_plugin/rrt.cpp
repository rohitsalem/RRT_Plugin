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

// Functions of RRT class
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

//Function to calculate Weighted distance
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

// Function to get the NearestNeighbor
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

// Function returns a vector configuration by adding two configurations
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

// Returns a vector of random configuration
vector<double> RRT::randConfiguration()
{

    vector<double> randConfig;
    for (unsigned int j = 0;j<7;j++)
        randConfig.push_back(  ((double)rand()/RAND_MAX) * (upper[j]-lower[j]) +lower[j]);
    return randConfig;


}

//empty constructor for NODETREE
NODETREE::NODETREE()
{

}

//Constructor to initialize a Tree with an initial node
NODETREE::NODETREE(NODE *init)
{
    startNode=init;
    _nodes.push_back(startNode);
}

//Destructor for NODETREE
NODETREE::~NODETREE()
{
    _nodes.clear();
}

//returns the last node of the tree
NODE* NODETREE::lastNode()
{
    return _nodes.back();
}

//Function to check if the joint limits are in range
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

    distance=Dist;
    int j=0;
    while(distance>0.2)
    {

        v= RRT::vectorAdd(nearest->getConfig(),unitvector);
        robot->SetActiveDOFValues(v);
        if(!(env->CheckCollision(robot))&& !(robot->CheckSelfCollision()) && !isNotInlimits(v))
        {      if(j<10) //checking if the number of steps taken are less than 10
            {
                nearest=new NODE(v,nearest);
                t.NODETREE::addNode(nearest);
                distance=euclidianDistance(v,config);
                ++j;
            }
            else
                break;
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
        }
    }

}


std::vector<NODE*> RRT::RRTPlanner(OpenRAVE::EnvironmentBasePtr env, vector<double> goalConfig, double goalBias)
{
    i=0;
    // auto startTime=get_time::now();
    cout <<"RRT Planner :" <<endl;
    srand(time(NULL));
    env->GetRobots(robots);
    robot=robots.at(0);

    vector<double> initial= {-0.15,0.075,-1.008,-0.11,0,-0.11,0};

    robot->GetActiveDOFLimits(lower,upper);
    double l=0; //Set this l=0 to make it fast, because joint 4 and 6 are revolute joints
    //double l= M_PI;
    lower[4] = -l;
    lower[6] = -l;
    upper[4] = l;
    upper[6] = l;

    vector<double > qrand;
    NODE* Nearest;
    NODE* start=new NODE (initial);
    NODETREE t(start);
    double distance;
    int number;
    for(int i=0; i<30000; ++i) //the loop terminated if the number of nodes explored is greater than 3000
    {
        number=i;
        NODE* endNode;
        endNode= t.getNodes().back();
        distance= euclidianDistance(endNode->getConfig(), goalConfig);

        if(distance>=0.01) //The loop goes on till the distance between last node of the tree and goal node is greater than 0.1
        {
            if(rand()/(double)(RAND_MAX)> goalBias) //The if/else conditions to choose the next node in the tree to be a random node or the goal node: depends on the goal Bias.
            {
                qrand=RRT::randConfiguration();
            }
            else
            {
                qrand=goalConfig;
            }

            Nearest=RRT::nearestNeighbhor (qrand,t); //getting the nearest node in the tree from the
            if(i%100==0)
            cout <<"Number of nodes explored :" << i <<endl;
            RRT::RRTconnect(env,t,Nearest,qrand);

        }
        else
        {
            cout<<"Reached goal"<< endl;
            break;
        }

    }

    cout <<"no of Nodes explored :" << number<< endl;

    // Generating the path
    std::vector<NODE*> path;
    NODE* goalNode;
    goalNode =t.lastNode();
    while(goalNode->getParent()!=NULL)
    {
        goalNode=goalNode->getParent();
        path.push_back(goalNode);
    }


    //Generating the trajectory

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

//    auto endTime=get_time::now();
//    auto rrtTime=endTime-startTime;

//       cout<<"RRT Time:"<< chrono::duration_cast<s>(rrtTime).count()<<endl;
//     cout<<"RRT Path Size:" <<path.size() <<endl;

    //path=shortCutSmooth(env, path, 200, goalConfig, goalBias);

    return path ;
}


std::vector<NODE*> RRT::shortCutSmooth(OpenRAVE::EnvironmentBasePtr env,vector<NODE*> path, int iterations,vector<double> goalConfig, double goalBias)
{
    //cout <<"Path Size Before Smoothing : " << path.size() << endl;
    //  auto startTime=get_time::now();
    for(i=0;i<200;i++)
    {
        //cout <<path.size() << endl;
        min=rand() % path.size();
        max=rand() % path.size();
        int temp;
        if(abs(max-min)>1)
        {

            if(min>max)
            {
                temp=max;
                max=min;
                min=temp;
            }

            NODE* nodeMax= path[max];
            NODE* nodeMin=path[min];

            bool collision=false;
            double dist;

            vector<double> configMax;
            vector<double> configMin;
            vector<double> unitvector;

            configMax=nodeMax->getConfig();
            configMin=nodeMin->getConfig();


            dist=euclidianDistance(configMin,configMax);

            for(unsigned int i=0; i<7;++i)
            {
                unitvector.push_back(0.2*(configMax[i]-configMin[i])/dist);

            }

            while(dist>0.2)
            {

                configMin= RRT::vectorAdd(configMin,unitvector);

                if(!isNotInlimits(configMin))
                {
                    robot->SetActiveDOFValues(configMin);
                    if(!(env->CheckCollision(robot))&& !(robot->CheckSelfCollision()))
                    {


                        dist=euclidianDistance(configMin,configMax);
                        //  cout << "Distance :" << dist <<endl;
                    }

                    else
                    {
                        // cout<<"Collision "<< endl;
                        collision=true;
                        break;
                    }
                }
                else
                    break;
            }
            if(!collision)
            {
                path.erase(path.begin()+min+1,path.begin()+max);

            }

        }
        //  cout  <<path.size() << endl;
    }
    //  auto endTime=get_time::now();
    //auto smoothTime=endTime-startTime;
    //cout<<"Smoothened Path size:"<<path.size()<<endl;
    //    cout<<"Smoothing Time:"<< chrono::duration_cast<ns>(smoothTime).count()<<endl;;
    return path;
}


std::vector<NODE*> RRT::BiRRTPlanner(OpenRAVE::EnvironmentBasePtr env,vector<double> startConfig,vector<double> goalConfig)
{
    srand(time(NULL));
    cout<<"Bi RRT Planner " <<endl;
    //auto startTime=get_time::now();
    env->GetRobots(robots);
    robot=robots.at(0);
    robot->GetActiveDOFLimits(lower,upper);
    //double l=0;
    double l= M_PI/4;
    lower[4] = -l;
    lower[6] = -l;  //because they are revolute joints
    upper[4] = l;
    upper[6] = l;
    NODE* START =new NODE(startConfig);
    NODE* GOAL = new NODE(goalConfig);

    NODETREE t1(START);
    NODETREE temp;
    NODETREE t2(GOAL);
    //cout <<"hello" <<endl;
    while(t1.lastNode()->getConfig()!=t2.lastNode()->getConfig())
    {

        vector<double> qrand= RRT::randConfiguration();

        NODE* node1= nearestNeighbhor(qrand,t1);

        RRT::RRTconnect(env,t1,node1,qrand);

        qrand=t1.lastNode()->getConfig();

        node1= nearestNeighbhor(qrand,t2);
        RRT::RRTconnect(env,t2,node1,qrand);
        temp=t1;
        t1=t2;
        t2=temp;

    }
    if((t1.startNode->getConfig()==startConfig))
    {
        temp=t1;
        t1=t2;
        t2=temp;
    }

    std::vector<NODE*> path;
    NODE* goalNode;
    goalNode =t1.lastNode();
    while(goalNode->getParent()!=NULL)
    {
        goalNode=goalNode->getParent();
        path.push_back(goalNode);
    }
    reverse(path.begin(),path.end());

    goalNode =t2.lastNode();
    while(goalNode->getParent()!=NULL)
    {
        goalNode=goalNode->getParent();
        path.push_back(goalNode);
    }
//    auto endTime =get_time::now();
//    auto biTime=endTime-startTime;
//    cout<<"Birrt Path size:"<<path.size()<<endl;
//      cout<<"Birrt Time:"<< chrono::duration_cast<ns>(biTime).count()<<endl;
   cout <<"Number of Nodes: " << t1.getNodes().size()+t2.getNodes().size()<<endl;

    //Generating the trajectory
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

    return path;
}



