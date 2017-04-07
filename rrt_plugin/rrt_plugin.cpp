
#include "rrt.h"
#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;


class rrt_module : public ModuleBase
{
public:
    RRT rrt;
    rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&rrt_module::MyCommand,this,_1,_2),
                        "Input : MyCommand Goal %f, %f, %f, %f, %f, %f, %f; GoalBias %f; Step %f;shortCutSmooth %d; iterations %d ; biRRTFlag '");
    }
    virtual ~rrt_module() {}


    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {

        std::string input;

        std::vector <double> goalConfig;
        double q;
        char charInput='0';
        sinput >> input;

        // To take goal config values from the input

        if (input == "Goal")
        {
            while(charInput!=';')
            {
                sinput >> q;
                goalConfig.push_back(q);
                sinput >> charInput;
            }

        }

        charInput='0';
        double goalBias;
        double iterations,biRRTflag;
        sinput>>input;
        //To take the Goal Bias value from the input
        if (input== "GoalBias")
            sinput >> goalBias;

        sinput >> charInput; // To store the semicolon from the input
    //cout << "Goal bias" <<goalBias <<endl;

    charInput='0';
    sinput >>input;
    if(input=="biRRTflag")
        sinput>>biRRTflag;
       // cout<<"Bi rrt flag : " << biRRTflag << endl;


        charInput='0';
        int shortCutSmooth;
        sinput >>input;
        //To take the Shortcutsmooth flag
        if(input=="shortCutsmooth")
            sinput>>shortCutSmooth;
        sinput >> charInput;

        charInput='0';
        sinput >>input;
        if(input=="iterations")
            sinput>>iterations;

        charInput='0';
        double step;
        sinput >> input;
        //To take the Step Size from the input
        if (input == "Step")
            sinput >> step;
        sinput >> charInput;
        vector<double> startConfig={-0.15,0.075,-1.008,-0.11,0,-0.11,0};

        vector<NODE*> path;
        if(biRRTflag==1)
        {
            path=rrt.BiRRTPlanner(GetEnv(),startConfig,goalConfig);

        }
        else
        {
            path= rrt.RRTPlanner(GetEnv(),goalConfig, goalBias);
        }

        // path=rrt.shortCutSmooth(GetEnv(), path, iterations, goalConfig, goalBias);

        reverse(path.begin(),path.end());
        for(unsigned int i=0;i<path.size();i++)
        {
            for (unsigned int j=0;j<path[0]->getConfig().size();++j)
            {
                sout<< path[i]->getConfig()[j];
                if (j !=path[0]->getConfig().size()-1) sout<<",";
            }
            if (i !=path.size()-1) sout<<endl;
        }
        return true;
    }


};

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrt_module" ) {
        return InterfaceBasePtr(new rrt_module(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("rrt_module");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
