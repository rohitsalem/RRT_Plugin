#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

class rrt_module : public ModuleBase
{
public:
    rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&rrt_module::MyCommand,this,_1,_2),
                        "Input : Goal <%f, %f, %f, %f, %f, %f, %f> ; GoalBias <0.1>; Step <0.3>; Weights < > ");

    }
    virtual ~rrt_module() {}


    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;

        std::vector <float> goalConfig;
        float q;
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
        float goalBias;
        sinput>>input;
        //To take the Goal Bias value from the input
        if (input== "GoalBias")
            sinput >> goalBias;
        sinput >> charInput; // To store the semicolon from the input

        charInput='0';
        float step;
        sinput >> input;
        //To take the Step Size from the input
        if (input == "Step")
            sinput >> step;

        charInput='0';
        std::vector<float> weights;
        float w;
        sinput >> input;
        //To take the Weights from the input
        if (input ==" Weights")
        {
            while(charInput !=';')
            {
                sinput >> w;
                weights.push_back(w);
                sinput >> charInput;
            }
        }

            sout << "output";
        return true;}

    //need to take data returned form the rrt and send it to python as path.
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
