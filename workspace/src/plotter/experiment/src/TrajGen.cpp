#include "SyncPlayer/TrajGen.h"

namespace SyncPlayer
{
    TrajGen::TrajGen()
    {
        // gen_param = params;
        high = false;
    }

    TrajGen::~TrajGen(){}

    void TrajGen::loadParams(std::map<std::string, double> params)
    {
        gen_param = params;
    }

    void TrajGen::startGen(ros::Time time)
    {
        start_time = time;
    }

    double TrajGen::generate(ros::Time time)
    {
        if(high){
            if(time - start_time < ros::Duration(gen_param["up"])){
                traj = gen_param["max"];
            }
            else{
                high = false;
                start_time = time;
                traj = gen_param["min"];
            }
        }
        else{
            if(time - start_time < ros::Duration(gen_param["down"])){
                traj = gen_param["min"];
            }
            else{
                high = true;
                start_time = time;
                traj = gen_param["max"];
            }
        }
        return traj;
    }
}