#include "SyncPlayer/TrajGen.h"

namespace SyncPlayer
{
    TrajGen::TrajGen()
    {
        // gen_param = params;
        high = false;
        peak = false;
    }

    TrajGen::~TrajGen(){}

    void TrajGen::loadParams(std::map<std::string, double> params)
    {
        gen_param = params;
    }

    void TrajGen::startGen(ros::Time time)
    {
        start_time = time;
        traj = gen_param["min"];
    }

    bool TrajGen::getPeak()
    {
        return this->peak;
    }

    double TrajGen::generate(ros::Time time)
    {
        // if(high){
        //     if(time - start_time < ros::Duration(gen_param["up"])){
        //         traj = gen_param["max"];
        //     }
        //     else{
        //         high = false;
        //         start_time = time;
        //         traj = gen_param["min"];
        //     }
        // }
        // else{
        //     if(time - start_time < ros::Duration(gen_param["down"]/2)){
        //         traj = gen_param["min"];
        //     }
        //     else{
        //         if (!peak){
        //             high = true;
        //             traj = gen_param["max"];
        //             peak = true;
        //         }
        //         else{
        //             peak = false;
        //         }
        //         start_time = time;
        //     }
        // }

        if(high && time - start_time >= ros::Duration(gen_param["up"])){
                high = false;
                traj = gen_param["min"];
                start_time = time;
            }
            else if(!high && time-start_time >= ros::Duration(gen_param["down"]/2)){
                if (!peak){
                    high = true;
                    traj = gen_param["max"];
                    peak = true;
                }
                else{
                    peak = false;
                }
                start_time = time;
            }

        return traj;
    }
}