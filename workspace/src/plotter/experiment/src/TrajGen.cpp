#include "SyncPlayer/TrajGen.h"

namespace SyncPlayer
{
    TrajGen::TrajGen()
    {
        // gen_param = params;
        high = false;
        peak = false;
        start = false;
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
        this->start = true;
    }

    bool TrajGen::getPeak()
    {
        return this->peak;
    }

    bool TrajGen::getHigh()
    {
        return this->high;
    }

    bool TrajGen::get_start()
    {
        return this->start;
    }

    void TrajGen::set_start(bool toggle)
    {
        this->start = toggle;
    }

    double TrajGen::generate(ros::Time time)
    {
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