#include "SyncPlayer/TrajGen.h"

namespace SyncPlayer
{
    TrajGen::TrajGen(std::map<std::string, double> params)
    {
        gen_param = params;
    }

    TrajGen::~TrajGen(){}

    void TrajGen::startGen(ros::Time time)
    {
        start_time = time;
    }

    double TrajGen::generate()
    {

    }
}