#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "ros/ros.h"

namespace SyncPlayer
{
    class TrajGen
    {
        private:
            ros::Time start_time;
            std::map<std::string, double> gen_param;
        public:
            TrajGen(std::map<std::string, double> params);
            ~TrajGen();
            double generate();
            void startGen(ros::Time time);
    };

}

#endif