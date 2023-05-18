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
            bool high;
            bool start;
            double traj;
            double peak;
        public:
            TrajGen();
            ~TrajGen();
            double generate(ros::Time time);
            void startGen(ros::Time time);
            void loadParams(std::map<std::string, double> params);
            bool getPeak();
            bool getHigh();

            bool get_start();
            void set_start(bool toggle);
    };

}

#endif