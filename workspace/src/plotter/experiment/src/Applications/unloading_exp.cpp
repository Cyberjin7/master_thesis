#include "ros/ros.h"
#include <random>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "unloading_experiment");
    ros::NodeHandle n;

    std::vector<std::string> targets;

    ros::param::get("~/targets", targets);

    int trial_size;
    int block_size;

    ros::param::get("~trials", trial_size);
    ros::param::get("~block", block_size);

    std::vector<std::string> trials;

    if(trial_size%block_size != 0){
        ROS_WARN_STREAM("Trial cannot be divided into equal blocks. Please adjust");
        ros::shutdown();
        return 0;
    }
    else{
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine rng(seed);

        std::vector<std::string> order;
        int blocks = trial_size/block_size;

        for(auto it:targets){
            order.insert(order.end(), blocks, it);
        }
        std::shuffle(std::begin(order), std::end(order), rng);

        for(auto block:order){
            trials.insert(trials.end(), block_size, block);
        }

        for(auto it:trials){
            ROS_INFO_STREAM("" << it);
        }
        
    }



    while(ros::ok()){

    }

    return 0;
}