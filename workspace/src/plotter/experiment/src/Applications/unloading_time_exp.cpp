#include "ros/ros.h"
#include "experiment_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include <random>

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res, ros::ServiceClient *client)
{
    experiment_srvs::Trigger start_recording;
    start_recording.request.header.stamp = ros::Time::now();
    start_recording.request.trigger.data = true;
    client->call(start_recording); 
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unloading_experiment");
    ros::NodeHandle n;

    std::vector<std::string> targets;
    std::map<std::string, double> objects;

    ros::param::get("~/targets", targets);
    ros::param::get("~/objects", objects);

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

    ros::Time time;
    bool start;
    int trial_it;
    int order_it;

    ros::ServiceClient start_client = n.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
    ros::ServiceServer start_server = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("toggle_experiment", boost::bind(start,_1,_2, &start_client));


    while(ros::ok()){
        ros::spinOnce();
    }


    return 0;
}