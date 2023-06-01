#include "ros/ros.h"
#include <random>
#include "sync_msgs/MassTrial.h"
#include "experiment_srvs/Trigger.h"

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

    std::vector<std::string>::iterator ptr = trials.begin();

    ros::Publisher trial_pub = n.advertise<sync_msgs::MassTrial>("trial", 1);
    ros::ServiceClient start_client = n.serviceClient<experiment_srvs::Trigger>("toggle_recorder");

    sync_msgs::MassTrial trial_msg;

    experiment_srvs::Trigger trigger;

    std::string usr_input;

    while(ros::ok()){
        ROS_INFO_STREAM("Press [ENTER] to iterate trial.");
        //std::cin >> usr_input;
        std::getline(std::cin, usr_input);
        
        if(usr_input.empty()){
            if(ptr == trials.begin()){
                trigger.request.header.stamp = ros::Time::now();
                trigger.request.trigger.data = true;
                start_client.call(trigger);
                ROS_INFO_STREAM("Start recording");
            }
            else if(ptr == trials.end()){
                trigger.request.header.stamp = ros::Time::now();
                trigger.request.trigger.data = false;
                start_client.call(trigger);
                ROS_INFO_STREAM("Stop recording");
                ros::shutdown();
                break;
            }
            // ROS_INFO_STREAM("Iterating");
            // ros::shutdown();
            ROS_INFO_STREAM("Object: " << *ptr);
            ROS_INFO_STREAM("Mass: " << objects[*ptr]);
            if(ptr<trials.end()){
                trial_msg.header.stamp = ros::Time::now();
                trial_msg.object = *ptr;
                trial_msg.mass = objects[*ptr];
                trial_pub.publish(trial_msg);
                ptr++;
            }
            else{
                ROS_INFO_STREAM("Last trial.");
                // ros::shutdown();
            }
        }
        else if(usr_input.compare("q") == 0 || usr_input.compare("Q") == 0){
            ROS_WARN_STREAM("Quit node");
            ros::shutdown();
        }
    }

    return 0;
}