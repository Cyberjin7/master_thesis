#include "ros/ros.h"
#include "sync_msgs/CustomData.h"
#include "sync_msgs/SyncQ.h"
#include "sync_msgs/MassData.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/MassChange.h"
#include "MassChanger/MassChanger.h"


void syncCallback(const sync_msgs::SyncQ::ConstPtr &msg, MassChanger& mc)
{
    mc.updateTime(msg->header.stamp);
    if(mc.start){
        if(mc.current_time - mc.start_time > mc.wait_time){
            if(mc.mass_iterator < mc.mass_order.size()){
                mc.start_time = mc.current_time;
                mc.changeMass();
            }
            else{
                mc.endExperiment();
            }
        }
    }

    // mc.updateTime(msg->header.stamp);
    // if(mc.start){
    //     ros::Duration trial_length;
    //     if(mc.trial_iterator == 0){
    //         trial_length = ros::Duration(mc.trial_params["length"] + mc.delay);
    //     }
    //     else{
    //         trial_length = ros::Duration(mc.trial_params["length"]);
    //     }

    //     if(mc.current_time - mc.start_time > trial_length){
    //         if(mc.trial_iterator < mc.physical_mass_trials.size()){
    //             sync_msgs::MassData mass_trial_msg;
    //             mass_trial_msg.physical_mass = mc.physical_mass_trials[mc.trial_iterator];
    //             mass_trial_msg.virtual_mass = mc.mass_list[mc.mass_order[mc.trial_iterator-1]];
    //             mc.mass_trial_pub.publish(mass_trial_msg);
    //             ROS_INFO_STREAM("Published "<<mc.physical_mass_trials[mc.trial_iterator]<<" at "<<mc.mass_list[mc.mass_order[mc.mass_iterator-1]]);

    //             if((mc.trial_iterator+1)%(int(mc.trial_params["min"])*mc.mass_order.size()) == 0){
    //                 if(mc.mass_iterator < mc.mass_order.size()){
    //                     mc.changeMass();
    //                 }
    //                 else{
    //                     mc.endExperiment();
    //                 }
    //             }

    //             mc.start_time = mc.current_time;
    //             ++(mc.trial_iterator);
    //         }
    //         else{
    //             mc.endExperiment();
    //         }
    //     }
    // }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mass_changer");
    ros::NodeHandle n;

    int rate;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Rate: " << rate);
    ros::Rate loop(rate);

    MassChanger mass_changer(n);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);



    std::vector<std::string> order;
    for (auto const& imap: mass_changer.mass_list)
    {
        order.push_back(imap.first);
    }
    std::shuffle(std::begin(order), std::end(order), rng);

    // phys_list: 
    // physical_mass_trials: 
    std::vector<double> phys_list;
    for(auto it: order){
        for (int j = 0; j < mass_changer.trial_params["min"]; ++j){
            phys_list.push_back(mass_changer.mass_list[it]);
        }
    }

    // TODO: change number of trials from trial_params["min"] to its own dedicated parameter in yaml file
    for(int i=0; i < order.size(); ++i){
        int trial = mass_changer.trial_params["min"]*order.size();
        mass_changer.mass_order.insert(mass_changer.mass_order.end(), trial, order[i]);
        mass_changer.mass_time.insert(mass_changer.mass_time.end(), trial, mass_changer.trial_params["length"]);
        ROS_INFO_STREAM("Number of trials for mass " << order[i] << ": " << trial);

        std::shuffle(std::begin(phys_list), std::end(phys_list), rng);
        mass_changer.physical_mass_trials.insert(mass_changer.physical_mass_trials.end(), phys_list.begin(), phys_list.end());
    }

    // For now: assuming number of trials for a physical mass can be divided by block_size
    int block_size = 3;

    if(mass_changer.physical_mass_trials.size()%block_size != 0){
        ROS_WARN_STREAM("Number of trials cannot be evenly divided into blocks!");
        ros::shutdown();
        return 0;
    }

    std::vector<int> block(mass_changer.physical_mass_trials.size()/block_size);
    for(int i=0;i<block.size(); ++i){
        block[i] = i;
    }

    std::shuffle(std::begin(block), std::end(block), rng);

    std::vector<std::string> new_mass_order;
    std::vector<int> new_mass_time; // this should be double tbh
    std::vector<double> new_physical_mass;
    for(auto it: block){
        ROS_INFO_STREAM(""<<it);    
        int start_index = it*block_size;
        for(int i = start_index; i < start_index+block_size; ++i){
            new_mass_order.push_back(mass_changer.mass_order[i]);
            new_mass_time.push_back(mass_changer.mass_time[i]);
            new_physical_mass.push_back(mass_changer.physical_mass_trials[i]);
        }
    }
    ROS_INFO_STREAM("\n");
    ROS_INFO_STREAM("For Mass Order: ");
    for (int i = 0 ; i < mass_changer.mass_order.size(); ++i){
        ROS_INFO_STREAM("" << i << " Old: " << mass_changer.mass_order[i] << " New: " << new_mass_order[i]);
    }
    ROS_INFO_STREAM("For physical mass order: ");
    for (int i = 0 ; i < mass_changer.physical_mass_trials.size(); ++i){
        ROS_INFO_STREAM("" << i << " Old: " << mass_changer.physical_mass_trials[i] << " New: " << new_physical_mass[i]);
    }
    ROS_INFO_STREAM("\n");

    mass_changer.mass_order = new_mass_order;
    mass_changer.mass_time = new_mass_time;
    mass_changer.physical_mass_trials = new_physical_mass;
    

    // for (int i = 0; i < mass_changer.mass_order.size(); ++i){
    //     ROS_INFO_STREAM("Mass: " << mass_changer.mass_order[i]);
    //     unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //     std::default_random_engine rng(seed);
    //     std::shuffle(std::begin(phys_list), std::end(phys_list), rng);
    //     mass_changer.physical_mass_trials.insert(mass_changer.physical_mass_trials.begin(), phys_list.begin(), phys_list.end());
    //     double trial = mass_changer.trial_params["min"] * mass_changer.mass_order.size();
    //     mass_changer.trials.push_back(trial);
    //     mass_changer.mass_time.push_back(mass_changer.trial_params["length"] * trial);
    // }

    ros::Subscriber subscriber = n.subscribe<sync_msgs::SyncQ>("state_sync", 100, boost::bind(syncCallback, _1, mass_changer));

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}