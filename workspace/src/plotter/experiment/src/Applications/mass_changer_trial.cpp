#include "ros/ros.h"
#include "sync_msgs/CustomData.h"
#include "sync_msgs/SyncQ.h"
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
    std::uniform_int_distribution<int> uniform_dist(mass_changer.trial_params["min"], mass_changer.trial_params["max"]);
    for(int i=0; i <mass_changer.mass_order.size(); ++i){
        ROS_INFO_STREAM("Mass: " << mass_changer.mass_order[i]);
        int trial = uniform_dist(rng);
        ROS_INFO_STREAM("Number of trials: " << trial);
        mass_changer.trials.push_back(trial);
        mass_changer.mass_time.push_back(mass_changer.trial_params["length"]*trial);
        ROS_INFO_STREAM("Duration: " << mass_changer.mass_time[i]);
    }

    ros::Subscriber subscriber = n.subscribe<sync_msgs::SyncQ>("state_sync", 100, boost::bind(syncCallback, _1, mass_changer));    

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}