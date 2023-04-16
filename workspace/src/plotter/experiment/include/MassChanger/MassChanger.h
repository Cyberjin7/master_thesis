#ifndef MASS_CHANGER_H
#define MASS_CHANGER_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sync_msgs/CustomData.h"
#include "experiment_srvs/MassChange.h"
#include "experiment_srvs/Trigger.h"

#include <random>

class MassChanger
{
private:
    /* data */
public:
    MassChanger(ros::NodeHandle handle);
    ~MassChanger();
    bool toggleCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res);
    void syncCallback(const sync_msgs::CustomData::ConstPtr &msg); 
    void changeMass();
    virtual void generateOrder();
    ros::Time start_time;
    ros::Time current_time;
    bool start;
    ros::NodeHandle handler;

    std::string change_mode;
    std::string traj_mode;
    int delay;

    std::map<std::string, double> mass_list;
    std::map<std::string, double> trial_params;
    std::map<std::string, int> time_limits;
    std::vector<std::string> mass_order;
    std::vector<double> trials;
    std::vector<double> physical_mass_trials;

    ros::Duration wait_time;
    std::vector<int> mass_time; 
    int mass_iterator;
    int trial_iterator;

    ros::Subscriber sync_sub;
    ros::ServiceClient mass_client;
    ros::ServiceServer toggle_server;
    experiment_srvs::MassChange mass_srv;
    ros::ServiceClient toggle_recorder;
    ros::Publisher exp_pub;
    ros::Publisher mass_trial_pub;

    void updateTime(ros::Time time);

    std::vector<std::string> randomizeOrder(std::map<std::string, double> list, std::default_random_engine rng);
    void sendExperimentData();
    void endExperiment();
};

#endif