#ifndef MASS_CHANGER_H
#define MASS_CHANGER_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "custom_ros_msgs/CustomData.h"
#include "experiment_srvs/MassChange.h"

class MassChanger
{
private:
    /* data */
public:
    MassChanger(ros::NodeHandle handle);
    ~MassChanger();
    bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void syncCallback(const custom_ros_msgs::CustomData::ConstPtr &msg);
    void changeMass(double mass);
    ros::Time start_time;
    ros::Time current_time;
    bool start;
    ros::NodeHandle handler;
    std::map<std::string, double> mass_list;
    std::map<std::string, int> time_limits;
    std::vector<std::string> mass_order;
    std::vector<int> mass_time; 
    ros::ServiceClient mass_client;
    experiment_srvs::MassChange mass_srv;
    int mass_iterator;
};

// MassChanger::MassChanger(ros::NodeHandle handle)
// {
// }

// MassChanger::~MassChanger()
// {
// }

// bool MassChanger::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
// {
// }

// void MassChanger::syncCallback(const custom_ros_msgs::CustomData::ConstPtr& msg)
// {
// }

// void MassChanger::changeMass(double mass)
// {
// }

#endif