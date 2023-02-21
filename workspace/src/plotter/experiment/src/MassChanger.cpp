#include "MassChanger/MassChanger.h"

MassChanger::MassChanger(ros::NodeHandle handle)
{
    handler = handle;
    start = false;
    handle.getParam("~mass", this->mass_list);
    handle.getParam("~time", this->time_limits);
    this->start_time = ros::Time(0);
    for (auto const& imap: this->mass_list) // for more info on loop: https://stackoverflow.com/questions/15927033/what-is-the-correct-way-of-using-c11s-range-based-for
    {
        this->mass_order.push_back(imap.first);
    }
    // to shuffle vector, see: https://cplusplus.com/reference/algorithm/random_shuffle/
}

MassChanger::~MassChanger()
{
}

bool MassChanger::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    this->start_time = this->current_time;
    start = true;
    return true;
}

void MassChanger::syncCallback(const custom_ros_msgs::CustomData::ConstPtr &msg)
{
    this->current_time = msg->header.stamp;
}


void MassChanger::changeMass(double mass)
{
}