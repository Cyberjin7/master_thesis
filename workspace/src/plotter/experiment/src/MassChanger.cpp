#include "MassChanger/MassChanger.h"
#include <algorithm>
#include <random>
#include <chrono>
#include "experiment_srvs/MassChange.h"

MassChanger::MassChanger(ros::NodeHandle handle)
{
    this->handler = handle;
    start = false;
    ros::param::get("~mass", this->mass_list);
    ros::param::get("~time", this->time_limits);

    this->start_time = ros::Time(0);

    for (auto const& imap: this->mass_list) // for more info on loop: https://stackoverflow.com/questions/15927033/what-is-the-correct-way-of-using-c11s-range-based-for
    {
        this->mass_order.push_back(imap.first);
    }
    // to shuffle vector, see: https://cplusplus.com/reference/algorithm/random_shuffle/
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);
    std::uniform_int_distribution<int> uniform_dist(this->time_limits["min_length"],this->time_limits["max_length"]);

    std::shuffle(std::begin(this->mass_order), std::end(this->mass_order), rng);
    for (auto i: this->mass_order)
    {
        ROS_INFO_STREAM("Mass: " << i);
        this->mass_time.push_back(uniform_dist(rng));
    }

    // for (int i = 0; i < this->mass_order.size(); ++i)
    // {
    //     this->mass_time.push_back(uniform_dist(rng));
    //     ROS_INFO_STREAM("Duration: " << this->mass_time[i]);
    // }

    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass");
    this->mass_iterator = 0;

}

MassChanger::~MassChanger()
{
}

bool MassChanger::startCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res)
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
    this->mass_srv.request.mass.data = mass;
    this->mass_client.call(this->mass_srv);
}

void MassChanger::updateTime(ros::Time time)
{
    this->current_time = time;
}