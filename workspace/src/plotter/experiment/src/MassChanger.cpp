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

    for (int i = 0; i < this->mass_order.size(); ++i)
    {
        this->mass_time.push_back(uniform_dist(rng));
        ROS_INFO_STREAM("Duration: " << this->mass_time[i]);
    }

    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass");
    this->mass_iterator = 0;

    this->toggle_server = handle.advertiseService("toggle_trigger", &MassChanger::toggleCallback, this);
    this->sync_sub = handle.subscribe("q_sync", 100, &MassChanger::syncCallback, this);

}

MassChanger::~MassChanger()
{
}

bool MassChanger::toggleCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res)
{
    // this->start_time = this->current_time;
    // this->start_time = req.header.stamp;
    this->start = req.trigger.data;
    ROS_INFO_STREAM("Toggle received: " << this->start);
    if (this->start)
    {
        this->start_time = req.header.stamp;
        // experiment_srvs::MassChange change_mass;
        // change_mass.request.mass.data = this->mass_list[this->mass_order[this->mass_iterator]];
        // this->mass_client.call(change_mass);
        // ROS_INFO_STREAM("Change mass to: " << change_mass.request.mass.data);
        // ROS_INFO_STREAM("For " << this->mass_time[this->mass_iterator] << " seconds");
        // ++(this->mass_iterator);
        this->changeMass();
    }
    return true;
}

void MassChanger::syncCallback(const custom_ros_msgs::CustomData::ConstPtr &msg)
{
    // this->current_time = msg->header.stamp;
    this->updateTime(msg->header.stamp);
    if (this->start)
    {
        //ROS_INFO_STREAM("Diff: " << (this->current_time - this->start_time).toSec());
        if ((this->current_time - this->start_time > ros::Duration(this->mass_time[this->mass_iterator-1])) 
                                                        && (this->mass_iterator < this->mass_order.size()))
        {
            this->start_time = this->current_time;
            // experiment_srvs::MassChange change_mass;
            // change_mass.request.mass.data = this->mass_list[this->mass_order[this->mass_iterator]];
            // this->mass_client.call(change_mass);
            // ROS_INFO_STREAM("Change mass to: " << change_mass.request.mass.data);
            // ROS_INFO_STREAM("For " << this->mass_time[this->mass_iterator] << " seconds");
            // ++(this->mass_iterator);
            this->changeMass();
        }
    }
    else
    {

    }
}


void MassChanger::changeMass()
{
    experiment_srvs::MassChange change_mass;
    change_mass.request.mass.data = this->mass_list[this->mass_order[this->mass_iterator]];
    this->mass_client.call(change_mass);
    ROS_INFO_STREAM("Change mass to: " << change_mass.request.mass.data);
    ROS_INFO_STREAM("For " << this->mass_time[this->mass_iterator] << " seconds");
    ++(this->mass_iterator);
}

void MassChanger::updateTime(ros::Time time)
{
    this->current_time = time;
}