#include "MassChanger/MassChanger.h"
#include <algorithm>
#include <random>
#include <chrono>
#include "experiment_srvs/MassChange.h"
#include "sync_msgs/ExperimentData.h"

MassChanger::MassChanger(ros::NodeHandle handle)
{
    this->handler = handle;
    start = false;
    ros::param::get("~mass", this->mass_list);
    ros::param::get("~time", this->time_limits);
    ros::param::get("~mode", this->change_mode);
    ros::param::get("~trial", this->trial_params);

    ros::param::get("~trajectory", this->traj_mode);
    ros::param::get("~delay", this->delay);


    this->start_time = ros::Time(0);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);


    // for (auto const& imap: this->mass_list) // for more info on loop: https://stackoverflow.com/questions/15927033/what-is-the-correct-way-of-using-c11s-range-based-for
    // {
    //     this->mass_order.push_back(imap.first);
    // }
    // std::shuffle(std::begin(this->mass_order), std::end(this->mass_order), rng);
    this->mass_order = randomizeOrder(this->mass_list, rng);

    if (this->change_mode == "TIME"){
        std::uniform_int_distribution<int> uniform_dist(this->time_limits["min_length"],this->time_limits["max_length"]);
        for (int i = 0; i < this->mass_order.size(); ++i)
        {
            ROS_INFO_STREAM("Mass: " << this->mass_order[i]);
            this->mass_time.push_back(uniform_dist(rng));
            ROS_INFO_STREAM("Duration: " << this->mass_time[i]);
        }
    }
    else if (this->change_mode == "TRIAL"){
        std::uniform_int_distribution<int> uniform_dist(this->trial_params["min"],this->trial_params["max"]);
        for (int i = 0; i < this->mass_order.size(); ++i)
        {
            ROS_INFO_STREAM("Mass: " << this->mass_order[i]);
            int trial = uniform_dist(rng);
            ROS_INFO_STREAM("Number of trials: " << trial);
            this->trials.push_back(trial);
            this->mass_time.push_back(this->trial_params["length"] * trial);
            ROS_INFO_STREAM("Duration: " << this->mass_time[i]);
        }
    }

    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass");
    this->mass_iterator = 0;

    this->toggle_server = handle.advertiseService("toggle_mass", &MassChanger::toggleCallback, this);
    this->sync_sub = handle.subscribe("q_sync", 100, &MassChanger::syncCallback, this);

    if (this->traj_mode != "BAG"){
        this->toggle_recorder = handle.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
    }

    this->exp_pub = this->handler.advertise<sync_msgs::ExperimentData>("exp", 1);

}

MassChanger::~MassChanger()
{
}

std::vector<std::string> MassChanger::randomizeOrder(std::map<std::string, double> list, std::default_random_engine rng)
{
    std::vector<std::string> order;
    for (auto const& imap: list) // for more info on loop: https://stackoverflow.com/questions/15927033/what-is-the-correct-way-of-using-c11s-range-based-for
    {
        order.push_back(imap.first);
    }
    // to shuffle vector, see: https://cplusplus.com/reference/algorithm/random_shuffle/
    std::shuffle(std::begin(order), std::end(order), rng);

    return order;
}

bool MassChanger::toggleCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res)
{
    // this->start_time = this->current_time;
    // this->start_time = req.header.stamp;
    this->start = req.trigger.data;
    ROS_INFO_STREAM("Toggle received: " << this->start);

    if (this->traj_mode != "BAG"){
        experiment_srvs::Trigger record_trigger;
        record_trigger.request.trigger.data = this->start;
        record_trigger.request.header.stamp = this->current_time;
        this->toggle_recorder.call(record_trigger);
    }

    if (this->start)
    {   
        this->sendExperimentData();
        this->start_time = req.header.stamp;
        this->changeMass();
    }
    return true;
}

void MassChanger::sendExperimentData()
{
    sync_msgs::ExperimentData exp_data;
    exp_data.mode = this->change_mode;
    exp_data.trial_length = this->trial_params["length"];
    exp_data.trials = this->trials;
    for(auto it: this->mass_order){
        exp_data.mass.push_back(this->mass_list[it]);
    }
    exp_data.header.stamp = this->current_time;
    this->exp_pub.publish(exp_data);
}

void MassChanger::syncCallback(const sync_msgs::CustomData::ConstPtr &msg)
{
    // this->current_time = msg->header.stamp;
    this->updateTime(msg->header.stamp);
    if (this->start)
    {
        if (this->current_time - this->start_time > this->wait_time)
        {
            if (this->mass_iterator < this->mass_order.size()){
                this->start_time = this->current_time;
                this->changeMass();
            }
            else{
                this->start = false;
                experiment_srvs::Trigger end_trigger;
                end_trigger.request.trigger.data = this->start;
                end_trigger.request.header.stamp = this->current_time;
                this->toggle_recorder.call(end_trigger);
            }
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

    if(this->mass_iterator == 0){
        this->wait_time = ros::Duration(this->mass_time[this->mass_iterator] + this->delay);
    }
    else{
        this->wait_time = ros::Duration(this->mass_time[this->mass_iterator]);
    }

    ROS_INFO_STREAM("For " << this->wait_time << " seconds");
    ++(this->mass_iterator);
}

void MassChanger::updateTime(ros::Time time)
{
    this->current_time = time;
}