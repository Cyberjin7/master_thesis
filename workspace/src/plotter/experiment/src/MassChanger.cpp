#include "MassChanger/MassChanger.h"
#include <algorithm>
#include <random>
#include <chrono>
#include "experiment_srvs/MassChange.h"
#include "sync_msgs/ExperimentData.h"
#include "sync_msgs/MassData.h"

MassChanger::MassChanger(ros::NodeHandle handle)
{
    this->handler = handle;
    start = false;
    // TODO: Parameters should be loaded externally
    ros::param::get("~mass", this->mass_list);
    ros::param::get("~time", this->time_limits);
    // ros::param::get("~mode", this->change_mode);
    ros::param::get("~trial", this->trial_params);

    ros::param::get("~trajectory", this->traj_mode);
    ros::param::get("~delay", this->delay);


    this->start_time = ros::Time(0);

    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass");
    this->mass_iterator = 0;

    this->toggle_server = handle.advertiseService("toggle_mass", &MassChanger::toggleCallback, this);

    if (this->traj_mode != "BAG"){
        this->toggle_recorder = handle.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
    }

    this->exp_pub = this->handler.advertise<sync_msgs::ExperimentData>("exp", 1);

    this->mass_trial_pub = handle.advertise<sync_msgs::MassData>("mass_trial", 1); // publishing real time rather than send as meta data because bagpy sucks with arrays

}

MassChanger::~MassChanger()
{
}

void MassChanger::generateOrder(){}

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

void MassChanger::endExperiment()
{
    this->start = false;
    experiment_srvs::Trigger end_trigger;
    end_trigger.request.trigger.data = this->start;
    end_trigger.request.header.stamp = this->current_time;
    this->toggle_recorder.call(end_trigger);
}

void MassChanger::sendExperimentData()
{
    sync_msgs::ExperimentData exp_data;
    // exp_data.mode = this->change_mode;
    exp_data.trial_length = this->trial_params["length"];
    exp_data.trials = this->trials;
    exp_data.delay = this->delay;
    ros::param::get("~rate", exp_data.rate);
    ros::param::get("~amplitude/min", exp_data.target_low);
    ros::param::get("~amplitude/max", exp_data.target_high);
    for(auto it: this->mass_order){
        exp_data.mass.push_back(this->mass_list[it]);
    }
    exp_data.header.stamp = this->current_time;
    this->exp_pub.publish(exp_data);
}

void MassChanger::sendTrialData(double virtual_mass, double physical_mass, double length)
{
    sync_msgs::MassData trial_data;
    trial_data.virtual_mass = virtual_mass;
    trial_data.physical_mass = physical_mass;
    trial_data.trial_length = length;
    mass_trial_pub.publish(trial_data);
}

void MassChanger::changeMass()
{
    nextMass();
    sendTrialData(this->mass_list[this->mass_order[this->mass_iterator]], 0.0, this->wait_time.toSec());
    ++(this->mass_iterator);
}

void MassChanger::nextMass()
{
    if(this->mass_iterator==0 || (this->mass_order[this->mass_iterator] != this->mass_order[this->mass_iterator-1])){
        // TODO: instead of service, use the publisher mass_trial_pub
        experiment_srvs::MassChange change_mass;
        change_mass.request.mass.data = this->mass_list[this->mass_order[this->mass_iterator]];
        this->mass_client.call(change_mass);
        ROS_INFO_STREAM("Change mass to: " << change_mass.request.mass.data);
    }

    if(this->mass_iterator == 0){
        this->wait_time = ros::Duration(this->mass_time[this->mass_iterator] + this->delay);
    }
    else{
        this->wait_time = ros::Duration(this->mass_time[this->mass_iterator]);
    }

    ROS_INFO_STREAM("Next trial: " << this->wait_time << " seconds");
}

void MassChanger::iterMass()
{
    ++(this->mass_iterator);
}

void MassChanger::updateTime(ros::Time time)
{
    this->current_time = time;
}