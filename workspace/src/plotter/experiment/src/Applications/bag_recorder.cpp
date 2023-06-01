#include <filesystem>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "sync_msgs/CustomData.h"
#include "experiment_srvs/Trigger.h"
#include "sync_msgs/ExperimentData.h"
#include "sync_msgs/SyncQ.h"
#include "sync_msgs/response.h"
#include "sync_msgs/MassData.h"
#include "exo_msgs/state.h"
#include "std_msgs/Float64MultiArray.h"
#include "sync_msgs/MassTrial.h"

namespace fs = std::filesystem;

namespace bagRecorder
{
    rosbag::Bag bag;
    bool record;

    void qCallback(const sync_msgs::CustomData::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            // ROS_INFO_STREAM("Writing q");
            bagRecorder::bag.write("q", msg->header.stamp, msg);
        }
    }

    void stateCallback(const exo_msgs::stateConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("state", msg->header.stamp, msg);
        }
    }

    void statesyncCallback(const sync_msgs::SyncQ::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("state_sync", msg->header.stamp, msg);
        }
    }

    void responseCallback(const sync_msgs::response::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("response", msg->header.stamp, msg);
        }
    }

    void refCallback(const sync_msgs::CustomData::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            // ROS_INFO_STREAM("Writing traj");
            bagRecorder::bag.write("traj", msg->header.stamp, msg);
        }
    }

    void expCallback(const sync_msgs::ExperimentData::ConstPtr& msg)
    {
        // if(bagRecorder::record){
        //     ROS_INFO_STREAM("Writing experiment data");
        //     bagRecorder::bag.write("exp_data", msg->header.stamp, msg);
        // }
        ROS_INFO_STREAM("Writing experiment data");
        bagRecorder::bag.write("exp_data", msg->header.stamp, msg);
    }

    void massCallback(const sync_msgs::MassData::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("mass", ros::Time::now(), msg);
        }
    }

    void emgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("emg", ros::Time::now(), msg);
        }
    }

    void massChangeCallback(const sync_msgs::MassTrial::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            bagRecorder::bag.write("mass_change", msg->header.stamp, msg);
        }
    }

    bool toggleCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res, fs::path bag_path)
    {
        if (req.trigger.data)
        {
            bagRecorder::record = true;
            // bagRecorder::bag.open(bag_path, rosbag::bagmode::Write);
            ROS_INFO_STREAM("Starting recording");
        }
        else
        {
            bagRecorder::record = false;
            bagRecorder::bag.close();
            ROS_INFO_STREAM("Stopped recording");
        }
        return true;
    }

    fs::path makePath(std::string& subject, std::string& path)
    { 
        // bagRecorder::bag_path = path; // https://stackoverflow.com/questions/33149878/experimentalfilesystem-linker-error
        fs::path bag_path = path;
        ROS_INFO_STREAM("Dir: " << bag_path);
        int i = 0;
        for (auto const& dir_entry : fs::directory_iterator{bag_path})
        {
            // ROS_INFO_STREAM("File: " << dir_entry.path().stem());
            if (dir_entry.path().stem().string().find(subject) != std::string::npos)
            {
                ++i;
            }
        }

        if (i > 0)
        {
            bag_path /= subject;
            bag_path += std::to_string(i) + ".bag";
        }
        else
        {
            bag_path /= subject;
            bag_path += ".bag";
        }
        ROS_INFO_STREAM("Save Path: " << bag_path);
        return bag_path;
    }
}



int main(int argc, char **argv)
{
    bagRecorder::record = false;
    

    // rosbag::Bag bag;

    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;

    std::string bag_dir;
    std::string subject;

    ros::param::get("~subject", subject);
    ros::param::get("~save_dir", bag_dir);
    fs::path bag_file = bagRecorder::makePath(subject, bag_dir);

    bool q_sync;
    bool q_ref_sync;
    bool exp;
    bool response;
    bool state;
    bool state_sync;
    bool mass_trial;
    bool emg;
    bool mass_change;

    ros::param::get("~q_sync", q_sync);
    ros::param::get("~q_ref_sync", q_ref_sync);
    ros::param::get("~exp", exp);
    ros::param::get("~response", response);
    ros::param::get("~state", state);
    ros::param::get("~state_sync", state_sync);
    ros::param::get("~mass_trial", mass_trial);
    ros::param::get("~emg", emg);
    ros::param::get("~mass_change", mass_change);

    int rate;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Rate: " << rate);

    ros::Rate loop(rate);

    // bagRecorder::bag.open(bag_file, rosbag::bagmode::Write);
    
    ros::ServiceServer toggle_server = n.advertiseService<experiment_srvs::Trigger::Request, experiment_srvs::Trigger::Response>("toggle_recorder", boost::bind(bagRecorder::toggleCallback, _1, _2, bag_file));

    ros::Subscriber q_sub;
    ros::Subscriber ref_sub;
    ros::Subscriber exp_sub;
    ros::Subscriber response_sub;
    ros::Subscriber state_sub;
    ros::Subscriber state_sync_sub;
    ros::Subscriber mass_trial_sub; 
    ros::Subscriber emg_sub;
    ros::Subscriber mass_change_sub;

    if(q_sync){
        q_sub = n.subscribe("q_sync", 100, bagRecorder::qCallback);
        ROS_INFO_STREAM("Subbed to: q_sync");
    }
    if(q_ref_sync){
        ref_sub = n.subscribe("q_ref_sync", 100, bagRecorder::refCallback);
        ROS_INFO_STREAM("Subbed to: q_ref_sync");
    }
    if(exp){
        exp_sub = n.subscribe("exp", 1, bagRecorder::expCallback);
        ROS_INFO_STREAM("Subbed to: exp");
    }
    if(response){
        response_sub = n.subscribe("response", 1, bagRecorder::responseCallback);
        ROS_INFO_STREAM("Subbed to: response");
    }
    if(state){
        state_sub = n.subscribe("state", 100, bagRecorder::stateCallback);
        ROS_INFO_STREAM("Subbed to: state");
    }
    if(state_sync){
        state_sync_sub = n.subscribe("state_sync", 100, bagRecorder::statesyncCallback);
        ROS_INFO_STREAM("Subbed to: state_sync");
    }
    if(mass_trial){
        mass_trial_sub = n.subscribe("mass_trial", 1, bagRecorder::massCallback);
        ROS_INFO_STREAM("Subbed to: mass_trial");
    }
    if(emg){
        emg_sub = n.subscribe("emg_data", 100, bagRecorder::emgCallback);
        ROS_INFO_STREAM("Subbed to: emg_data");
    }
    if(mass_change){
        mass_change_sub = n.subscribe("mass_change", 60, bagRecorder::massChangeCallback);
        ROS_INFO_STREAM("Subbed to: mass_change");
    }

    bagRecorder::bag.open(bag_file, rosbag::bagmode::Write);

    while(ros::ok())
    {
        ros::spinOnce();
        // loop.sleep();
    }

    return 0;
}