#include <filesystem>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "custom_ros_msgs/CustomData.h"
#include "experiment_srvs/Trigger.h"

namespace fs = std::filesystem;

namespace bagRecorder
{
    rosbag::Bag bag;
    bool record;

    void qCallback(const custom_ros_msgs::CustomData::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            ROS_INFO_STREAM("Writing q");
            bagRecorder::bag.write("q", msg->header.stamp, msg);
        }
    }

    void refCallback(const custom_ros_msgs::CustomData::ConstPtr& msg)
    {
        if(bagRecorder::record)
        {
            ROS_INFO_STREAM("Writing traj");
            bagRecorder::bag.write("traj", msg->header.stamp, msg);
        }
    }

    bool toggleCallback(experiment_srvs::Trigger::Request &req, experiment_srvs::Trigger::Response &res, fs::path bag_path)
    {
        if (req.trigger.data)
        {
            bagRecorder::record = true;
            bagRecorder::bag.open(bag_path, rosbag::bagmode::Write);
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
    ros::Rate loop(20);

    std::string bag_dir;
    std::string subject;

    ros::param::get("~subject", subject);
    ros::param::get("~save_dir", bag_dir);
    fs::path bag_file = bagRecorder::makePath(subject, bag_dir);

    // bagRecorder::bag.open(bag_file, rosbag::bagmode::Write);
    
    ros::Subscriber q_sub = n.subscribe("q_sync", 100, bagRecorder::qCallback);
    ros::Subscriber ref_sub = n.subscribe("q_ref_sync", 100, bagRecorder::refCallback);
    ros::ServiceServer toggle_server = n.advertiseService<experiment_srvs::Trigger::Request, experiment_srvs::Trigger::Response>("toggle_trigger", boost::bind(bagRecorder::toggleCallback, _1, _2, bag_file));

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}