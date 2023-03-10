#ifndef SYNC_PLAYER_H
#define SYNC_PLAYER_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "custom_ros_msgs/CustomData.h"
#include "experiment_srvs/Trigger.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace SyncPlayer
{
    class SyncPlayer
    {
    private:
        // rosbag::Bag bag;
        // rosbag::View traj_view;
        std::string trajectory_mode;
    public:
        ros::Publisher q_pub; // change to sync_pub
        ros::Publisher q_ref_pub; // change to sync_bag_pub
        ros::Time sync_time;
        ros::Time future_time;
        ros::Time node_start_time;
        ros::Duration time_diff;
        int delay;
        bool play;
        rosbag::Bag bag;
        rosbag::View traj_view;
        rosbag::View::iterator view_it;
        uint32_t bag_it;
        custom_ros_msgs::CustomData sync_msg;
        std_msgs::Float64::ConstPtr q;
        ros::Subscriber q_sub;
        ros::ServiceClient toggle_mass;
        ros::ServiceClient toggle_recorder;
        // ros::ServiceServer start_service;

        SyncPlayer(ros::NodeHandle handler, int time_delay, std::string traj_mode);
        ~SyncPlayer();
        void qCallback(const std_msgs::Float64::ConstPtr& msg);
        bool toggleCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        ros::Time calculateFuture();
        ros::Time calculateTime();
        void loadBag(std::string bag_path);
        void synchronize();
        void playToggle();
        void itBag();

    };
}

#endif