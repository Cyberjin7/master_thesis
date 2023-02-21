#include "ros/ros.h"
#include "custom_ros_msgs/CustomData.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/Trigger.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <fstream>


class SyncPlayer{
    public:
        ros::Publisher q_pub;
        ros::Publisher q_ref_pub;
        ros::Time sync_time;
        ros::Time future_time;
        ros::Time node_start_time;
        ros::Duration time_diff;
        int delay;
        bool play;

        SyncPlayer(ros::NodeHandle handler, int time_delay){
            q_pub = handler.advertise<custom_ros_msgs::CustomData>("q_sync", 100);
            q_ref_pub = handler.advertise<custom_ros_msgs::CustomData>("q_ref_sync", 100);
            play = false;
            delay = time_delay;
            node_start_time = ros::Time::now();
            time_diff = ros::Duration(0);
        }

        void qCallback(const std_msgs::Float64::ConstPtr& msg);
        bool startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        ros::Time calculateFuture();
        ros::Time calculateTime();

};

void SyncPlayer::qCallback(const std_msgs::Float64::ConstPtr& msg)
{
    custom_ros_msgs::CustomData sync_msg;
    sync_msg.header.stamp = this->sync_time;
    sync_msg.value.data = msg->data;
    this->q_pub.publish(sync_msg);
}

bool SyncPlayer::startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO_STREAM("Received trigger");
    this->play = true;
    return true;
}

ros::Time SyncPlayer::calculateTime()
{
    ros::Time time_now = ros::Time::now();
    this->time_diff = time_now - this->node_start_time;
    time_now.sec = this->time_diff.sec;
    time_now.nsec = this->time_diff.nsec;
    return time_now;
}

ros::Time SyncPlayer::calculateFuture()
{
    this->future_time = this->sync_time;
    this->future_time.sec += this->delay;
    return this->future_time;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_player");
    ros::NodeHandle n;

    std::string bag_path;
    int delay;
    ros::param::get("~bag_path", bag_path);
    ros::param::get("~delay", delay);

    //ROS_INFO_STREAM("Path is: " << bag_path);
    ROS_INFO_STREAM("Delay is:" << delay);

    std::ifstream bag_file;
    bag_file.open(bag_path);
    if(!bag_file)
    {
        ROS_WARN_STREAM("Bag file doesn't exist.");
        return 0;
    }

    SyncPlayer player(n, delay);
    ros::Subscriber q_sub = n.subscribe("q", 100, &SyncPlayer::qCallback, &player);
    ros::ServiceServer start_service = n.advertiseService("start_trigger", &SyncPlayer::startCallback, &player);


    ros::Rate loop(20);

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    
    custom_ros_msgs::CustomData ref_msg;

    while (ros::ok())
    {

        if(player.play)
        {
            for(rosbag::MessageInstance const m: rosbag::View(bag))
            {
                if(ros::isShuttingDown())
                {
                    bag.close();
                    break;
                }
                else
                {
                    std_msgs::Float64::ConstPtr i = m.instantiate<std_msgs::Float64>();
                    if (i != nullptr)
                    {
                        player.sync_time = player.calculateTime();
                        ref_msg.header.stamp = player.calculateFuture();
                        ref_msg.value.data = i->data;
                        player.q_ref_pub.publish(ref_msg);
                    }
                    ros::spinOnce();
                    loop.sleep();
                }
            }
            bag.close();
            player.play = false;
        }
        
        if(!player.play)
        {
            player.sync_time = player.calculateTime();
            ref_msg.header.stamp = player.calculateFuture();
            ref_msg.value.data = 0;
            player.q_ref_pub.publish(ref_msg);
        }

        
        ros::spinOnce();
        loop.sleep();
    }



    return 0;
}