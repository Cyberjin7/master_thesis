#include "ros/ros.h"
#include "custom_ros_msgs/CustomData.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/Trigger.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "SyncPlayer/SyncPlayer.h"

#include <fstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_player");
    ros::NodeHandle n;

    std::string bag_path;
    int delay;
    ros::param::get("~bag_path", bag_path);
    ros::param::get("~delay", delay);

    ROS_INFO_STREAM("Path is: " << bag_path);
    ROS_INFO_STREAM("Delay is:" << delay);

    SyncPlayer player(n, delay);
    try{
        player.loadBag(bag_path);
    }
    catch(rosbag::BagIOException)
    {
        return 0;
    }

    //ros::Subscriber q_sub = n.subscribe("q", 100, &SyncPlayer::qCallback, &player);
    ros::ServiceServer start_service = n.advertiseService("start_trigger", &SyncPlayer::startCallback, &player);

    ros::Rate loop(20);

    while (ros::ok())
    {
        player.synchronize();
        
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}