#include "ros/ros.h"
#include "custom_ros_msgs/CustomData.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/Trigger.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "SyncPlayer/SyncPlayer.h"
#include "MassChanger/MassChanger.h"

#include <fstream>


// bool startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, SyncPlayer* play) // SyncPlayer* play with &player works
// {
//     play->playToggle(true);
//     // ROS_INFO_STREAM("Toggle: " << play->play);
//     return true;
// }

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

    std::ifstream bag_file;
    bag_file.open(bag_path);
    if(!bag_file)
    {
        ROS_WARN_STREAM("Bag file doesn't exist.");
        return 0;
    }

    SyncPlayer player(n, delay);
    player.loadBag(bag_path);

    MassChanger mass(n);

    //ros::Subscriber q_sub = n.subscribe("q", 100, &SyncPlayer::qCallback, &player);
    // ros::ServiceServer start_srv = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("user_trigger", boost::bind(startCallback, _1, _2, &player));
    // Source: https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998#post-id-63998 
    ros::ServiceServer start_srv = n.advertiseService("user_trigger", &SyncPlayer::startCallback, &player);
    ros::Rate loop(20);

    while (ros::ok())
    {
        // player.synchronize();
        mass.updateTime(player.sync_time);
        
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}