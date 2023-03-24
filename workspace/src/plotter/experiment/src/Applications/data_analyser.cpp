#include "ros/ros.h"
#include "sync_msgs/SyncQ.h"

void stateCallback(const sync_msgs::SyncQ::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_analyzer");
    ros::NodeHandle n;

    int rate;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Rate: " << rate);

    ros::Rate loop(rate);

    ros::Subscriber state_sub = n.subscribe("state_sync", 100, stateCallback);

    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}