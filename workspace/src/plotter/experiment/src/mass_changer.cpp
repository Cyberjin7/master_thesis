#include "ros/ros.h"
#include "custom_ros_msgs/CustomData.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/MassChange.h"

bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

void syncCallback(const custom_ros_msgs::CustomData::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mass_changer");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<experiment_srvs::MassChange>("change_mass");
    ros::ServiceServer service = n.advertiseService("start_trigger", startCallback);
    ros::Subscriber subscriber = n.subscribe("q_sync", 100, syncCallback);

    ros::Rate loop(20);

    double mass_light, mass_normal, mass_heavy, time_min, time_max;
    ros::param::get("~mass/light", mass_light);
    ros::param::get("~mass/normal", mass_normal);
    ros::param::get("~mass/heavy", mass_heavy);
    ros::param::get("~time/min_length", time_min);
    ros::param::get("~time/max_length", time_max);

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}