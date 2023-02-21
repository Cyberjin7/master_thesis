#include "ros/ros.h"
#include "custom_ros_msgs/CustomData.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/MassChange.h"
#include "MassChanger/MassChanger.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "mass_changer");
    ros::NodeHandle n;

    MassChanger mass_changer(n);

    // double mass_light, mass_normal, mass_heavy, time_min, time_max;
    // ros::param::get("~mass/light", mass_light);
    // ros::param::get("~mass/normal", mass_normal);
    // ros::param::get("~mass/heavy", mass_heavy);
    // ros::param::get("~time/min_length", time_min);
    // ros::param::get("~time/max_length", time_max);

    ros::ServiceClient client = n.serviceClient<experiment_srvs::MassChange>("change_mass");
    ros::ServiceServer service = n.advertiseService("start_trigger", &MassChanger::startCallback, &mass_changer);
    ros::Subscriber subscriber = n.subscribe("q_sync", 100, &MassChanger::syncCallback, &mass_changer);

    ros::Rate loop(20);

    

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}