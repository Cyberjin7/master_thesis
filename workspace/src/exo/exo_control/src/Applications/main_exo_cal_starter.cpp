#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_start_client");
    ros::NodeHandle n;
    ros::ServiceClient start_client = n.serviceClient<std_srvs::Empty>("cal_trigger");

    std::string usr_input;
    std_srvs::Empty trigger;


    ROS_INFO_STREAM("Press [ENTER] to start exoskeleton calibration");
    while(ros::ok())
    {
        //std::cin >> usr_input;
        std::getline(std::cin, usr_input);
        if(usr_input.empty()){
            ROS_INFO_STREAM("Starting calibration");
            start_client.call(trigger);
            // ros::shutdown();
        }
        else if(usr_input.compare("q") == 0 || usr_input.compare("Q") == 0){
            ROS_WARN_STREAM("Quit node");
            ros::shutdown();
        }
    }

    return 0;
}