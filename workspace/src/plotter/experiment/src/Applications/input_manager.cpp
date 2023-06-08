#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/Trigger.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "input_manager");
    ros::NodeHandle n;
    ros::ServiceClient start_client = n.serviceClient<std_srvs::Empty>("toggle_experiment");

    std::string usr_input;
    ROS_INFO_STREAM("Press [ENTER] to start experiment");
    while(ros::ok()){
        std::getline(std::cin, usr_input);

        if(usr_input.empty()){
            std_srvs::Empty trigger;
            start_client.call(trigger);
            ROS_INFO_STREAM("Starting experiment");
            ros::shutdown();
        }
        else if(usr_input.compare("q") == 0 || usr_input.compare("Q") == 0){
            ROS_INFO_STREAM("Quiting input manager");
            ros::shutdown();
        }
    }
    return 0;
}