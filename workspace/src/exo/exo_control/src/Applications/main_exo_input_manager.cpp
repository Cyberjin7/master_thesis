#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "input_manager");
    ros::NodeHandle n;
    ros::ServiceClient start_client = n.serviceClient<std_srvs::Empty>("cal_trigger");
    ros::ServiceClient pred_client = n.serviceClient<std_srvs::Empty>("predictive_toggle");
    ros::ServiceClient cheat_client = n.serviceClient<std_srvs::Empty>("cheat_toggle");
    ros::Publisher comp_pub = n.advertise<std_msgs::Float64>("external_kp", 1);

    std::string usr_input;
    std_srvs::Empty trigger;
    std_msgs::Float64 external_kp;

    ROS_INFO_STREAM("Press [ENTER] to start exoskeleton calibration");
    ROS_INFO_STREAM("Input a number and press [ENTER] to adjust external force compensation gain");
    ROS_INFO_STREAM("Press q or Q and then [ENTER] to quit node");

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
        else if(usr_input.compare("predictive") == 0 || usr_input.compare("p") == 0){
            ROS_WARN_STREAM("Toggle predictive");
            pred_client.call(trigger);
        }
        else if(usr_input.compare("cheat") == 0 || usr_input.compare("c") == 0){
            ROS_WARN_STREAM("Toggle cheat");
            cheat_client.call(trigger);
        }
        else{
            external_kp.data = std::atof(usr_input.c_str());
            ROS_INFO_STREAM("Sending: " << external_kp.data);
            comp_pub.publish(external_kp);
        }
    }

    return 0;
}