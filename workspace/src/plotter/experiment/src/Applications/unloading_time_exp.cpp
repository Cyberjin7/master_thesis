#include "ros/ros.h"
#include "experiment_srvs/Trigger.h"
#include "experiment_srvs/AngleChange.h"
#include "std_srvs/Empty.h"
#include "sync_msgs/MassTrial.h"
#include "sync_msgs/TrialType.h"
#include <random>

bool recordCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res, ros::ServiceClient *client, bool *start, ros::Time *time, bool *init)
{
    experiment_srvs::Trigger start_recording;
    start_recording.request.header.stamp = ros::Time::now();
    start_recording.request.trigger.data = true;
    client->call(start_recording); 
    *start = true;
    *time = ros::Time::now();
    *init = true;
    return true;
}

void sendAngle(ros::ServiceClient& client, std::string type, double angle)
{
    if(type.compare("rest") == 0){
        experiment_srvs::AngleChange change_request;
        change_request.request.angle.data = angle;
        client.call(change_request);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unloading_experiment");
    ros::NodeHandle n;

    std::vector<std::string> targets;
    std::map<std::string, double> objects;

    ros::param::get("~/targets", targets);
    ros::param::get("~/objects", objects);

    int trial_size;
    int block_size;

    ros::param::get("~trials", trial_size);
    ros::param::get("~block", block_size);

    double loading_time;
    double unloading_time;
    double resting_time;

    ros::param::get("~loading_time", loading_time);
    ros::param::get("~unloading_time", unloading_time);
    ros::param::get("~resting_time", resting_time);

    double start_position;
    ros::param::get("~start_position", start_position);

    std::vector<std::string> time_order;
    ros::param::get("~order", time_order);

    std::vector<double> times;

    for(auto order: time_order){
        if(!order.compare("load")){
            times.push_back(loading_time);
        }
        else if(!order.compare("unload")){
            times.push_back(unloading_time);
        }
        else if(!order.compare("rest")){
            times.push_back(resting_time);
        }
    }

    std::vector<std::string> trials;

    if(trial_size%block_size != 0){
        ROS_WARN_STREAM("Trial cannot be divided into equal blocks. Please adjust");
        ros::shutdown();
        return 0;
    }
    else{
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine rng(seed);

        std::vector<std::string> order;
        int blocks = trial_size/block_size;

        for(auto it:targets){
            order.insert(order.end(), blocks, it);
        }
        std::shuffle(std::begin(order), std::end(order), rng);

        for(auto block:order){
            trials.insert(trials.end(), block_size, block);
        }

        for(auto it:trials){
            ROS_INFO_STREAM("" << it);
        }
        
    }

    ros::Time start_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    bool start = false;
    bool init = false;
    // int trial_it = 0;
    std::vector<std::string>::iterator trial_it = trials.begin();
    int order_it = 0;
    // std::vector<double>::iterator order_it = times.begin();

    ros::ServiceClient record_client = n.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
    ros::ServiceServer start_server = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("toggle_experiment", boost::bind(recordCallback,_1,_2, &record_client, &start, &start_time, &init)); // making a class would be better
    ros::Publisher trial_pub = n.advertise<sync_msgs::MassTrial>("load_trial", 1);
    ros::Publisher type_pub = n.advertise<sync_msgs::TrialType>("load_type", 1);
    ros::ServiceClient angle_client = n.serviceClient<experiment_srvs::AngleChange>("change_angle_request");

    sync_msgs::MassTrial trial_msg;
    sync_msgs::TrialType type_msg;

    experiment_srvs::AngleChange angle_request;

    while(ros::ok()){
        current_time = ros::Time::now();
        if(start){
            if(init){
                trial_msg.header.stamp = start_time;
                trial_msg.mass = objects[*trial_it];
                trial_msg.object = *trial_it;

                type_msg.header.stamp = start_time;
                type_msg.type = time_order[order_it];

                // if(time_order[order_it].compare("rest") == 0){
                //     angle_request.request.angle.data = start_position;
                //     angle_client.call(angle_request);
                // }
                sendAngle(angle_client, time_order[order_it], start_position);

                trial_pub.publish(trial_msg);
                type_pub.publish(type_msg);

                init = false;
            }

            if(trial_it < trials.end()){
                // current_time = ros::Time::now();
                if(current_time - start_time >= ros::Duration(times[order_it])){
                
                    if(order_it < times.size()-1){
                        order_it++;
                    }
                    else{
                        order_it = 0;
                        trial_it++;
                        if(trial_it == trials.end()){
                            experiment_srvs::Trigger record_toggle;
                            record_toggle.request.header.stamp = current_time;
                            record_toggle.request.trigger.data = false;
                            record_client.call(record_toggle);
                            ros::shutdown();
                            break;
                        }
                        else{
                            trial_msg.header.stamp = current_time;
                            trial_msg.mass = objects[*trial_it];
                            trial_msg.object = *trial_it;
                            trial_pub.publish(trial_msg);
                        }
                        
                    }

                    sendAngle(angle_client, time_order[order_it], start_position);

                    type_msg.header.stamp = current_time;
                    type_msg.type = time_order[order_it];
                    type_pub.publish(type_msg);

                    start_time = current_time;
                }
            }

        }
        ros::spinOnce();
    }


    return 0;
}