#include "ros/ros.h"
#include "sync_msgs/CustomData.h"
#include "sync_msgs/SyncQ.h"
#include "sync_msgs/MassData.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/MassChange.h"
#include "MassChanger/MassChanger.h"

class TrialPhysical : public MassChanger
{
    public:
        TrialPhysical(ros::NodeHandle handle) : MassChanger(handle){}

        void syncCallback(const sync_msgs::SyncQ::ConstPtr &msg)
        {
            updateTime(msg->header.stamp);
            if(start){
                if(current_time - start_time > wait_time){
                    if(mass_iterator < mass_order.size()){
                        start_time = current_time;
                        changeMass();
                    }
                    else{
                        endExperiment();
                    }
                }
            }
        }

        void generateOrder() override
        {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine rng(seed);

            std::vector<std::string> order;
            for (auto const& imap: mass_list)
            {
                order.push_back(imap.first);
            }
            std::shuffle(std::begin(order), std::end(order), rng);

            // phys_list: 
            // physical_mass_trials: 
            std::vector<double> phys_list;
            for(auto it: order){
                for (int j = 0; j < trial_params["min"]; ++j){
                    phys_list.push_back(mass_list[it]);
                }
            }

            // TODO: change number of trials from trial_params["min"] to its own dedicated parameter in yaml file
            for(int i=0; i < order.size(); ++i){
                int trial = trial_params["min"]*order.size();
                mass_order.insert(mass_order.end(), trial, order[i]);
                mass_time.insert(mass_time.end(), trial, trial_params["length"]);
                ROS_INFO_STREAM("Number of trials for mass " << order[i] << ": " << trial);

                std::shuffle(std::begin(phys_list), std::end(phys_list), rng);
                physical_mass_trials.insert(physical_mass_trials.end(), phys_list.begin(), phys_list.end());
            }

            // For now: assuming number of trials for a physical mass can be divided by block_size
            int block_size = 3;
            if(physical_mass_trials.size()%block_size != 0){
                ROS_WARN_STREAM("Number of trials cannot be evenly divided into blocks!");
                ros::shutdown();
            }

            std::vector<int> block(physical_mass_trials.size()/block_size);
            for(int i=0;i<block.size(); ++i){
                block[i] = i;
            }

            std::shuffle(std::begin(block), std::end(block), rng);

            std::vector<std::string> new_mass_order;
            std::vector<int> new_mass_time; // this should be double tbh
            std::vector<double> new_physical_mass;
            for(auto it: block){
                // ROS_INFO_STREAM(""<<it);    
                int start_index = it*block_size;
                for(int i = start_index; i < start_index+block_size; ++i){
                    new_mass_order.push_back(mass_order[i]);
                    new_mass_time.push_back(mass_time[i]);
                    new_physical_mass.push_back(physical_mass_trials[i]);
                }
            }
            // ROS_INFO_STREAM("\n");
            // ROS_INFO_STREAM("For Mass Order: ");
            // for (int i = 0 ; i < mass_order.size(); ++i){
            //     ROS_INFO_STREAM("" << i << " Old: " << mass_order[i] << " New: " << new_mass_order[i]);
            // }
            // ROS_INFO_STREAM("For physical mass order: ");
            // for (int i = 0 ; i < physical_mass_trials.size(); ++i){
            //     ROS_INFO_STREAM("" << i << " Old: " << physical_mass_trials[i] << " New: " << new_physical_mass[i]);
            // }
            // ROS_INFO_STREAM("\n");

            mass_order = new_mass_order;
            mass_time = new_mass_time;
            physical_mass_trials = new_physical_mass;
        }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mass_changer");
    ros::NodeHandle n;

    int rate;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Rate: " << rate);
    ros::Rate loop(rate);

    TrialPhysical mass_changer(n);
    mass_changer.generateOrder();
    
    ros::Subscriber subscriber = n.subscribe("state_sync", 100, &TrialPhysical::syncCallback, &mass_changer);

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}