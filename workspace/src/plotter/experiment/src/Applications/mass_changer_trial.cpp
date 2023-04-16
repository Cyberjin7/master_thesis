#include "ros/ros.h"
#include "sync_msgs/CustomData.h"
#include "sync_msgs/SyncQ.h"
#include "std_srvs/Empty.h"
#include "experiment_srvs/MassChange.h"
#include "MassChanger/MassChanger.h"



class TrialMass : public MassChanger
{
    public:
        TrialMass(ros::NodeHandle handle) : MassChanger(handle){}

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
            // TODO: adapt as member function
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine rng(seed);
            std::uniform_int_distribution<int> uniform_dist(trial_params["min"], trial_params["max"]);

            std::vector<std::string> order;
            for (auto const& imap: mass_list)
            {
                order.push_back(imap.first);
            }
            std::shuffle(std::begin(order), std::end(order), rng);

            for(int i=0; i < order.size(); ++i){
                int trial = uniform_dist(rng);
                mass_order.insert(mass_order.end(), trial, order[i]);
                mass_time.insert(mass_time.end(), trial, trial_params["length"]);
                ROS_INFO_STREAM("Number of trials for mass " << order[i] << ": " << trial);
            }
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

    TrialMass mass_changer(n);
    mass_changer.generateOrder();

    ros::Subscriber subscriber = n.subscribe("state_sync", 100, &TrialMass::syncCallback, &mass_changer);    

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}