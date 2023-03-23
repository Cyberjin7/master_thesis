#include "SyncPlayer/SyncPlayer.h"
namespace SyncPlayer{

    SyncPlayer::SyncPlayer(ros::NodeHandle handler, int time_delay, std::string traj_mode)
    {
        this->q_pub = handler.advertise<sync_msgs::CustomData>("q_sync", 100);
        this->q_ref_pub = handler.advertise<sync_msgs::CustomData>("q_ref_sync", 100);

        this->sync_state_pub = handler.advertise<sync_msgs::SyncQ>("state_sync", 100);

        this->toggle_mass = handler.serviceClient<experiment_srvs::Trigger>("toggle_mass");
        // this->toggle_recorder = handler.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
        this->play = false;
        this->delay = time_delay;
        this->node_start_time = ros::Time::now();
        this->time_diff = ros::Duration(0);
        this->q_sub = handler.subscribe("q", 100, &SyncPlayer::qCallback, this);
        this->state_sub = handler.subscribe("state", 100, &SyncPlayer::stateCallback, this);

        this->trajectory_mode = traj_mode;

        this->q_msg.value.data = 0.0;

        this->traj_gen = TrajGen();

        if (traj_mode == "BAG"){
            this->toggle_recorder = handler.serviceClient<experiment_srvs::Trigger>("toggle_recorder");
        }

    }

    SyncPlayer::~SyncPlayer()
    {
        
    }

    // make this external callback? Subscriber as well?
    void SyncPlayer::qCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        // this->synchronize();
        // custom_ros_msgs::CustomData sync_msg;
        // sync_msg.header.stamp = this->sync_time;
        this->q_msg.value.data = msg->data;
        // this->q_pub.publish(sync_msg);
    }

    void SyncPlayer::stateCallback(const exo_msgs::state::ConstPtr& msg)
    {
        this->state_msg.state = *msg;
    }

    // Make this an external callback?
    bool SyncPlayer::toggleCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        ROS_INFO_STREAM("Received trigger");
        this->playToggle();


        // experiment_srvs::Trigger start_trigger;
        // start_trigger.request.trigger.data = this->play;
        // start_trigger.request.header.stamp = this->sync_time;

        // this->toggle_client.call(start_trigger);

        return true;
    }

    void SyncPlayer::playToggle()
    {
        if (!this->play)
        {
            this->play = true;
            this->traj_gen.startGen(this->calculateTime());
        }
        else{
            this->play = false;
        }
        // this->play = play;
        ROS_INFO_STREAM("Toggle is: " << this->play);
        experiment_srvs::Trigger start_trigger;
        start_trigger.request.trigger.data = this->play;
        start_trigger.request.header.stamp = this->sync_time;

        this->toggle_mass.call(start_trigger);
        if (this->trajectory_mode == "BAG"){
            this->toggle_recorder.call(start_trigger);
        }
    }

    void SyncPlayer::loadBag(std::string bag_path)
    {
        std::ifstream bag_file;
        bag_file.open(bag_path);
        if(!bag_file)
        {
            ROS_WARN_STREAM("Bag file doesn't exist.");
            throw rosbag::BagIOException("");
        }
        this->bag.open(bag_path, rosbag::bagmode::Read);
        this->traj_view.addQuery(this->bag);
        view_it = this->traj_view.begin();
        this->bag_it = 0;

    }

    ros::Time SyncPlayer::calculateTime()
    {
        ros::Time time_now = ros::Time::now();
        this->time_diff = time_now - this->node_start_time;
        time_now.sec = this->time_diff.sec;
        time_now.nsec = this->time_diff.nsec;
        return time_now;
    }

    ros::Time SyncPlayer::calculateFuture()
    {
        this->future_time = this->sync_time;
        this->future_time.sec += this->delay;
        return this->future_time;
    }

    void SyncPlayer::itBag()
    {
        // if (it < traj_vew.end()) does not seem to work. Below is not optimal nor best practice but is the next best option
            if (this->bag_it < this->traj_view.size())
            {
                this->q = (*(this->view_it)).instantiate<std_msgs::Float64>();
                if (this->q!= nullptr)
                {
                    this->ref_msg.value.data = this->q->data;
                }
                ++view_it;
                ++bag_it;
            }
            else
            {
                this->bag.close();
                //this->play = false;
                playToggle();
                ROS_WARN_STREAM("Finished playing trajectory!");
                this->ref_msg.value.data = 0;
            }
    }

    void SyncPlayer::synchronize()
    {
        this->sync_time = this->calculateTime();
        this->ref_msg.header.stamp = this->calculateFuture();
        if(this->play)
        {
            if (this->trajectory_mode == "BAG")
            {
                this->itBag();
            }
            else if (this->trajectory_mode == "GEN")
            {
                this->ref_msg.value.data = this->traj_gen.generate(this->sync_time);
            }
        }
        else
        {
            this->ref_msg.value.data = 0;
        }

        this->q_msg.header.stamp = this->sync_time;
        this->state_msg.Header.stamp = this->sync_time;

        this->q_ref_pub.publish(this->ref_msg);
        this->q_pub.publish(this->q_msg);
        this->sync_state_pub.publish(this->state_msg);
    }

}