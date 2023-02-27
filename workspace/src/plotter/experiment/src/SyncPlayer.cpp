#include "SyncPlayer/SyncPlayer.h"

SyncPlayer::SyncPlayer(ros::NodeHandle handler, int time_delay)
{
    this->q_pub = handler.advertise<custom_ros_msgs::CustomData>("q_sync", 100);
    this->q_ref_pub = handler.advertise<custom_ros_msgs::CustomData>("q_ref_sync", 100);
    this->toggle_client = handler.serviceClient<experiment_srvs::Trigger>("toggle_trigger");
    this->play = false;
    this->delay = time_delay;
    this->node_start_time = ros::Time::now();
    this->time_diff = ros::Duration(0);
    this->q_sub = handler.subscribe("q", 100, &SyncPlayer::qCallback, this);

}

SyncPlayer::~SyncPlayer()
{
    
}

// make this external callback? Subscriber as well?
void SyncPlayer::qCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->synchronize();
    custom_ros_msgs::CustomData sync_msg;
    sync_msg.header.stamp = this->sync_time;
    sync_msg.value.data = msg->data;
    this->q_pub.publish(sync_msg);
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
    }
    else{
        this->play = false;
    }
    // this->play = play;
    ROS_INFO_STREAM("Toggle is: " << this->play);
    experiment_srvs::Trigger start_trigger;
    start_trigger.request.trigger.data = this->play;
    start_trigger.request.header.stamp = this->sync_time;

    this->toggle_client.call(start_trigger);
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

void SyncPlayer::synchronize()
{
    this->sync_time = this->calculateTime();
    this->sync_msg.header.stamp = this->calculateFuture();
    if(this->play)
    {
        // if (it < traj_vew.end()) does not seem to work. Below is not optimal nor best practice but is the next best option
        if (this->bag_it < this->traj_view.size())
        {
            this->q = (*(this->view_it)).instantiate<std_msgs::Float64>();
            if (this->q!= nullptr)
            {
                this->sync_msg.value.data = this->q->data;
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
            this->sync_msg.value.data = 0;
        }
    }
    else
    {
        this->sync_msg.value.data = 0;
    }
    this->q_ref_pub.publish(this->sync_msg);
}