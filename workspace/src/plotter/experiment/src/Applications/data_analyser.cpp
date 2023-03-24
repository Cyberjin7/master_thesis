#include "ros/ros.h"
#include "sync_msgs/SyncQ.h"
#include "sync_msgs/response.h"

class Analyser
{
    private:
        double min;
        double max;
        double settling_threshold;
        double rising_threshold;
        double overshoot;
        ros::Time start_t;
        ros::Duration rising_t;
        bool start;
        ros::Publisher response_pub;
        sync_msgs::response response_msg;

    public:
        Analyser(ros::NodeHandle handler, double minimum, double maximum, double set_thresh, double rise_thresh);
        ~Analyser();
        void stateCallback(const sync_msgs::SyncQ::ConstPtr& msg);
        double calculateOvershoot(double value, double overshoot, double target);
};

Analyser::Analyser(ros::NodeHandle handler, double minimum, double maximum, double set_thresh, double rise_thresh)
{
    this->start = false;
    this->min = minimum;
    this->max = maximum;
    this->settling_threshold = set_thresh;
    this->rising_threshold = maximum*rise_thresh;

    this->response_pub = handler.advertise<sync_msgs::response>("response", 1);
}

Analyser::~Analyser()
{

}

void Analyser::stateCallback(const sync_msgs::SyncQ::ConstPtr& msg)
{
    if(msg->high && !this->start){
        ROS_INFO_STREAM("Analysing...");
        this->start = true;
        this->start_t = msg->header.stamp;
        this->overshoot = 0.0;
    }
    else if(!msg->high && this->start){
        ROS_INFO_STREAM("Done");
        this->start = false;
        this->response_msg.overshoot = this->overshoot;
        this->response_msg.rise_t = this->rising_t.toSec();
        this->response_msg.header.stamp = msg->header.stamp;
        this->response_pub.publish(this->response_msg);
    }

    if(this->start){
        this->overshoot = calculateOvershoot(msg->state.q_state.q, this->overshoot, this->max);

        if(msg->state.q_state.q >= this->rising_threshold){
            this->rising_t = msg->header.stamp - this->start_t;
        }
    }

}

double Analyser::calculateOvershoot(double value, double overshoot, double target)
{
    if(value - target > overshoot){
        return value - target;
    }
    else{
        return overshoot;
    }
}

void stateCallback(const sync_msgs::SyncQ::ConstPtr& msg, bool *start, ros::Time *start_t, ros::Duration *rising_t, double *overshoot)
{
    if(msg->high && !*start){
        ROS_INFO_STREAM("Peak");
        *start = true;
        *start_t = msg->header.stamp;
    }
    else if(!msg->high && *start){
        ROS_INFO_STREAM("Low");
        *start = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_analyzer");
    ros::NodeHandle n;

    int rate;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Rate: " << rate);

    ros::Rate loop(rate);

    double min;
    double max;
    ros::param::get("~gen/min", min);
    ros::param::get("~gen/max", max);

    double rise_threshold;
    double settling_threshold;
    ros::param::get("~analysis/rise_threshold", rise_threshold);
    ros::param::get("~analysis/settling_threshold", settling_threshold);

    bool start = false;

    ros::Time start_time;
    ros::Time current_time;
    ros::Duration rising_time = ros::Duration(0);
    double overshoot = 0;

    double q = 0;

    Analyser response_analyser(n, min, max, settling_threshold, rise_threshold);


    // ros::Subscriber state_sub = n.subscribe("state_sync", 100, stateCallback);
    ros::Subscriber state_sub = n.subscribe("state_sync", 200, &Analyser::stateCallback, &response_analyser);

    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}