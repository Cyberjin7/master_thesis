#include "ros/ros.h"
#include "pupil_msgs/fixation.h"
#include "pupil_msgs/gaze.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <inttypes.h>
#include "std_msgs/Float64.h"
#include "experiment_srvs/MassChange.h"
#include "geometry_msgs/Point.h"
#include "sync_msgs/MassTrial.h"

#include <sstream>

class ObjectEyer{
    private:
        std::vector<double> gaze_pos;
        std::vector<darknet_ros_msgs::BoundingBox> boxes;
        int64_t xmin;
        int64_t xmax;
        int64_t ymin;
        int64_t ymax;
        ros::ServiceClient mass_client;
        ros::Publisher target_pub;
        ros::Publisher mass_change_pub;
        std::map<std::string, double> object_mass_list;
        double vision_threshold;
        bool hold;
        darknet_ros_msgs::BoundingBox target_box;
        experiment_srvs::MassChange change_mass_request;
        sync_msgs::MassTrial mass_change;
    public:
        ObjectEyer(ros::NodeHandle handle, std::map<std::string, double> object_list);
        void fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation);
        void gazeCallback(const pupil_msgs::gaze::ConstPtr& gaze);
        void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box);
        void sendMass(std::vector<double> gaze_norm_pos);
        void emgCallback(const std_msgs::Float64::ConstPtr& activity);
        void handCallback(const geometry_msgs::Point::ConstPtr& hand_pos);
        void mass_request(std::string target_object);
        void mass_request(double target_mass);
        void set_threshold(double threshold_value);
};

ObjectEyer::ObjectEyer(ros::NodeHandle handle, std::map<std::string, double> object_list)
{
    this->gaze_pos.push_back(0.0);
    this->gaze_pos.push_back(0.0);
    this->xmin = 0;
    this->xmax = 0;
    this->ymin = 0;
    this->ymax = 0;

    this->boxes.clear();

    this->object_mass_list = object_list;

    this->target_pub = handle.advertise<std_msgs::Float64>("target", 60);
    this->mass_change_pub = handle.advertise<sync_msgs::MassTrial>("mass_change", 60);
    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass_request");

    this->hold = false;
}

void ObjectEyer::sendMass(std::vector<double> gaze_pos)
{
    // std::string object;
    for (auto box: this->boxes){
        if ((box.xmin <= gaze_pos[0]) && (gaze_pos[0] <= box.xmax) && (box.ymin <= gaze_pos[1]) && (gaze_pos[1] <= box.ymax)){
            ROS_INFO("Gaze is on: %s", box.Class.data());
            this->target_box = box;
        }
    }
    std_msgs::Float64 msg;
    msg.data = this->object_mass_list[this->target_box.Class.data()];
    this->target_pub.publish(msg);
}

void ObjectEyer::gazeCallback(const pupil_msgs::gaze::ConstPtr& gaze)
{
    this->gaze_pos[0] = gaze->norm_pos.x*1200;
    this->gaze_pos[1] = 720*(1 - gaze->norm_pos.y);
    // ROS_INFO("x: %f", this->gaze_pos[0]);
    // ROS_INFO("y: %f", this->gaze_pos[1]);

    sendMass(this->gaze_pos);
}

void ObjectEyer::fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation)
{
    this->gaze_pos[0] = fixation->norm_pos[0]*1200;
    this->gaze_pos[1] = 720*( 1 - fixation->norm_pos[1]);
    // ROS_INFO("x: %f", this->gaze_pos[0]);
    // ROS_INFO("y: %f", this->gaze_pos[1]);

    sendMass(this->gaze_pos);
}

void ObjectEyer::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes)
{
    if (!this->boxes.empty()){
        this->boxes.clear();
    }

    for (auto box : boxes->bounding_boxes)
    {
        // ROS_INFO("Object: %s", box.Class.data());
        // ROS_INFO("x_min: %" PRId64, box.xmin);
        // ROS_INFO("x_max: %" PRId64, box.xmax);
        // ROS_INFO("y_min: %" PRId64, box.ymin);
        // ROS_INFO("y_max: %" PRId64, box.ymax);
        this->boxes.push_back(box);
    }

}

void ObjectEyer::emgCallback(const std_msgs::Float64::ConstPtr& activity)
{
    this->mass_request(this->target_box.Class.data());
}

void ObjectEyer::handCallback(const geometry_msgs::Point::ConstPtr& hand_pos)
{
    // have toggle variable for if bounding box is touching hand or not via threshold
    // if toggle is off and bounding box and hand position fall below threshold, toggle to on and publish mass of object
    // if toggle is on and bounding box and position go over threshold, toggle to off and publish mass of 0
    double pos_x = hand_pos->x*1200;
    double pos_y = hand_pos->y*720;
    if(!this->hold){
        if((pos_x >= this->target_box.xmin) && (pos_x <= this->target_box.xmax) && (pos_y <= this->target_box.ymax)){
            if(this->target_box.ymax - pos_y < this->vision_threshold){
                ROS_INFO_STREAM("Holding object");
                this->hold = true;
                this->mass_request(this->target_box.Class.data());
            }
        }
    }
    else{
        if((pos_y <= this->target_box.ymax)&&(this->target_box.ymax - pos_y > this->vision_threshold)){
            ROS_INFO_STREAM("Not holding object");
            this->hold = false;
            this->mass_request(0.0);
        }
    }
}

void ObjectEyer::mass_request(std::string target_object)
{
    this->change_mass_request.request.mass.data = this->object_mass_list[target_object];
    this->mass_client.call(this->change_mass_request);
    this->mass_change.header.stamp = ros::Time::now();
    this->mass_change.object = target_object;
    this->mass_change.mass = this->object_mass_list[target_object];
    this->mass_change_pub.publish(this->mass_change);
}

void ObjectEyer::mass_request(double target_mass)
{
    this->change_mass_request.request.mass.data = target_mass;
    this->mass_client.call(this->change_mass_request);
    this->mass_change.header.stamp = ros::Time::now();
    this->mass_change.object = "NONE";
    this->mass_change.mass = 0.0;
    this->mass_change_pub.publish(this->mass_change);
}

void ObjectEyer::set_threshold(double threshold_value)
{
    this->vision_threshold = threshold_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_chooser");

    ros::NodeHandle n;

    std::map<std::string, double> object_list;
    ros::param::get("~objects", object_list);

    ObjectEyer eyer(n, object_list);

    bool fixation;
    ros::param::get("~fixation", fixation);
    bool emg;
    ros::param::get("~emg", emg);
    bool vision;
    ros::param::get("~vision", vision);

    double threshold;
    ros::param::get("~vision_threshold", threshold);
    eyer.set_threshold(threshold);

    ros::Subscriber eye_sub;
    if(fixation){
        eye_sub = n.subscribe("pupil_fixation", 1, &ObjectEyer::fixationCallback, &eyer);
    }
    else{
        eye_sub = n.subscribe("pupil_gaze", 60, &ObjectEyer::gazeCallback, &eyer);
    }

    ros::Subscriber object_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &ObjectEyer::objectCallback, &eyer);

    if(emg && vision){
        ROS_WARN_STREAM("Cannot use EMG and mediapipe hand tracking together. Please choose one");
        ros::shutdown();
    }

    if(emg){
        ros::Subscriber emg_activity = n.subscribe("emg_activity_thresh", 1, &ObjectEyer::emgCallback, &eyer);
    }
    if(vision){
        ros::Subscriber hand_tracker = n.subscribe("hand_position", 60, &ObjectEyer::handCallback, &eyer);
    }


    ros::Rate r(60);

    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}