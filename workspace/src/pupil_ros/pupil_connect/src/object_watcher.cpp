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
#include "pupil_msgs/hand_pos.h"
#include "pupil_msgs/hand.h"
#include <cmath>

#include <sstream>

class ObjectEyer{
    private:
        std::vector<double> gaze_pos;
        std::vector<geometry_msgs::Point> landmarks;
        std::vector<darknet_ros_msgs::BoundingBox> boxes;
        int64_t xmin;
        int64_t xmax;
        int64_t ymin;
        int64_t ymax;
        ros::ServiceClient mass_client;
        ros::Publisher target_pub;
        ros::Publisher mass_change_pub;
        ros::Publisher hold_pub;
        std::map<std::string, double> object_mass_list;
        double vision_threshold;
        bool hold;
        darknet_ros_msgs::BoundingBox target_box;
        experiment_srvs::MassChange change_mass_request;
        sync_msgs::MassTrial mass_change;
        std::vector<double> hand_pos;
        std::vector<double> wrist_pos;
        std::vector<double> mft_pos; // middle finger tip 
        darknet_ros_msgs::BoundingBox holding_box;
        bool gaze;
        bool object;
        bool hand;
    public:
        ObjectEyer(ros::NodeHandle handle, std::map<std::string, double> object_list);
        void fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation);
        void gazeCallback(const pupil_msgs::gaze::ConstPtr& gaze);
        void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box);
        void sendMass(std::vector<double> gaze_norm_pos);
        void emgCallback(const std_msgs::Float64::ConstPtr& activity);
        void handCallback(const pupil_msgs::hand_pos::ConstPtr& hand_pos);
        void handAllCallback(const pupil_msgs::hand::ConstPtr& hand_all);
        void mass_request(std::string target_object);
        void mass_request(double target_mass);
        void set_threshold(double threshold_value);
        void assess_hold();
        bool initialized();
        bool within_x(double xmin, double xmax, double value);
        bool within_y(double ymin, double ymax, double value);
        void update_hold();
};

ObjectEyer::ObjectEyer(ros::NodeHandle handle, std::map<std::string, double> object_list)
{
    this->gaze_pos.push_back(0.0);
    this->gaze_pos.push_back(0.0);
    this->xmin = 0;
    this->xmax = 0;
    this->ymin = 0;
    this->ymax = 0;

    this->hand_pos.push_back(0.0);
    this->hand_pos.push_back(0.0);
    this->wrist_pos.push_back(0.0);
    this->wrist_pos.push_back(0.0);
    this->mft_pos.push_back(0.0);
    this->mft_pos.push_back(0.0);

    this->boxes.clear();

    this->object_mass_list = object_list;

    this->target_pub = handle.advertise<std_msgs::Float64>("target", 60);
    this->hold_pub = handle.advertise<darknet_ros_msgs::BoundingBox>("holding_object", 60);
    this->mass_change_pub = handle.advertise<sync_msgs::MassTrial>("mass_change", 60);
    this->mass_client = handle.serviceClient<experiment_srvs::MassChange>("change_mass_request");

    this->hold = false;
    this->gaze = false;
    this->object = false;
    this->hand = false;
}

void ObjectEyer::sendMass(std::vector<double> gaze_pos)
{
    // std::string object;
    // for (auto box: this->boxes){
    //     if ((box.xmin <= gaze_pos[0]) && (gaze_pos[0] <= box.xmax) && (box.ymin <= gaze_pos[1]) && (gaze_pos[1] <= box.ymax)){
    //         // ROS_INFO("Gaze is on: %s", box.Class.data());
    //         this->target_box = box;
    //     }
    // }
    
    double x_center;
    double y_center; 
    double prev_distance = this->vision_threshold; 
    double current_distance = prev_distance;
    // this->target_box = this->boxes[0];
    for(int i = 0; i < this->boxes.size(); ++i){
        x_center = (this->boxes[i].xmax + this->boxes[i].xmin)/2.0;
        y_center = (this->boxes[i].ymax + this->boxes[i].ymin)/2.0;
        current_distance = sqrt(pow(x_center - gaze_pos[0],2) + pow(y_center - gaze_pos[1],2));
        if(current_distance < prev_distance){
            this->target_box = this->boxes[i];
            prev_distance = current_distance;
        }
    }

    // ROS_INFO_STREAM("Target object is: " << this->target_box.Class.data());

    std_msgs::Float64 msg;
    msg.data = this->object_mass_list[this->target_box.Class.data()];
    this->target_pub.publish(msg);
}

void ObjectEyer::gazeCallback(const pupil_msgs::gaze::ConstPtr& gaze)
{
    if(!this->gaze){this->gaze = true;}
    this->gaze_pos[0] = gaze->norm_pos.x*1200;
    this->gaze_pos[1] = 720*(1 - gaze->norm_pos.y);
    // ROS_INFO("x: %f", this->gaze_pos[0]);
    // ROS_INFO("y: %f", this->gaze_pos[1]);

    sendMass(this->gaze_pos);
}

void ObjectEyer::fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation)
{
    if(!this->gaze){this->gaze = true;}
    this->gaze_pos[0] = fixation->norm_pos[0]*1200;
    this->gaze_pos[1] = 720*( 1 - fixation->norm_pos[1]);
    // ROS_INFO("x: %f", this->gaze_pos[0]);
    // ROS_INFO("y: %f", this->gaze_pos[1]);

    sendMass(this->gaze_pos);
}

void ObjectEyer::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes)
{
    // Initialization: Once a bounding box has been received for the first time, set object to true
    // Only if object, gaze and hand are true, then hold detection is assessed
    if(!this->object){this->object = true;}

    // Clears previous list of detected objects
    if (!this->boxes.empty()){
        this->boxes.clear();
    }

    // Iterate through all identified objects
    // If the objects are not person, keyboard and tvmonitor, then add to list of detected object
    // These objects were excluded as they are often in the view environment but we would never hold these hence only getting in the way of 
    // detection of other objects
    for (auto box : boxes->bounding_boxes)
    {
        // ROS_INFO("Object: %s", box.Class.data());
        // ROS_INFO("x_min: %" PRId64, box.xmin);
        // ROS_INFO("x_max: %" PRId64, box.xmax);
        // ROS_INFO("y_min: %" PRId64, box.ymin);
        // ROS_INFO("y_max: %" PRId64, box.ymax);
        if(box.Class.compare("person") != 0 && box.Class.compare("keyboard") != 0 && box.Class.compare("tvmonitor") != 0){
            this->boxes.push_back(box);
        }
    }

}

/**
 * Updates the bounding box coordinates of the object that is being held
*/
void ObjectEyer::update_hold()
{
    // Iterate through all received bounding boxes
    for (auto box: this->boxes){
        // If the class of the current bounding box is the same as the class of the held object and it is being held: then update coordinates
        if(box.Class.compare(this->holding_box.Class) == 0 && this->hold){
            this->holding_box.xmax = box.xmax;
            this->holding_box.xmin = box.xmin;
            this->holding_box.ymax = box.ymax;
            this->holding_box.ymin = box.ymin;
            break;
        }
    }
}

void ObjectEyer::emgCallback(const std_msgs::Float64::ConstPtr& activity)
{
    this->mass_request(this->target_box.Class.data());
}

void ObjectEyer::handCallback(const pupil_msgs::hand_pos::ConstPtr& hand_pos)
{
    if(!this->hand){this->hand = true;}

    this->wrist_pos[0] = hand_pos->wrist_pos.x*1200;
    this->wrist_pos[1] = hand_pos->wrist_pos.y*720;
    this->mft_pos[0] = hand_pos->mft_pos.x*1200;
    this->mft_pos[1] = hand_pos->mft_pos.y*720;
}

void ObjectEyer::handAllCallback(const pupil_msgs::hand::ConstPtr& hand_all)
{
    if(!this->hand){this->hand = true;}
    this->landmarks.clear();
    // this->landmarks = hand_all->landmark;
    for (auto landmark: hand_all->landmark){
        landmark.x = landmark.x*1200;
        landmark.y = landmark.y*720;
        this->landmarks.push_back(landmark);
    }
}

// TODO: both within_x and within_y not necessary. They both do the same thing. 
/**
 * Assess whether 'value' falls within 'xmin' and 'xmax' boundary values
 * @param xmin minimum value
 * @param xmax maximum value
 * @param value value to assess
 * @return true if value is between xmin and xmax
*/
bool ObjectEyer::within_x(double xmin, double xmax, double value)
{
    return ((value >= xmin)&&(value <= xmax));
}

/**
 * Assess whether 'value' falls within 'xmin' and 'xmax' boundary values
 * @param ymin minimum value
 * @param ymax maximum value
 * @param value value to assess
 * @return true if value is between ymin and ymax
*/
bool ObjectEyer::within_y(double ymin, double ymax, double value)
{
    return ((value >= ymin)&&(value <= ymax));
}


void ObjectEyer::assess_hold()
{
    if(!this->hold){
        if(within_x(this->target_box.xmin, this->target_box.xmax, this->landmarks[9].x) && within_y(this->landmarks[12].y, this->landmarks[0].y, this->target_box.ymax)){
            ROS_INFO_STREAM("Holding " << this->target_box.Class.data());
            this->hold = true;
            this->holding_box = this->target_box;
            this->mass_request(this->target_box.Class.data());
            this->hold_pub.publish(this->target_box);
        }

    }
    else{
        if(!within_y(this->landmarks[12].y, this->landmarks[0].y, this->holding_box.ymax)){
            ROS_INFO_STREAM("Not holding");
            this->hold = false;
            this->mass_request(0.0);

            this->target_box.xmax = 0.0;
            this->target_box.xmin = 0.0;
            this->target_box.ymax = 0.0;
            this->target_box.ymin = 0.0;
            this->target_box.Class.clear();
            this->target_box.Class.assign("none");
            this->hold_pub.publish(this->target_box);
        }
    }
}

/**
 * Fail safe to prevent undefined behavior. Returns true if hand, gaze and bounding box data has been received at least once for each subscriber
 * Undefined behavior in the beginning when hand and bounding box data is not necessarily available yet
 * @return true if hand, gaze, object are true, else false. 
*/
bool ObjectEyer::initialized()
{
    return (this->hand && this->gaze && this->object);
}

void ObjectEyer::mass_request(std::string target_object)
{
    if(this->object_mass_list.find(target_object) == this->object_mass_list.end()){
        ROS_WARN_STREAM("No mass for this object");
    }
    else{
        this->change_mass_request.request.mass.data = this->object_mass_list[target_object];
        this->mass_client.call(this->change_mass_request);
        this->mass_change.header.stamp = ros::Time::now();
        this->mass_change.object = target_object;
        this->mass_change.mass = this->object_mass_list[target_object];
        this->mass_change_pub.publish(this->mass_change);
    }
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

/**
 * Sets distance threshold value. Distance between centroid of bounding box and gaze must be below this threshold to be detected as gaze target
 * @param threshold_value Distance threshold
*/
void ObjectEyer::set_threshold(double threshold_value)
{
    this->vision_threshold = threshold_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_chooser");

    ros::NodeHandle n;

    // Loads list of objects with their corresponding mass
    std::map<std::string, double> object_list;
    ros::param::get("~objects", object_list);

    // Create class instance
    ObjectEyer eyer(n, object_list);

    // Load variables from launch file
    bool fixation;
    ros::param::get("~fixation", fixation);
    bool emg;
    ros::param::get("~emg", emg);
    bool vision;
    ros::param::get("~vision", vision);

    // Load minimum distance for object gaze detection
    double threshold;
    ros::param::get("~vision_threshold", threshold);
    eyer.set_threshold(threshold);

    // Subscribe to either fixation or gaze data based on variable in launch file
    ros::Subscriber eye_sub;
    if(fixation){
        eye_sub = n.subscribe("pupil_fixation", 1, &ObjectEyer::fixationCallback, &eyer);
    }
    else{
        eye_sub = n.subscribe("pupil_gaze", 60, &ObjectEyer::gazeCallback, &eyer);
    }

    // Subscribe to bounding boxes
    ros::Subscriber object_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &ObjectEyer::objectCallback, &eyer);


    /*
    Choose between EMG or mediapipe to predict object grasping
    */

    // Fail safe to prevent both from being used at the same time
    if(emg && vision){
        ROS_WARN_STREAM("Cannot use EMG and mediapipe hand tracking together. Please choose one");
        ros::shutdown();
    }

    ros::Subscriber emg_activity;
    ros::Subscriber hand_tracker;
    if(emg){
        emg_activity = n.subscribe("emg_activity_thresh", 1, &ObjectEyer::emgCallback, &eyer);
    }
    if(vision){
        hand_tracker = n.subscribe("hand", 60, &ObjectEyer::handAllCallback, &eyer);
    }


    ros::Rate r(60);

    while(ros::ok()){
        // If gaze data, hand data and object data has been received and vision is being used for hold detection:
        // Update coordinates of held object if applicable
        // Assess whether the object is still being held or if a new object is being held while not holding an object
        if(eyer.initialized() && vision){
            eyer.update_hold();
            eyer.assess_hold();
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}