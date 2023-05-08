#include "ros/ros.h"
#include "pupil_msgs/fixation.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <inttypes.h>

#include <sstream>

class ObjectEyer{
    private:
        std::vector<double> gaze_pos;
        std::vector<darknet_ros_msgs::BoundingBox> boxes;
        int64_t xmin;
        int64_t xmax;
        int64_t ymin;
        int64_t ymax;
        ros::Publisher fix_object_pub;
    public:
        ObjectEyer(ros::NodeHandle handle);
        void fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation);
        void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box);
};

ObjectEyer::ObjectEyer(ros::NodeHandle handle)
{
    this->gaze_pos.push_back(0.0);
    this->gaze_pos.push_back(0.0);
    this->xmin = 0;
    this->xmax = 0;
    this->ymin = 0;
    this->ymax = 0;

    this->boxes.clear();

    this->fix_object_pub = handle.advertise<darknet_ros_msgs::BoundingBoxes>("fixated_objects", 1);
}

void ObjectEyer::fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation)
{
    // std::vector<double>(fixation->norm_pos.data(), fixation->norm_pos.size()*sizeof(fixation->norm_pos[0]));
    // this->gaze_pos = {fixation->norm_pos[0]*1200, fixation->norm_pos[0]*1200};

    this->gaze_pos[0] = fixation->norm_pos[0]*1200;
    this->gaze_pos[1] = 720*( 1 - fixation->norm_pos[1]);
    // ROS_INFO("x: %f", this->gaze_pos[0]);
    // ROS_INFO("y: %f", this->gaze_pos[1]);

    darknet_ros_msgs::BoundingBoxes msg;

    for (auto box : this->boxes){
        if ((box.xmin <= this->gaze_pos[0]) && (this->gaze_pos[0] <= box.xmax) && (box.ymin <= this->gaze_pos[1]) && (this->gaze_pos[1] <= box.ymax)){
            ROS_INFO("Gaze is on: %s", box.Class.data());
            msg.bounding_boxes.push_back(box);
        }
    }
    this->fix_object_pub.publish(msg);
}

void ObjectEyer::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes)
{
    if (!this->boxes.empty()){
        this->boxes.clear();
    }

    // ROS_INFO("Object identified: ");
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_chooser");

    ros::NodeHandle n;

    ObjectEyer eyer(n);

    ros::Subscriber fixation_sub = n.subscribe("pupil_fixation", 1, &ObjectEyer::fixationCallback, &eyer);
    ros::Subscriber object_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &ObjectEyer::objectCallback, &eyer);

    ros::Publisher fix_object_pub = n.advertise<darknet_ros_msgs::BoundingBoxes>("fixated_objects", 1);

    ros::Rate r(30);

    // ros::spin();
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}