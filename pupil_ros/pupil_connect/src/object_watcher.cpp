#include "ros/ros.h"
#include "pupil_msgs/fixation.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <sstream>

class ObjectEyer{
    private:
        std::vector<double> gaze_pos;
        std::array<double, 2> test;
        int64_t xmin;
        int64_t xmax;
        int64_t ymin;
        int64_t ymax;
    public:
        ObjectEyer();
        void fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation);
        void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box);
};

ObjectEyer::ObjectEyer()
{
    gaze_pos.push_back(0.0);
    gaze_pos.push_back(0.0);
    test.fill(0.0);
    xmin = 0;
    xmax = 0;
    ymin = 0;
    ymax = 0;
}

void ObjectEyer::fixationCallback(const pupil_msgs::fixation::ConstPtr& fixation)
{
    gaze_pos[0] = fixation->norm_pos[0]*1200;
    gaze_pos[1] = 720*( 1 - fixation->norm_pos[1]);
    ROS_INFO("x: %f", gaze_pos[0]);
    ROS_INFO("y: %f", gaze_pos[1]);
}

void ObjectEyer::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes)
{
    ROS_INFO("Object identified: ");
    for (auto box : boxes->bounding_boxes)
    {
        ROS_INFO("Object: %s", box.Class.data());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_chooser");

    ros::NodeHandle n;

    ObjectEyer eyer;

    ros::Subscriber fixation_sub = n.subscribe("pupil_fixation", 1, &ObjectEyer::fixationCallback, &eyer);
    ros::Subscriber object_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &ObjectEyer::objectCallback, &eyer);

    ros::spin();

    return 0;
}