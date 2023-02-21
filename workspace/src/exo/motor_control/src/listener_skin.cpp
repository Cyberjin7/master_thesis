#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tum_ics_skin_msgs/SkinCellDataArray.h>

void skinCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
{
  // ROS_INFO_STREAM("First cellId: " << msg->data[0].cellId);
  // ROS_INFO_STREAM("Second cellId: " << msg->data[1].cellId);
  // ROS_INFO_STREAM("Third cellId: " << msg->data[2].cellId);
  // ROS_INFO_STREAM("acc x: " << msg->data[0].acc[0]);
  // ROS_INFO_STREAM("acc y: " << msg->data[0].acc[1]);
  // ROS_INFO_STREAM("acc z: " << msg->data[0].acc[2]);
  // double acc_x_cal = msg->data[0].acc[0] * cos(0.5236) + msg->data[0].acc[1] * sin(0.5236);
  // double acc_y_cal = -msg->data[0].acc[0] * sin(0.5236) + msg->data[0].acc[1] * cos(0.5236);
  // double acc_z_cal = msg->data[0].acc[2];
  // ROS_INFO_STREAM("Calibrated acc x: " << msg->data[0].acc[0] * cos(0.5236) + msg->data[0].acc[1] * sin(0.5236)); //30deg = 0.5236rad
  // ROS_INFO_STREAM("Calibrated acc y: " << -msg->data[0].acc[0] * sin(0.5236) + msg->data[0].acc[1] * cos(0.5236));
  // ROS_INFO_STREAM("Calibrated acc z: " << msg->data[0].acc[2]);
  ROS_INFO_STREAM("prox: " << msg->data[0].prox[0]);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_skin");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("patch5", 1000, skinCallback);
  ros::spin();

  return 0;
}