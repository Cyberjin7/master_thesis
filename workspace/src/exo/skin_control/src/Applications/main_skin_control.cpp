#include <ros/ros.h>
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>
#include <QApplication>
#include <QFileInfo>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <geometry_msgs/Vector3.h>

using namespace tum_ics_skin_descr;
using namespace Eigen;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "skin_control",
              ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(100);

    // ros::Publisher acc_pub = n.advertise<geometry_msgs::Vector3>("skin_acc", 1000);

    QApplication a(argc, argv);
    QString configFilePath = QString(argv[1]);
    QFileInfo fi(configFilePath);
    if(!fi.absoluteDir().exists())
    {
        qCritical("Invalid path '%s'",configFilePath.toLatin1().data());
        return -1;
    }

    Patch::TfMarkerDataPatches tfPatches;

    if(!tfPatches.loadFromParam(configFilePath,"~patch_list"))
    {
        return -1;
    }
    tfPatches.createDataConnections();
    tfPatches.enableDataConnctions();

    while(ros::ok())
    {
        /*
        Vector3d acc;
        acc << tfPatches.patch(0)->dataFromId(2).acc.at(0),
               tfPatches.patch(0)->dataFromId(2).acc.at(1),
               tfPatches.patch(0)->dataFromId(2).acc.at(2); // get grav vals
        ROS_WARN_STREAM(acc);
        */

        // geometry_msgs::Vector3 msg;
        // msg.x = tfPatches.patch(0)->dataFromId(2).acc.at(0);
        // msg.y = tfPatches.patch(0)->dataFromId(2).acc.at(1);
        // msg.z = tfPatches.patch(0)->dataFromId(2).acc.at(2);
        // acc_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    qDebug("exit");

    return 0;

}