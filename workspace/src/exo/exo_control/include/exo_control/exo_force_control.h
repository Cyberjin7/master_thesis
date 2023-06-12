#ifndef EXO_ROBOT_FORCECONTROL_H
#define EXO_ROBOT_FORCECONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tum_ics_skin_msgs/SkinCellDataArray.h>
#include "std_msgs/Float64.h"
// #include <geometry_msgs/Vector3.h>
#include <QVector>

using namespace Eigen;
using std::string;

namespace ExoControllers{
    
    class ForceControl{
        private:
            ros::NodeHandle n;
            double m_L2;
            double m_kp_up;
            double m_kp_down;
            bool m_startFlag;
            double m_W_des;
            double m_tao;
            double m_gx;
            double m_gy;
            double m_gz;
            double m_prox_upper;
            double m_prox_lower;
            double m_f_upper;
            double m_f_lower;
            double delta_t;
            double m_ki_up;
            double m_ki_down;
            double m_kd_up;
            double m_kd_down;
            double m_e;
            double m_prev_e;
            double m_e_int;
            // pthread_mutex_t count_mutex;
            ros::Publisher cell6;
            ros::Publisher cell7;
            ros::Publisher cell8;
            ros::Publisher cell11;
            ros::Publisher cell12;
            ros::Publisher cell13;
            ros::Publisher down_pub;
            ros::Publisher up_pub;
            
        public:
            ForceControl(double L2, ros::NodeHandle handle);
            ~ForceControl();
            bool init(double W_des);
            double update(double Ws);
            void gCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg);
            void pUpCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg);
            void pLowCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg);
            void fUpCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg);
            void fLowCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg);

            void downKpCallback(const std_msgs::Float64::ConstPtr& msg);
            void upKpCallback(const std_msgs::Float64::ConstPtr& msg);
            // void gCallback(const geometry_msgs::Vector3::ConstPtr& msg);
            double get_m_gx();
            double get_m_gy();
            double get_m_gz();
            double get_m_prox_upper();
            double get_m_prox_lower();
            double get_m_f_upper();
            double get_m_f_lower();

            QVector<double> downFilterreading;
            QVector<double> upFilterreading;
    };
}

#endif