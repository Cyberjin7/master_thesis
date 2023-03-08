#ifndef EXO_ROBOT_CALIBRATION_H
#define EXO_ROBOT_CALIBRATION_H

#include <vector>
#include <map>
#include "ros/ros.h"

namespace ExoControllers
{
    class Calibration
    {
        private:
            bool start;
            bool record;
            double interval_angle;
            ros::Duration interval_duration;
            ros::Duration wait_duration;
            ros::Time t;
            std::vector<double> temp_vals; // container to store values during each calibration angle
            std::map<double, double> cal_values; // angles to calibrate at
            std::vector<double> cal_angles;
            int intv_iter;
        public:
            Calibration(double interval_angle, double interval_duration, double wait_duration, double angle_min, double angle_max);
            ~Calibration();

            bool get_start();
            bool get_record();
            double get_interval_angle();
            ros::Duration get_interval_duration();
            ros::Duration get_wait_duration();
            ros::Time get_t();
            std::vector<double> get_cal_angles();

            void set_start(bool toggle);

            double calibrate(double force);
            double interp_force(double q);
    };
}

#endif