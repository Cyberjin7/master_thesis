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
            double interval_duration;
            double wait_duration;
            double t;
            std::vector<double> values;
            std::map<double, double> cal_angles;
        public:
            Calibration(double interval_angle, double interval_duration);

            bool get_start();
            bool get_record();
            double get_interval_angle();
            double get_interval_duration();
            double get_wait_duration();
            double get_t();

            void calibrate();
            double interp();
    };
}

#endif