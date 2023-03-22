#include "exo_control/exo_calibration.h"
#include <numeric>

namespace ExoControllers{

    Calibration::Calibration(double interval_angle, double interval_duration, double wait_duration, double angle_min, double angle_max){
        this->start = false;
        this->record = false;

        this->interval_angle = interval_angle;
        this->interval_duration = ros::Duration(interval_duration);
        this->wait_duration = ros::Duration(wait_duration);

        int intervals = std::ceil((angle_max - angle_min)/interval_angle); // number of intervals
        this->cal_values.clear();
        this->cal_angles.clear();

        for (int i=0; i < intervals; ++i)
        {
            this->cal_angles.push_back(angle_min + interval_angle*i);
            this->cal_values.insert({this->cal_angles[i], 0.0});
        }
        this->cal_angles.push_back(angle_max);
        this->cal_values.insert({angle_max, 0.0}); // in case it does not divide fully, add max angle as additional calibration value

        this->intv_iter = 0;

    }

    Calibration::~Calibration(){}

    double Calibration::calibrate(double force){
        if(!this->record){
            if(ros::Time::now() - t > this->wait_duration){
                this->record = true;
                ROS_INFO_STREAM("Wait over");
                ROS_INFO_STREAM("Recording... ");
                t = ros::Time::now();
            }
        }
        else{
            if(ros::Time::now() - t > this->interval_duration){
                auto const count = static_cast<float>(this->temp_vals.size());
                ROS_INFO_STREAM("Average Force is: " << std::reduce(this->temp_vals.begin(), this->temp_vals.end())/count);
                this->cal_values.insert_or_assign(this->cal_angles[this->intv_iter], std::reduce(this->temp_vals.begin(), this->temp_vals.end())/count);
                ROS_INFO_STREAM("Finished recording for: " << this->cal_angles[this->intv_iter]);
                ROS_INFO_STREAM("Value is: " << this->cal_values[this->cal_angles[this->intv_iter]]);
                if (this->intv_iter < this->cal_angles.size()){
                    ++(this->intv_iter);
                    if(this->intv_iter != this->cal_angles.size()){
                        ROS_INFO_STREAM("Next angle: " << this->cal_angles[this->intv_iter]);
                    }
                    else{
                        ROS_INFO_STREAM("Calibration finished");
                        this->start = false;
                        this->intv_iter = 0;
                    }
                    t = ros::Time::now();
                }
                this->record = false;
                this->temp_vals.clear();

            }
            else{
                this->temp_vals.push_back(force);
            }
        }
        return this->cal_angles[this->intv_iter];
    }

    double Calibration::interp_force(double q){
        // ROS_INFO_STREAM("q: " << q);
        int i;
        for (i = 0; i < this->cal_angles.size(); ++i)
        {
            // The overlap in angles is intended. 
            if((q >= this->cal_angles[i]) && (q <= this->cal_angles[i+1])){
                break;
            }
        }
        double q_i = this->cal_angles[i];
        double q_i1 = this->cal_angles[i+1];
        double force = (q - q_i)*(this->cal_values[q_i1] - this->cal_values[q_i])/(q_i1 - q_i) + this->cal_values[q_i];
        // ROS_INFO_STREAM("Interp: " << force);
        return force;
    }

    bool Calibration::get_record(){
        return this->record;
    }

    bool Calibration::get_start(){
        return this->start;
    }

    double Calibration::get_interval_angle(){
        return this->interval_angle;
    }

    ros::Duration Calibration::get_interval_duration(){
        return this->interval_duration;
    }

    ros::Duration Calibration::get_wait_duration(){
        return this->wait_duration;
    }

    ros::Time Calibration::get_t(){
        return this->t;
    }

    std::vector<double> Calibration::get_cal_angles(){
        return this->cal_angles;
    }

    void Calibration::set_start(bool toggle)
    {
        this->start = toggle;
    }
}