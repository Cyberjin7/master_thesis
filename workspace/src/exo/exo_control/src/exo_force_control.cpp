#include<exo_control/exo_force_control.h>
#include "std_msgs/Float64.h"

namespace ExoControllers{

    ForceControl::ForceControl(double L2, ros::NodeHandle handle)
    {
        ROS_INFO_STREAM("Force Controller Created");
        
        std::string ns="~force_ctrl";
        std::stringstream s;
        s.str("");
        s<<ns<<"/kp_up";
        ros::param::get(s.str(),m_kp_up);
        ros::param::get("~force_ctrl/kp_down", m_kp_down);
        ROS_WARN_STREAM("force m_kp_up: \n"<<m_kp_up);
        ROS_WARN_STREAM("force m_kp_down: \n"<<m_kp_down);

        ros::param::get("~force_ctrl/kd_up", m_kd_up);
        ros::param::get("~force_ctrl/kd_down", m_kd_down);
        double rate;
        ros::param::get("~rate", rate);
        delta_t = 1/rate;
        m_e = 0.0;
        m_prev_e = 0.0;
        m_e_int = 0.0;

        m_L2 = L2;
        m_startFlag = false;
        m_tao = 0;

        cell6 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell6", 10);
        cell7 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell7", 10);
        cell8 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell8", 10);
        cell11 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell11", 10);
        cell12 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell12", 10);
        cell13 = handle.advertise<tum_ics_skin_msgs::SkinCellData>("cell13", 10);
        down_pub = handle.advertise<std_msgs::Float64>("down_sensor", 10);
        up_pub = handle.advertise<std_msgs::Float64>("up_sensor", 10);

        upFilterreading = QVector<double>(10,0.0);
        downFilterreading = QVector<double>(10,0.0);

        // count_mutex = PTHREAD_MUTEX_INITIALIZER;
    }

    ForceControl::~ForceControl()
    {
    }

    bool ForceControl::init(double W_des)
    {
        m_W_des = W_des;
        m_startFlag = false;        
        return true;
    }

    double ForceControl::update(double Ws)
    {
        if(!m_startFlag)
        {
            m_startFlag = true;
        }
        m_prev_e = m_e;
        m_e = Ws - m_W_des;


        double e_d = (m_e - m_prev_e) / delta_t;
        m_e_int = m_e_int + delta_t*(m_e + m_prev_e)/2;

        // if (Ws >= 0)
        // {
        //     m_tao = m_L2*m_kp_up*(Ws - m_W_des);
        // }
        // else{
        //     m_tao = m_L2*m_kp_down*(Ws - m_W_des);
        // }
        if (Ws >= 0)
        {
            m_tao = m_L2*(m_kp_up*m_e + m_kd_up*e_d + m_ki_up*m_e_int);
        }
        else{
            m_tao = m_L2*(m_kp_down*m_e + m_kd_down*e_d + m_ki_down*m_e_int);
        }

        // m_tao = m_L2*m_kp*(Ws - m_W_des); // Update for new model

        return m_tao;
    }

    void ForceControl::gCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        // pthread_mutex_lock( &this->count_mutex );
        m_gx = (msg->data[0].acc[0] * cos(0.5236) + msg->data[0].acc[1] * sin(0.5236));
        m_gy = -msg->data[0].acc[0] * sin(0.5236) + msg->data[0].acc[1] * cos(0.5236);
        m_gz = msg->data[0].acc[2];
        // pthread_mutex_unlock( &this->count_mutex );
        // ROS_INFO_STREAM("Callback calibrated acc: x: " << m_gx << ", y: " << m_gy <<", z: " << m_gz);
    }

    void ForceControl::pUpCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        m_prox_upper = msg->data[0].prox[0];
        // ROS_INFO_STREAM("Callback prox_upper: " << m_prox_upper);
    }

    void ForceControl::pLowCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        m_prox_lower = msg->data[0].prox[0];
        // ROS_INFO_STREAM("Callback prox_lower: " << m_prox_lower);
    }

    void ForceControl::fUpCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg) // use cellID: 8
    {
        // cellID: 8, 7, 6
        // cell 8: all sensors increase on extension. drop on flexion (when extended). 2 is strongest. when flexed, increase when contraction. very strong
        m_f_upper = msg->data[0].force[2];
        // ROS_INFO_STREAM("Callback f_upper: " << m_f_upper);
        for (auto cell: msg->data)
        {
            if(cell.cellId == 6)
            {
                cell6.publish(cell);
            }
            else if(cell.cellId == 7)
            {
                cell7.publish(cell);
            }
            else if(cell.cellId == 8)
            {
                cell8.publish(cell);
                double gain = 4.350802359e+02;
                upFilterreading[0] = upFilterreading[1];
                upFilterreading[1] = upFilterreading[2];
                upFilterreading[2] = upFilterreading[3];
                upFilterreading[3] = upFilterreading[4];
                upFilterreading[4] = (cell.force[0] + cell.force[1] + cell.force[2])/gain;

                upFilterreading[5] = upFilterreading[6];
                upFilterreading[6] = upFilterreading[7];
                upFilterreading[7] = upFilterreading[8];
                upFilterreading[8] = upFilterreading[9];
                upFilterreading[9] = (upFilterreading[0] + upFilterreading[4]) 
                                        + 4 * (upFilterreading[1] + upFilterreading[3]) + 6 * upFilterreading[2]
                                        + ( -0.2615609657 * upFilterreading[5]) + (  1.3908552184 * upFilterreading[6])
                                        + ( -2.8482276388 * upFilterreading[7]) + (  2.6821585601 * upFilterreading[8]);
                
                double up_filtered = upFilterreading[9];
                std_msgs::Float64 force_msg;
                force_msg.data = up_filtered;
                up_pub.publish(force_msg);
            }
        }
    }

    void ForceControl::fLowCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg) // use cellID: 13
    {
        // cellID: 13, 12, 11
        // 12 is useless
        // 13.2: drop when extending, increase on flexion (when extended) (0 is similar). Increase when flexing when flexed. Best. use this
        // 13.1: drop on flexion. increase on extension (when extended) (nearly same as 8.2 but more sensitive). Not useful when flexed
        // 11: very small changes
        // contracctio only slightly visible
        // 8.2 and 13.2 most useful overall
        // 13.0 and 8.0?
        m_f_lower = msg->data[0].force[0]; // 2 is most sensitive but not in the best position. Trying out 0
        // ROS_INFO_STREAM("Callback f_lower: " << m_f_lower);
        for (auto cell: msg->data)
        {
            if(cell.cellId == 11)
            {
                cell11.publish(cell);
            }
            else if(cell.cellId == 12)
            {
                cell12.publish(cell);
            }
            else if(cell.cellId == 13) // use this
            {
                cell13.publish(cell);
                double gain = 4.350802359e+02;
                downFilterreading[0] = downFilterreading[1];
                downFilterreading[1] = downFilterreading[2];
                downFilterreading[2] = downFilterreading[3];
                downFilterreading[3] = downFilterreading[4];
                downFilterreading[4] = (cell.force[0] + cell.force[1] + cell.force[2])/gain;

                downFilterreading[5] = downFilterreading[6];
                downFilterreading[6] = downFilterreading[7];
                downFilterreading[7] = downFilterreading[8];
                downFilterreading[8] = downFilterreading[9];
                downFilterreading[9] = (downFilterreading[0] + downFilterreading[4]) 
                                        + 4 * (downFilterreading[1] + downFilterreading[3]) + 6 * downFilterreading[2]
                                        + ( -0.2615609657 * downFilterreading[5]) + (  1.3908552184 * downFilterreading[6])
                                        + ( -2.8482276388 * downFilterreading[7]) + (  2.6821585601 * downFilterreading[8]);
                
                double down_filtered = downFilterreading[9];
                std_msgs::Float64 force_msg;
                force_msg.data = down_filtered;
                down_pub.publish(force_msg);
            }

        }
    }

    // void ForceControl::gCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    // {
    //     // pthread_mutex_lock( &this->count_mutex );
    //     // m_gx = msg->x * cos(0.5236) + msg->y * sin(0.5236);
    //     // m_gy = -msg->x * sin(0.5236) + msg->y * cos(0.5236);
    //     // m_gz = msg->z;
    //     m_gx = msg->x;
    //     m_gy = msg->y;
    //     m_gz = msg->z;
    //     // pthread_mutex_unlock( &this->count_mutex );
    //     ROS_INFO_STREAM("Callback acc: x: " << m_gx << ", y: " << m_gy <<", z: " << m_gz);
    // }

    double ForceControl::get_m_gx()
    {
        // double local;
        // pthread_mutex_lock( &this->count_mutex );
        // local=this->m_gx;
        // pthread_mutex_unlock( &this->count_mutex );
        // return local;
        return m_gx;
    }

    double ForceControl::get_m_gy()
    {
        // double local;
        // pthread_mutex_lock( &this->count_mutex );
        // local=this->m_gy;
        // pthread_mutex_unlock( &this->count_mutex );
        // return local;
        return m_gy;
    }

    double ForceControl::get_m_gz()
    {
        // double local;
        // pthread_mutex_lock( &this->count_mutex );
        // local=this->m_gz;
        // pthread_mutex_unlock( &this->count_mutex );
        // return local;
        return m_gz;
    }

    double ForceControl::get_m_prox_upper()
    {
        return m_prox_upper;
    }

    double ForceControl::get_m_prox_lower()
    {
        return m_prox_lower;
    }

    double ForceControl::get_m_f_upper()
    {
        return m_f_upper;
    }

    double ForceControl::get_m_f_lower()
    {
        return m_f_lower;
    }
}
