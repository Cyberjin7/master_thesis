#include<exo_control/exo_force_control.h>

namespace ExoControllers{

    ForceControl::ForceControl(double L2)
    {
        ROS_INFO_STREAM("Force Controller Created");
        
        std::string ns="~force_ctrl";
        std::stringstream s;
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("force m_kp: \n"<<m_kp);

        m_L2 = L2;
        m_startFlag = false;
        m_tao = 0;

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

        m_tao = m_L2*m_kp*(Ws - m_W_des); // to DO

        return m_tao;
    }

    void ForceControl::gCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        // pthread_mutex_lock( &this->count_mutex );
        m_gx = msg->data[0].acc[0] * cos(0.5236) + msg->data[0].acc[1] * sin(0.5236);
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

    void ForceControl::fUpCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        m_f_upper = msg->data[0].force[2];
        // ROS_INFO_STREAM("Callback f_upper: " << m_f_upper);
    }

    void ForceControl::fLowCallback(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr& msg)
    {
        m_f_lower = msg->data[0].force[2];
        // ROS_INFO_STREAM("Callback f_lower: " << m_f_lower);
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
