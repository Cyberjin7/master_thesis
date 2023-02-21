#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include <exo_control/exo_force_control.h>
#include <exo_control/q.h>

// #include <pthread.h>

double deg2rad(double degree) {
    return (degree * 3.14159265359 / 180);
}


void checkRange(double& q1, double& qd1, double& qdd1) {
    if (q1 > deg2rad(100) || q1 < deg2rad(10)) {
        if (q1 > deg2rad(100)) q1 = deg2rad(100); 
        if (q1 < deg2rad(10))  q1 = deg2rad(10);
        qd1 = 0;
        qdd1 = 0;
    }
}

int main(int argc, char** argv)
{
    int f = 200;
    ros::init(argc, argv, "exo_control",
        ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher exo_pub = n.advertise<std_msgs::Float64>("/q_control_publisher", 10);
    // ros::Publisher pub_q_state = n.advertise<exo_control::q>("/q_state", 1);
    ros::Publisher pub_q_state = n.advertise<std_msgs::Float64>("/q_state", 1);
    // ros::Publisher pub_q_des = n.advertise<exo_control::q>("/q_des", 1);
    // ros::Publisher pub_q_intention = n.advertise<std_msgs::Float64>("/q_intention", 1);
    ros::Rate r(f);

    std_msgs::Float64 q_state;
    // exo_control::q q_state;
    exo_control::q q_des;

    double delta_t = 1 / (double)f;

    // load your params
    double L1;
    std::string ns = "~L1";
    std::stringstream s;
    s.str("");
    s << ns;
    ros::param::get(s.str(), L1);
    double L2;
    ns = "~L2";
    s.str("");
    s << ns;
    ros::param::get(s.str(), L2);
    double m2;
    ns = "~m2";
    s.str("");
    s << ns;
    ros::param::get(s.str(), m2);
    double b1;
    ns = "~b1";
    s.str("");
    s << ns;
    ros::param::get(s.str(), b1);
    double k1;
    ns = "~k1";
    s.str("");
    s << ns;
    ros::param::get(s.str(), k1);
    double theta1;
    ns = "~theta1";
    s.str("");
    s << ns;
    ros::param::get(s.str(), theta1);
    double I233;
    ns = "~I233";
    s.str("");
    s << ns;
    ros::param::get(s.str(), I233);
    double g;
    ns = "~g";
    s.str("");
    s << ns;
    ros::param::get(s.str(), g);

    double tao = 0;

    //init params 
    //double q1 = deg2rad(90);
    double q1 = deg2rad(90);
    double qd1 = 0;
    double qdd1 = 0;

    // static g 
    double gx = g;
    double gy = 0;
    double gz = 0;

    double m_matrix;
    double c_matrix;
    double g_matrix = 0;
    double b_matrix;

    // Initialize PosControl object
     ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx, gy);
     Vector3d qEnd;
     qEnd << deg2rad(45),0.0,0.0;
     double timeEnd = 5;
     posControl.init(qEnd,timeEnd);

    //load force control
    ExoControllers::ForceControl forceControl(L2);
    double W_des = 0;
    double Ws = 0;

    forceControl.init(W_des);
    ros::Subscriber sub_g = n.subscribe("patch1", 1000, &ExoControllers::ForceControl::gCallback, &forceControl);
    ros::Subscriber sub_pUp = n.subscribe("patch3", 1000, &ExoControllers::ForceControl::pUpCallback, &forceControl);
    ros::Subscriber sub_pLow = n.subscribe("patch2", 1000, &ExoControllers::ForceControl::pLowCallback, &forceControl);
    // ros::Subscriber sub_fUp = n.subscribe("patch5", 1000, &ExoControllers::ForceControl::fUpCallback, &forceControl);
    // ros::Subscriber sub_fLow = n.subscribe("patch4", 1000, &ExoControllers::ForceControl::fLowCallback, &forceControl);

    // ros::Subscriber sub_g = n.subscribe("skin_acc", 1000, &ExoControllers::ForceControl::gCallback, &forceControl);

    while (ros::ok())
    {
        gx = g * forceControl.get_m_gx();
        gy = g * forceControl.get_m_gy();
        gz = g * forceControl.get_m_gz();
        // ROS_INFO_STREAM("Main acc: x: " << gx << ", y: " << gy <<", z: " << gz); //30deg = 0.5236rad

        Ws = forceControl.get_m_prox_upper();
        Ws = forceControl.get_m_prox_lower();
        if (forceControl.get_m_prox_upper() > forceControl.get_m_prox_lower())
        {
            Ws = - forceControl.get_m_prox_upper();
            //ROS_INFO_STREAM("Upper Ws: " << Ws);
        }
        else 
        {
            Ws = forceControl.get_m_prox_lower();
            //ROS_INFO_STREAM("Lower Ws: " << Ws);
        }
        // Ws = forceControl.get_m_f_upper();
        // Ws = forceControl.get_m_f_lower();
        //cd tutorial-code/exo_ws/ROS_INFO_STREAM("Main Ws: " << Ws);

        // call position control update
        // tao = posControl.update(delta_t,q1,qd1,qdd1);
        // ROS_WARN_STREAM("tao=" << tao);

        m_matrix = I233 + L2 * L2 * m2 / 4;
        c_matrix = 0;
        g_matrix = -L2 / 2 * gx * m2 * sin(q1) + L2 / 2 * gy * m2 * cos(q1) - k1 * (theta1 - q1);
        b_matrix = b1;

        // // call force control update
        tao = forceControl.update(Ws) + g_matrix;
       
        // // ROS_WARN_STREAM("tao=" << tao);

        // m_matrix = I233 + L2 * L2 * m2 / 4;
        // c_matrix = 0;
        // g_matrix = -L2 / 2 * gx * m2 * sin(q1) + L2 / 2 * gy * m2 * cos(q1) - k1 * (theta1 - q1);
        // b_matrix = b1;

        // calculate qdd1 and integrate
        qdd1 = (tao - b_matrix * qd1 - c_matrix * qd1 - g_matrix) / m_matrix;
        qd1 = delta_t * qdd1 + qd1;
        q1 = delta_t * qd1 + q1;
        
        checkRange(q1, qd1, qdd1);

        std_msgs::Float64 msg;
        msg.data = q1 * 180 / 3.14159265359;
        exo_pub.publish(msg);
        
        // ROS_WARN_STREAM("qdd1="<<qdd1);
        // ROS_WARN_STREAM("qd1="<<qd1);
        //ROS_WARN_STREAM("tau = " << tao << "q1="<<msg.data);
        // ROS_WARN_STREAM("q1=" << q1 * 180 / 3.14159265359 << " degrees");

        q_state.data = q1 * 180 / 3.14159265359;        
        // q_state.q = q1 * 180 / 3.14159265359;
        // q_state.qd = qd1 * 180 / 3.14159265359;
        // q_state.qdd = qdd1 * 180 / 3.14159265359;
        pub_q_state.publish(q_state);
        
        // q_des.q = posControl.get_m_q_des() * 180 / 3.14159265359;
        // q_des.qd = posControl.get_m_qd_des() * 180 / 3.14159265359;
        // q_des.qdd = posControl.get_m_qdd_des() * 180 / 3.14159265359;
        // pub_q_des.publish(q_des);

        // pub_q_intention.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
