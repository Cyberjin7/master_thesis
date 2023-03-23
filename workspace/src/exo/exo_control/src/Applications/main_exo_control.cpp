#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include <exo_control/exo_force_control.h>
#include "exo_control/exo_calibration.h"
#include <exo_control/q.h>
#include "experiment_srvs/MassChange.h"
#include "std_srvs/Empty.h"
#include "exo_msgs/q.h"
#include "exo_msgs/state.h"

// #include <pthread.h>

double deg2rad(double degree) {
    return (degree * 3.14159265359 / 180.0);
}


void checkRange(double& q1, double& qd1, double& qdd1) {
    if (q1 > deg2rad(100) || q1 < deg2rad(10)) {
        if (q1 > deg2rad(100)) q1 = deg2rad(100); 
        if (q1 < deg2rad(10))  q1 = deg2rad(10);
        qd1 = 0;
        qdd1 = 0;
    }
}

bool change_mass(experiment_srvs::MassChange::Request &req, experiment_srvs::MassChange::Response &res, double *mass)
{
    *mass = req.mass.data;
    ROS_INFO_STREAM("Received mass: " << *mass);
    return true;
}

bool start_cal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res, ExoControllers::Calibration* exo_cal)
{
    exo_cal->set_start(true);
    return true;
}

int main(int argc, char** argv)
{
    // int f = 200;
    ros::init(argc, argv, "exo_control",
        ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher exo_pub = n.advertise<std_msgs::Float64>("/q_control_publisher", 10);
    // ros::Publisher pub_q_state = n.advertise<exo_control::q>("/q_state", 1);
    ros::Publisher pub_q_state = n.advertise<exo_msgs::q>("/q_state", 100);
    ros::Publisher state_pub = n.advertise<exo_msgs::state>("state", 100);
    // ros::Publisher pub_q_state = n.advertise<std_msgs::Float64>("/q_state", 1);
    // ros::Publisher pub_q_state = n.advertise<std_msgs::Float64>("q", 100);
    // ros::Publisher pub_q_des = n.advertise<exo_control::q>("/q_des", 1);
    // ros::Publisher pub_q_intention = n.advertise<std_msgs::Float64>("/q_intention", 1);
    int f;
    ros::param::get("~rate", f);
    ROS_INFO_STREAM("Rate: " << f);
    ros::Rate r(f);

    // std_msgs::Float64 q_state;
    // exo_control::q q_state;
    exo_control::q q_des;

    exo_msgs::q q_state;
    exo_msgs::state state;

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
    double L3;
    ns = "~L3";
    s.str("");
    s << ns;
    ros::param::get(s.str(), L3);
    double m2;
    ns = "~m2";
    s.str("");
    s << ns;
    ros::param::get(s.str(), m2);
    double m3;
    ns = "~m3";
    s.str("");
    s << ns;
    ros::param::get(s.str(), m3);
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
    theta1 = deg2rad(theta1);
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

    double interval;
    ros::param::get("~calibration/interval", interval);
    double duration;
    ros::param::get("~calibration/duration", duration);
    double wait;
    ros::param::get("~calibration/wait", wait);
    double max_angle;
    ros::param::get("~max", max_angle);
    double min_angle;
    ros::param::get("~min", min_angle);

    double tao = 0;

    //init params 
    double q1 = deg2rad(45);
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
     ExoControllers::PosControl posControl(L1, L2, L3, m2, m3, b1, k1, theta1, gx, gy);
     Vector3d qEnd;
     qEnd << deg2rad(90),0.0,0.0;
     double timeEnd = 5;
     posControl.init(qEnd,timeEnd);

    //load force control
    ExoControllers::ForceControl forceControl((L2+L3)/2, n); // must adjust ForceControl to take into account new formula. For now just replace L2 with correct length
    double W_des = 0;
    double Ws = 0;
    double Ws_up = 0;
    double Ws_down = 0;

    // Calibration
    // double interval_angle, double interval_duration, double wait_duration, double angle_min, double angle_max
    ExoControllers::Calibration down_cal(interval, duration, wait, min_angle, max_angle);
    ExoControllers::Calibration up_cal(interval, duration, wait, min_angle, max_angle);
    // calibrator.set_start(true);
    for(auto i: down_cal.get_cal_angles()){
        ROS_INFO_STREAM("Calibration angle: " << i);
    }

    forceControl.init(W_des);
    ros::Subscriber sub_g = n.subscribe("patch1", 1000, &ExoControllers::ForceControl::gCallback, &forceControl); // patch1: upper arm
    ros::Subscriber sub_pUp = n.subscribe("patch3", 1000, &ExoControllers::ForceControl::fLowCallback, &forceControl); // patch3: bottom inner
    // ros::Subscriber sub_pLow = n.subscribe("patch2", 1000, &ExoControllers::ForceControl::pLowCallback, &forceControl); // patch2: bottom outer
    ros::Subscriber sub_fUp = n.subscribe("patch5", 1000, &ExoControllers::ForceControl::fUpCallback, &forceControl); // patch5: upper inner
    // ros::Subscriber sub_fLow = n.subscribe("patch4", 1000, &ExoControllers::ForceControl::pUpCallback, &forceControl); // patch4: upper outer

    // ros::Publisher cal_pub = n.advertise<std_msgs::Float64>("cal_force", 100);
    ros::Publisher torque_pub = n.advertise<std_msgs::Float64>("torque", 100);
    ros::Publisher delta_tau_pub = n.advertise<std_msgs::Float64>("delta_tau", 100);
    ros::Publisher up_pub = n.advertise<std_msgs::Float64>("up", 100);
    ros::Publisher down_pub = n.advertise<std_msgs::Float64>("down", 100);


    ros::ServiceServer mass_serv = n.advertiseService<experiment_srvs::MassChange::Request,experiment_srvs::MassChange::Response>("change_mass", boost::bind(change_mass, _1, _2, &m3));
    ros::ServiceServer cal_serv = n.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>("cal_trigger", boost::bind(start_cal, _1, _2, &down_cal));

    std_msgs::Float64 torque_msg;
    std_msgs::Float64 delta_tau_msg;
    std_msgs::Float64 up_msg;
    std_msgs::Float64 down_msg;

    double prev_torque = 0;
    double prev_q = 0;

    while (ros::ok())
    {
        gx = g * forceControl.get_m_gx();
        gy = g * forceControl.get_m_gy();
        gz = g * forceControl.get_m_gz();
        // ROS_INFO_STREAM("Main acc: x: " << gx << ", y: " << gy <<", z: " << gz); //30deg = 0.5236rad

        // m_matrix = I233 + L2 * L2 * m2 / 4;
        m_matrix = I233 + (std::pow(L2+L3, 2)*m2/4) + (std::pow(2*L2+L3, 2)*m3/4);
        c_matrix = 0;
        // g_matrix = -L2 / 2 * gx * m2 * sin(q1) + L2 / 2 * gy * m2 * cos(q1) - k1 * (theta1 - q1);
        g_matrix = (gy*cos(q1) - gx*sin(q1))*(L2+L3)*m2/2 + (gy*cos(q1) - gx*sin(q1))*(2*L2 + L3)*m3/2 - k1*(theta1-q1);
        b_matrix = b1;

        // call position control update
        // tao = posControl.update(delta_t,q1,qd1,qdd1);
        // ROS_WARN_STREAM("tao=" << tao);

        if (down_cal.get_start())
        {
            up_cal.set_start(true);
            double tmp;
            tmp = down_cal.calibrate(forceControl.downFilterreading[9]);
            up_cal.calibrate(forceControl.upFilterreading[9]);
            std_msgs::Float64 msg;
            msg.data = tmp;
            q1 = tmp;
            exo_pub.publish(msg);
        }
        else
        {
            // double force_change = forceControl.forceFilterreading[9] - forceControl.forceFilterreading[5];
            // ROS_INFO_STREAM("Change: " << force_change);
            // if (force_change > 0.01) // extension
            // {
            //     ROS_INFO_STREAM("Down: ");
            //     Ws = -force_change*10.0;
            // }
            // else if (force_change < -0.01) // contraction
            // {
            //     ROS_INFO_STREAM("Up: ");
            //     Ws = -force_change*10.0;
            // }
            // else{
            //     Ws = 0.0;
            // }

            // Ws = forceControl.downFilterreading[9] - down_cal.interp_force(q1*180/3.14159265359);
            Ws_down = forceControl.downFilterreading[9] - down_cal.interp_force(q1*180/3.14159265359);
            Ws_up = forceControl.upFilterreading[9] - up_cal.interp_force(q1*180/3.14159265359);

            if(Ws_down < 0){
                Ws_down = 0.0;
            }
            if(Ws_up < 0){
                Ws_up = 0.0;
            }

            up_msg.data = Ws_up;
            down_msg.data = Ws_down;
            up_pub.publish(up_msg);
            down_pub.publish(down_msg);
            // ROS_INFO_STREAM("Calibrated Ws_down: " << -Ws_down);
            // ROS_INFO_STREAM("Calibrated Ws_up: " << -Ws_up);
            // std_msgs::Float64 cal_force;
            // cal_force.data = -Ws_down;
            // cal_pub.publish(cal_force);
            // Ws = -force_change * 20;
            // Ws=0.0;
            if(std::abs(Ws_up) > std::abs(Ws_down)){
                Ws = Ws_up;
            }
            else{
                Ws = -Ws_down;
            }

            // // call force control update
            prev_torque = tao;
            tao = forceControl.update(Ws) + g_matrix;
            // ROS_WARN_STREAM("tao=" << tao);
            // if(std::abs(tao - prev_torque) > 0.1){
            //     tao = prev_torque;
            // }
            torque_msg.data = tao - g_matrix;
            torque_pub.publish(torque_msg);

            delta_tau_msg.data = tao - prev_torque;
            delta_tau_pub.publish(delta_tau_msg);

            prev_q = q1;
            // calculate qdd1 and integrate
            qdd1 = (tao - b_matrix * qd1 - c_matrix * qd1 - g_matrix) / m_matrix;
            qd1 = delta_t * qdd1 + qd1;
            q1 = delta_t * qd1 + q1;

            // if (std::abs(q1 - prev_q) < deg2rad(0.01)){
            //     q1 = prev_q;
            // }
            
            checkRange(q1, qd1, qdd1);

            std_msgs::Float64 msg;
            msg.data = q1 * 180 / 3.14159265359;
            // ROS_INFO_STREAM("q: " << msg.data);
            exo_pub.publish(msg);
            // ROS_INFO_STREAM("m3: " << m3);
        }
        
        
        // ROS_WARN_STREAM("qdd1="<<qdd1);
        // ROS_WARN_STREAM("qd1="<<qd1);
        //ROS_WARN_STREAM("tau = " << tao << "q1="<<msg.data);
        // ROS_WARN_STREAM("q1=" << q1 * 180 / 3.14159265359 << " degrees");

        // q_state.data = q1 * 180 / 3.14159265359;
        q_state.q = q1 * 180 / 3.14159265359;
        q_state.qd = qd1 * 180 / 3.14159265359;
        q_state.qdd = qdd1 * 180 / 3.14159265359;
        pub_q_state.publish(q_state);

        state.q_state.q = q1 * 180 / 3.14159265359;
        state.q_state.qd = qd1 * 180 / 3.14159265359;
        state.q_state.qdd = qdd1 * 180 / 3.14159265359;
        state.tau = tao - g_matrix;
        state.force_down = Ws_down;
        state.force_up = Ws_up;
        state_pub.publish(state);
        
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
