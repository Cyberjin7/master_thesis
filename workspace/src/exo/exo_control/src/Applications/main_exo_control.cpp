#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include <exo_control/exo_force_control.h>
#include "exo_control/exo_calibration.h"
#include <exo_control/q.h>
#include "experiment_srvs/MassChange.h"
#include "experiment_srvs/AngleChange.h"
#include "std_srvs/Empty.h"
#include "exo_msgs/q.h"
#include "exo_msgs/state.h"
#include "exo_msgs/calibration.h"

// #include <pthread.h>

double deg2rad(double degree) {
    return (degree * 3.14159265359 / 180.0);
}


void checkRange(double& q1, double& qd1, double& qdd1, double min, double max) {
    if (q1 > deg2rad(max) || q1 < deg2rad(min)) {
        if (q1 > deg2rad(max)) q1 = deg2rad(max); 
        if (q1 < deg2rad(min))  q1 = deg2rad(min);
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

bool change_angle(experiment_srvs::AngleChange::Request &req, experiment_srvs::AngleChange::Response &res, bool *forced, double *angle)
{
    *forced = true;
    *angle = deg2rad(req.angle.data);
    res.success.data = true;
    return true;
}

bool pred_toggle(std_srvs::EmptyRequest &req, std_srvs::Empty::Response &res, bool *pred)
{
    *pred = !*pred;
    ROS_INFO_STREAM("Predictive toggled to: " << *pred);
    return true;
}

bool cheat_toggle(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res, bool *cheat)
{
    *cheat = !*cheat;
    ROS_INFO_STREAM("Cheat toggled to: " << *cheat);
    return true;
}

bool start_cal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res, ExoControllers::Calibration* exo_cal)
{
    // exo_cal->set_start(true);
    exo_cal->start_cal();
    return true;
}

void externalCallback(const std_msgs::Float64::ConstPtr &msg, double *kp)
{
    *kp = msg->data;
    ROS_INFO_STREAM("Compensation Kp changed to: " << *kp);
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

    ros::Publisher ws_pub = n.advertise<std_msgs::Float64>("Ws", 100);
    ros::Publisher ws_N_pub = n.advertise<std_msgs::Float64>("Ws_N", 100);
    ros::Publisher intention_pub = n.advertise<std_msgs::Float64>("intention_force", 100);
    ros::Publisher compensation_pub = n.advertise<std_msgs::Float64>("compensation", 100);
    // ros::Publisher down_calibration_pub = n.advertise<std_msgs::Float64>("down_cal", 100);
    ros::Publisher down_calibration_pub = n.advertise<exo_msgs::calibration>("down_cal", 100);
    ros::Publisher up_calibration_pub = n.advertise<exo_msgs::calibration>("up_cal", 100);
    

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

    double kp_comp;
    ros::param::get("~compensation_ctrl/kp_down", kp_comp);

    double torque_limit;
    ros::param::get("~force_ctrl/tau_lim", torque_limit);

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
    bool predictive;
    ros::param::get("~predictive_wrench", predictive);
    bool cheat;
    ros::param::get("~cheat", cheat);

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

    bool forced_angle = false;

    // Calibration
    // double interval_angle, double interval_duration, double wait_duration, double angle_min, double angle_max
    ExoControllers::Calibration down_cal(interval, duration, wait, min_angle, max_angle, predictive);
    ExoControllers::Calibration up_cal(interval, duration, wait, min_angle, max_angle, false);
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

    ros::Subscriber comp_kp_sub = n.subscribe<std_msgs::Float64>("external_kp", 1, boost::bind(externalCallback, _1, &kp_comp));
    ros::Subscriber down_kp_sub = n.subscribe<std_msgs::Float64>("down_kp", 1, &ExoControllers::ForceControl::downKpCallback, &forceControl);
    ros::Subscriber up_kp_sub = n.subscribe<std_msgs::Float64>("up_kp", 1, &ExoControllers::ForceControl::upKpCallback, &forceControl);

    ros::ServiceServer mass_serv = n.advertiseService<experiment_srvs::MassChange::Request,experiment_srvs::MassChange::Response>("change_mass_request", boost::bind(change_mass, _1, _2, &m3));
    ros::ServiceServer cal_serv = n.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>("cal_trigger", boost::bind(start_cal, _1, _2, &down_cal));
    ros::ServiceServer angle_serv = n.advertiseService<experiment_srvs::AngleChange::Request,experiment_srvs::AngleChange::Response>("change_angle_request", boost::bind(change_angle, _1, _2, &forced_angle, &q1));

    ros::ServiceServer pred_toggle_serv = n.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("predictive_toggle", boost::bind(pred_toggle, _1, _2, &predictive));
    ros::ServiceServer cheat_toggle_serv = n.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("cheat_toggle", boost::bind(cheat_toggle, _1, _2, &cheat));

    std_msgs::Float64 torque_msg;
    std_msgs::Float64 delta_tau_msg;
    std_msgs::Float64 up_msg;
    std_msgs::Float64 down_msg;

    double prev_torque = 0;
    double prev_q = 0;
    double tmp;

    bool direction = true; // up is false, down is true

    std_msgs::Float64 ws_msg;
    std_msgs::Float64 ws_N_msg;
    std_msgs::Float64 intention_msg;
    std_msgs::Float64 compensation_msg;
    std_msgs::Float64 down_cal_msg;

    double intention_force = 0.0;
    double compensation_force = 0.0;

    exo_msgs::calibration cal_msg;

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
            if(!up_cal.get_start()){
                up_cal.start_cal();
            }
            // up_cal.set_start(true);
            tmp = down_cal.calibrate(forceControl.downFilterreading[9]);
            up_cal.calibrate(forceControl.upFilterreading[9]);
            std_msgs::Float64 msg;
            msg.data = tmp;
            // q1 = tmp*3.14159265359/180;
            q1 = deg2rad(tmp);
            // ROS_INFO_STREAM("q: " << q1);
            exo_pub.publish(msg);
            qd1 = 0;
            qdd1 = 0;

            if(down_cal.get_done()){
                ROS_INFO_STREAM("Sending calibration values");
                std::map<double, double> down_cal_values = down_cal.get_cal_values();
                std::map<double, double> up_cal_values = up_cal.get_cal_values();
                cal_msg.type = "down";
                for (auto const& [angle, value] : down_cal_values){
                    cal_msg.angle = angle;
                    cal_msg.value = value;
                    down_calibration_pub.publish(cal_msg);
                } 
                cal_msg.type = "up";
                for (auto const& [angle, value] : up_cal_values){
                    cal_msg.angle = angle;
                    cal_msg.value = value;
                    up_calibration_pub.publish(cal_msg);
                }
            }

        }
        else if (forced_angle){
            ROS_INFO_STREAM("Forcing angle to: " << q1);
            forced_angle = false;

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

            /*
            Approach 1: Roughly assume mass is placed on top of skin.
            Linear extrapolation: based on the stored calibration value, extrapolate sensor value for predicted mass and subtract in addition to stored value
            */
            // if(predictive){
            //     Ws_down = forceControl.downFilterreading[9] - (1+(m3/m2))*down_cal.interp_force(q1*180/3.14159265359);
            // }

            /*
            Approach 2: Subtract theoretical torque of predicted mass
            Linear extrapolate force value from sensor readings based on stored calibration values
            */
        //    if(predictive){
        //        Ws_down = forceControl.downFilterreading[9] - down_cal.interp_force(q1*180/3.14159265359);
        //        Ws_down = -m2*g*sin(q1)*(Ws_down)/(down_cal.interp_force(q1*180/3.14159265359)); // minus just to get rid of the minus in g
        //    }

            if(Ws_down < 0){
                Ws_down = 0.0;
            }
            if(Ws_up < 0){
                Ws_up = 0.0;
            }

            if(cheat){Ws_up = 0.0;}

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
                direction = false;
                // ROS_INFO_STREAM("Up: " << Ws);
            }
            else{
                // Ws = -Ws_down;
                if(down_cal.get_done() && predictive){
                    // ROS_INFO_STREAM("Cal: " << down_cal.interp_force(q1*180/3.14159265359));
                    // Ws = m2*g*sin(q1)*(Ws_down)/(down_cal.interp_force(q1*180/3.14159265359));

                    // down_cal_msg.data = down_cal.interp_force(q1*180/3.14159265359);
                    // down_calibration_pub.publish(down_cal_msg);

                    // ROS_INFO_STREAM("Down: " << Ws);
                    Ws = - Ws_down;
                }
                else{
                    Ws = -Ws_down;
                    // ROS_INFO_STREAM("Down: " << Ws);
                }
                direction = true;
                // ROS_INFO_STREAM("Down: " << m2*g*sin(q1)*(Ws_down)/(down_cal.interp_force(q1*180/3.14159265359)));
                // ROS_INFO_STREAM("Down: " << Ws);
            }

            // // call force control update
            prev_torque = tao;
            // tao = forceControl.update(Ws) + g_matrix;
            // Approach 2: 
            // TODO: make it work lol
            if(predictive && direction){
                // double intention_force = forceControl.update(Ws); // * kp_down;
                intention_force = forceControl.update(Ws);
                // double compensation_force = g*(L2+L3)*m3*sin(q1)*kp_comp;
                compensation_force = g*(L2+L3)*m3*sin(q1)*kp_comp;
                // ROS_INFO_STREAM("Ws: " << Ws);
                // ROS_INFO_STREAM("Down Ws: " << intention_force);
                // ROS_INFO_STREAM("Compensation: " << compensation_force);

                // ws_msg.data = Ws_down; 
                // ws_N_msg.data = Ws;
                // intention_msg.data = intention_force;
                // compensation_msg.data = compensation_force;

                // ws_pub.publish(ws_msg);
                // ws_N_pub.publish(ws_N_msg);
                // intention_pub.publish(intention_msg);
                // compensation_pub.publish(compensation_msg);

                // if(intention_force > compensation_force){ // less negative means less force (but means larger value)
                //     tao = g_matrix;
                // }
                // else{
                //     tao = intention_force + g_matrix - compensation_force;
                // }

                // if(intention_force<-torque_limit){
                //     intention_force = -torque_limit;
                // }

                tao = intention_force + g_matrix - compensation_force;

                // tao = tao - g*L3*m3*sin(q1); // minus because g has minus
                // if(tao - g_matrix > 0){
                //     tao = forceControl.update(0.0) + g_matrix;
                // }
                // ROS_INFO_STREAM("Intention: " << intention_force);
                // ROS_INFO_STREAM("Compensation: " << compensation_force);
                // ROS_WARN_STREAM("Down=" << tao - g_matrix);
            }
            else{
                intention_force = forceControl.update(Ws);
                tao = intention_force + g_matrix;
                // tao = forceControl.update(Ws) + g_matrix;
                // ROS_WARN_STREAM("Up=" << tao - g_matrix);
            }

            ws_msg.data = Ws;
            intention_msg.data = intention_force;
            compensation_msg.data = compensation_force;

            ws_pub.publish(ws_msg);
            // ws_N_pub.publish(ws_N_msg);
            intention_pub.publish(intention_msg);
            compensation_pub.publish(compensation_msg);

            // if(std::abs(tao-g_matrix) >= torque_limit){
            //     if (signbit(tao-g_matrix)){
            //         tao = -1*(torque_limit) +g_matrix;
            //     }
            //     else{
            //         tao = torque_limit + g_matrix;
            //     }
            //     // tao = torque_limit*signbit(tao);
            // }

            // ROS_WARN_STREAM("tao=" << tao - g_matrix);
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
            
            checkRange(q1, qd1, qdd1, min_angle, max_angle);

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
        q_state.q = q1 * 180 / 3.14159265359; // bug: in calibration, angles are in degs. converting is wrong but I don't care now
        q_state.qd = qd1 * 180 / 3.14159265359;
        q_state.qdd = qdd1 * 180 / 3.14159265359;
        pub_q_state.publish(q_state);

        state.header.stamp = ros::Time::now();
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
