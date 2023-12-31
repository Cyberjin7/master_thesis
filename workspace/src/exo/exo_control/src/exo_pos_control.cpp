#include <exo_control/exo_pos_control.h>

namespace ExoControllers{

    PosControl::PosControl(double L1, double L2, double L3, double m2, double m3, double b1, double k1, 
                            double theta1, double gx, double gy)    
    {
        ROS_INFO_STREAM("Position Controller Created");

        m_L1 = L1;
        m_L2 = L2;
        m_L3 = L3;
        m_m2 = m2;
        m_m3 = m3;
        m_b1 = b1;
        m_k1 = k1;
        m_theta1 = theta1;
        m_gx = gx;
        m_gy = gy;

        m_q_des = 0.0;
        m_qd_des = 0.0;
        m_qdd_des = 0.0;
        // m_kp = 0.0;
        // m_kd = 0.0;
        // m_ki = 0.0;
        m_tao = 0.0;
        m_taor = 0.0;
        m_startFlag = false;
        m_deltaQ = 0.0;
        m_timeStart = 0.0;
        m_qStart = Vector3d::Zero(); 
        m_qEnd = Vector3d::Zero(); 
        m_timeEnd = 0.0;

        error_gain = 0.0;
        prev_error = 0.0;

        std::string ns = "~pos_ctrl";
        std::stringstream s; 
        
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("pos m_kp: \n"<<m_kp);

        s.str("");
        s<<ns<<"/ki";
        ros::param::get(s.str(),m_ki);
        ROS_WARN_STREAM("pos m_ki: \n"<<m_ki);

        s.str("");
        s<<ns<<"/kd";
        ros::param::get(s.str(),m_kd);
        ROS_WARN_STREAM("pos m_kd: \n"<<m_kd);
    }

    PosControl::~PosControl()
    {
    }

    // 3. Trajectory generation
    void PosControl::trajGen(double qi, double qf, double qpi,
                             double qpf, double qppi, double qppf,
                             double t0, double tf, double tc,
                             bool startFlag)
    {
      VectorXd static aq1(6);
      VectorXd static q(6);
      MatrixXd static T(6,6);
      VectorXd p(6);
      VectorXd v(6);
      VectorXd a(6);

      if(!startFlag){
        q << qi,qf,qpi,qpf,qppi,qppf;

        T <<     1, t0, pow(t0,2), pow(t0,3),   pow(t0,4),    pow(t0,5),
                 1, tf, pow(tf,2), pow(tf,3),   pow(tf,4),    pow(tf,5), //toDO! 
                 0, 1,  2*t0,      3*pow(t0,2), 4*pow(t0,3),  5*pow(t0,4),
                 0, 1,  2*tf,      3*pow(tf,2), 4*pow(tf,3),  5*pow(tf,4),
                 0, 0,  2,         6*t0,        12*pow(t0,2), 20*pow(t0,3),
                 0, 0,  2,         6*tf,        12*pow(tf,2), 20*pow(tf,3);

        aq1 = T.colPivHouseholderQr().solve(q);
      }

      if(tc<=tf)
      {
      p << 1, tc, pow(tc,2), pow(tc,3), pow(tc,4), pow(tc,5);
      v << 0, 1,  2*tc,      3*pow(tc,2), 4*pow(tc,3),  5*pow(tc,4); //to Do
      a << 0, 0,  2,         6*tc,        12*pow(tc,2), 20*pow(tc,3); // to Do
      m_q_des =  p.transpose()*aq1;
      m_qd_des = v.transpose()*aq1; //to Do
      m_qdd_des = a.transpose()*aq1; // to Do 
      }
      else{ 
        m_q_des = qf; //to Do
        m_qd_des = qpf;
        m_qdd_des = qppf; //to Do
      }
    }

    // 4. regressor
    double PosControl::YrTheta(double q1, double qd1, double qd1r, double qdd1r)
    {
        MatrixXd Yr(1, 6);//to DO
        MatrixXd Theta(6, 1);//to DO

        Yr(0,0) = qdd1r;
        Yr(0,1) = qd1r;//to Do! 
        Yr(0,2) = q1;
        Yr(0,3) = sin(q1);
        Yr(0,4) = cos(q1);
        Yr(0,5) = 1;

        // Theta(0,0) = m_I233 + m_L2 * m_L2 * m_m2 / 4; // to Do 
        // Theta(1,0) = m_b1;
        // Theta(2,0) = m_k1;
        // Theta(3,0) = -m_L2 / 2 * m_gx * m_m2;
        // Theta(4,0) = m_L2 / 2 * m_gy * m_m2;
        // Theta(5,0) = -m_k1 * m_theta1;

        Theta(0,0) = m_I233 + (std::pow(m_L2+m_L3, 2)*m_m2/4) + (std::pow(2*m_L2+m_L3, 2)*m_m3/4);
        Theta(1,0) = m_b1;
        Theta(2,0) = m_k1;
        Theta(3,0) = -m_gx*(m_m2*(m_L2+m_L3) + m_m3*(2*m_L2 + m_L3))/2;
        Theta(4,0) = m_gy*(m_m2*(m_L2+m_L3) + m_m3*(2*m_L2 + m_L3))/2;
        Theta(5,0) = -m_k1 * m_theta1;

        MatrixXd taor = Yr*Theta; 
        return taor(0,0);
    }

    bool PosControl::init(Vector3d qEnd, double timeEnd)
    {
        m_qEnd = qEnd;
        m_timeEnd = timeEnd;

        m_startFlag = false;

        return true;
    }

    // 5. PD controller 
    double PosControl::update(double delta_t, double q1, double qd1, double qdd1)
    {
        if(!m_startFlag)
        {
            m_timeStart = ros::Time::now().toSec();
            m_qStart << q1,qd1,qdd1;
            trajGen(m_qStart[0],m_qEnd[0],m_qStart[1],m_qEnd[1], //gen traj to initpos
                m_qStart[2],m_qEnd[2],0.0,m_timeEnd,0.0,m_startFlag);
            m_startFlag = true; 
        }
        else
        {
            trajGen(m_qStart[0],m_qEnd[0],m_qStart[1],m_qEnd[1], //gen traj to initpos
                m_qStart[2],m_qEnd[2],0.0,m_timeEnd,ros::Time::now().toSec()-m_timeStart,m_startFlag);
        }

        double deltaQ = q1 - m_q_des;
        double deltaQd = qd1 - m_qd_des;// to Do! 
        error_gain = error_gain = 0.5 * (deltaQ + prev_error) / 20;
        prev_error = deltaQ;

        double qd1r = qd1 - m_kp*deltaQ - m_ki*error_gain;// to Do!
        double qdd1r = qdd1 - m_kp*deltaQd - m_ki*deltaQ; 

        double Sq = qd1 - qd1r;// to DO! 
        m_tao = -m_kd*Sq + YrTheta(q1, qd1, qd1r, qdd1r);// to Do! 

        // ROS_WARN_STREAM("t=" << ros::Time::now().toSec()-m_timeStart);
        // ROS_WARN_STREAM("m_q_des=" << m_q_des * 180 / 3.14159265359);
        // ROS_WARN_STREAM("m_qd_des=" << m_qd_des * 180 / 3.14159265359);
        // ROS_WARN_STREAM("m_qdd_des=" << m_qdd_des * 180 / 3.14159265359);
        // ROS_WARN_STREAM("q1=" << q1 * 180 / 3.14159265359);
        // ROS_WARN_STREAM("qd1=" << qd1 * 180 / 3.14159265359);
        // ROS_WARN_STREAM("qdd1=" << qdd1 * 180 / 3.14159265359);
        // ROS_WARN_STREAM("deltaQ=" << deltaQ * 180 / 3.14159265359);
        // ROS_WARN_STREAM("deltaQd=" << deltaQd * 180 / 3.14159265359);
        // ROS_WARN_STREAM("qd1r=" << qd1r * 180 / 3.14159265359);
        // ROS_WARN_STREAM("qdd1r=" << qdd1r * 180 / 3.14159265359);
        // ROS_WARN_STREAM("Sq=" << Sq * 180 / 3.14159265359);
        ROS_WARN_STREAM("YrTheta=" << YrTheta(q1, qd1, qd1r, qdd1r));

        return m_tao;
    }

    double PosControl::get_m_q_des()
    {
        return m_q_des;
    }

    double PosControl::get_m_qd_des()
    {
        return m_qd_des;
    }

    double PosControl::get_m_qdd_des()
    {
        return m_qdd_des;
    }
    
}