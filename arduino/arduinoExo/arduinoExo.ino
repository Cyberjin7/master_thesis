/*
 * Brief description
 * 
 */

#include <ros.h>
#include <Servo.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>


ros::NodeHandle nh;
Servo myservo;


#define SERVO_PIN 4

double posServo1 = 500; // 2100; //0
double posServo2 = 2500; // 1160; //90 (180)

int elbowMin = 10;
int elbowMax = 100;

double toggle = true; 

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// aim: 
// elbowMin -> 2500
// elbowMax -> 500


void angleCommand(const std_msgs::Float64& angle_msg){
  if (angle_msg.data >= elbowMin && angle_msg.data <= elbowMax){
    //double angle = mapf(angle_msg.data*3.1415926/180.0, 0.0, 3.1415926, posServo1, posServo2);
    double angle = mapf(angle_msg.data, elbowMax, elbowMin, posServo1, posServo2);
    myservo.writeMicroseconds(angle);
  }
}

ros::Subscriber<std_msgs::Float64> sub("/q_control_publisher", &angleCommand);


void attachCb(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  if (req.data){
    myservo.attach(SERVO_PIN);
    res.message = "Servo attached!";
  }
  else if (!req.data){
    myservo.detach();
    res.message = "Servo detached!";
  }
  res.success = true;
}

// ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> toggler("toggleMotor", attachCb);


void setup()
{
  pinMode(SERVO_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  // nh.advertiseService(toggler);

  myservo.attach(SERVO_PIN); // PWM pin
  delay(1000);
  
}

void loop()
{
  /*
  if(toggle){
    double angle = mapf(0.0, 0.0,1.570796*2, posServo1, posServo2);
    myservo.writeMicroseconds(angle);
    toggle = false;
  }
  else{
//    double angle = mapf(0.785398, 0.0,1.570796, posServo1, posServo2); 
    double angle = mapf(1.570796*2, 0.0,1.570796*2, posServo1, posServo2); 
    myservo.writeMicroseconds(angle);
    toggle = true;
  }
  */
  
  
  nh.spinOnce();
  //delay(1);
}
