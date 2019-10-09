#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>
//#include <Eigen/Dense>

using namespace std;
//using namespace Eigen;

//Thruster outputs
float Tz = 0;
float Tstbd = 0;
float Tport = 0;

//Sensor feedback
float theta = 0;
float u = 0;
float v = 0;
float r = 0;

//Tracking variables
float Tx = 0;
float psi_d = 0;

void dthrust_callback(const std_msgs::Float64::ConstPtr& tx)
{
	Tx = tx->data;
}

void dheading_callback(const std_msgs::Float64::ConstPtr& psid)
{
	psi_d = psid->data;
}

void ins_callback(const geometry_msgs::Pose2D::ConstPtr& ins)
{
	theta = ins->theta;
}

void vel_callback(const geometry_msgs::Vector3::ConstPtr& vel)
{
	u = vel->x;
	v = vel->y; 
	r = vel->z;
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "dynamicmodel_c");

  	ros::NodeHandle n;

	//ROS Publishers for each required sensor data
	ros::Publisher right_thruster_pub = n.advertise<std_msgs::Float64>("right_thruster", 1000);
	ros::Publisher left_thruster_pub = n.advertise<std_msgs::Float64>("left_thruster", 1000);

	ros::Subscriber desired_thrust_sub = n.subscribe("desired_thrust", 1000, dthrust_callback);
	ros::Subscriber desired_heading_sub = n.subscribe("desired_heading", 1000, dheading_callback);
	ros::Subscriber ins_pose_sub = n.subscribe("ins_pose", 1000, ins_callback);
	ros::Subscriber local_vel_sub = n.subscribe("local_vel", 1000, vel_callback);

	ros::Rate loop_rate(10);

  //Model pysical parameters
  float Xu = 0;
  float Nr = 0;
  float X_u_dot = -2.25;
  float Y_v_dot = -23.13;
  float N_r_dot = -2.79;
  float Xuu = 0;
  float m = 30;
  float Iz = 4.1;
  float B = 0.41;
  float c = 0.78;

  //Controller gains
  float Ka = 0;
  float k = 0.5;
  float kmin = 0.1;
  float k2 = 1;
  float miu = 0.1;
  float lambda = 1;

  //Auxiliry variables
  float Ka_dot = 0;
  float Ka_dot_last = 0;
  float ua = 0;


  while (ros::ok())
  {
	
  	
  	Xu = -25;
  	Xuu = 0;
  	float u_abs = abs(u);
  	if (u_abs > 1.2){
  		Xu = 64.55;
  		Xuu = -70.92;
  	}

  	Nr = (-0.52)*sqrt(pow(u,2)+pow(v,2));

  	float g;
  	g = (1 / (Iz - Y_v_dot));

    float f;
  	f = (((- X_u_dot - Y_v_dot)*u*v + (Nr*r)) / (Iz - Y_v_dot));

  	float zeta;
  	zeta = theta;

  	float zeta_dot;
  	zeta_dot = r;

  	float dzeta;
  	dzeta = psi_d;

  	float dzeta_dot;
  	dzeta_dot = 0;

  	float dzeta_dotdot;
  	dzeta_dotdot = 0;

  	float e = zeta - dzeta;
  	float e_dot = zeta_dot - dzeta_dot;

  	float s = e_dot + lambda*e;
  	float s_abs = abs(s);

  	int sign = 0;
  	int sign0 = 0;
  	if (Ka > kmin){
    		float signvar = s_abs - miu;
    		if (signvar = 0){
    			sign = 0;
    		}
    		else {
    			sign = copysign(1,signvar);
    		}
    		Ka_dot = k * sign;
  	}
  	else{
  		Ka_dot = kmin;
  	} 

  	Ka = (0.01)*(Ka_dot + Ka_dot_last)/2 + Ka;
  	Ka_dot_last = Ka_dot;

  	if (s = 0){
  		sign0 = 0;
  	}
  	else {
  		sign0 = copysign(1,s);
  	}
  	ua = -Ka*sqrt(s_abs)*sign0 - k2*s;


  	Tz = (f - dzeta_dotdot - lambda*(zeta_dot - dzeta_dot) + ua);

  	Tport = (Tx / 2) + (Tz / B);
  	Tstbd = (Tx / (2*c)) - (Tz / (B*c));


  	cout << Ka_dot << endl;
  	cout << Ka_dot_last << endl;

  	if (Tstbd > 36.5){
  		Tstbd = 20;
  	}
  	else if (Tstbd < -30){
  		Tstbd = -20;
  	}
  	if (Tport > 36.5){
  		Tport = 20;
  	}
  	else if (Tport < -30){
  		Tport = -20;
  	}

//Data publishing
  	std_msgs::Float64 rt;
  	std_msgs::Float64 lt;

  	rt.data = Tstbd;
  	lt.data = Tport;

    right_thruster_pub.publish(rt);
    left_thruster_pub.publish(lt);

    ros::spinOnce();

    loop_rate.sleep();
  }

	return 0;
}