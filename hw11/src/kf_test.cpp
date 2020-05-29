#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include "hw11/kalman.h"
#include <queue>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kf_test");
    ros::NodeHandle nh;

  int n = 3; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0 ; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix (3*3)
  Eigen::MatrixXd B(n, m); // input control (3*1)
  Eigen::MatrixXd C(m, n); // Output matrix (1*3)
  Eigen::MatrixXd Q(n, n); // Process noise covariance (3*3)
  Eigen::MatrixXd R(m, m); // Measurement noise covariance (1*1)
  Eigen::MatrixXd P(n, n); // Estimate error covariance (3*3)

  // Please set parameter in this part and use the dynamic model parameter.
	A << 1,dt,0,
		0,1,dt,
		0,0,1;
		
	C << 1,0,0;
	B << 0,
		0,
		1;
		
	Q << 400,0,0,
		0,0.01,0,
		0,0,0.01;
	
	R << 1;
	P << 1.0,0,0,
		0,1.0,0,
		0,0,1.0;

  //

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, B, C, Q, R, P);

  // List of noisy position measurements (y)
  std::vector<double> measurements = {
  21.36207, -18.41971, 143.90836, -52.76939, 81.52611, 67.17912, -5.93449, -0.94632, 111.84432, 38.45614, 141.10963, 196.00361, 110.93513, 157.82011, 163.859, 189.89853, 347.60249, 344.48271, 410.5627, 370.36573, 446.26576, 542.61435, 485.45122, 595.36523, 653.16042, 687.99158, 755.92574, 848.06704, 870.21786, 872.59879, 892.16949, 1048.16843, 1114.30984, 1098.00347, 1273.89832, 1283.50095, 1408.41329, 1356.39875, 1572.20031, 1519.29754, 1695.31637, 1692.42357, 1933.565, 1943.41573, 2008.02941, 2142.3841
  };

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  x0 << measurements[0], 0, -9.81;
  kf.init(dt,x0);
 
  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  std::queue<double> answer;
  for (int i = 0; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i];
    kf.update(y);
    //show the position velocity acceleration
    std::cout <<  kf.state().transpose() << std::endl;
  }
  std::ofstream in;
  //chage your path "home/ee405423/Desktop"
  in.open("/home/catkin_ws/src/Homework/hw11/src",std::ios::out | std::ios::app);
  

  int len = answer.size();
  for(int i =0; i<len;i++)
  {
    std::cout<<"answer.front()"<<answer.front()<<std::endl;
    in << answer.front()<<",";
    answer.pop();
  }
    ros::spin();
  return 0;
}
