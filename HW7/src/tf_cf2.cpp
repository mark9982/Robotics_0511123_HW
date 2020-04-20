#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
	static tf::TransformBroadcaster br;
	
	//B to C 0degree

	//r>x g>y b>z
	//B1 to C 0degree
	Eigen::Quaterniond q_C_B1(1,0,0,0);
	Eigen::Vector3d v_C_B1(1,-1,0);

	tf::Transform tf_C_B1;
	tf::Quaternion tf_q_C_B1(q_C_B1.x(),q_C_B1.y(),q_C_B1.z(),q_C_B1.w());
	tf::Vector3 tf_v_C_B1(v_C_B1(0),v_C_B1(1),v_C_B1(2));
	
	tf_C_B1.setOrigin(tf_v_C_B1);
	tf_C_B1.setRotation(tf_q_C_B1);

	br.sendTransform(tf::StampedTransform(tf_C_B1,
											ros::Time::now(),
											"C",
											"B1"));
											

	
	//C to A
	//打A不會過，要改A1									
	Eigen::Quaterniond q_A_C(1,0,0,0);
	Eigen::Vector3d v_A_C(0,1,0);

	tf::Transform tf_A_C;
	tf::Quaternion tf_q_A_C(q_A_C.x(),q_A_C.y(),q_A_C.z(),q_A_C.w());
	tf::Vector3 tf_v_A_C(v_A_C(0),v_A_C(1),v_A_C(2));
	
	tf_A_C.setOrigin(tf_v_A_C);
	tf_A_C.setRotation(tf_q_A_C);

	br.sendTransform(tf::StampedTransform(tf_A_C,
											ros::Time::now(),
											"A",
											"C"));
											
	//B to A
	Eigen::Quaterniond q_A_B(1,0,0,0);
	Eigen::Vector3d v_A_B(1,0,0);

	tf::Transform tf_A_B;
	tf::Quaternion tf_q_A_B(q_A_B.x(),q_A_B.y(),q_A_B.z(),q_A_B.w());
	tf::Vector3 tf_v_A_B(v_A_B(0),v_A_B(1),v_A_B(2));
	
	tf_A_B.setOrigin(tf_v_A_B);
	tf_A_B.setRotation(tf_q_A_B);

	br.sendTransform(tf::StampedTransform(tf_A_B,
											ros::Time::now(),
											"A",
											"B"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
