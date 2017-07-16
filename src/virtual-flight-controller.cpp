/*
 * virtual-flight-controller.cpp
 *
 *  Created on: May 22, 2017
 *      Author: kevin
 */

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo_msgs/ApplyBodyWrench.h>

#include <tf/transform_broadcaster.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <std_msgs/Float64MultiArray.h>

#include "../include/pauvsi_trajectory/Physics.h"
#include "../include/pauvsi_trajectory/Polynomial.hpp"
#include "../include/pauvsi_trajectory/Simulation.h"

#include <eigen3/Eigen/Geometry>

#define USE_GAZEBO true

void updateForces(const std_msgs::Float64MultiArrayConstPtr msg);

void physicsUpdate(double dt);

void publishState();

bool started;

Eigen::Vector4d forces;

Eigen::Matrix3d J;
Eigen::Matrix4d torque_transition;

double mass;

double max_motor_force, min_motor_force;

//these keep track of the state of the quad
Eigen::Vector3d pos, vel;
Eigen::Quaterniond attitude;
Eigen::Vector3d omega;

boost::mt19937 rng;

#if USE_GAZEBO
ros::ServiceClient wrench_client;
#else
ros::Publisher pose_pub, twist_pub;
#endif
ros::Subscriber motor_force_sub;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "virtual_flight_controller"); // initializes with a randomish name

	ros::NodeHandle nh;

#if USE_GAZEBO
	wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
#else
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
	twist_pub = nh.advertise<geometry_msgs::TwistStamped>(TWIST_TOPIC, 1);
#endif

	motor_force_sub = nh.subscribe(MOTOR_FORCE_TOPIC, 1, updateForces);

	started = false;

	//get physics constants
	J << J_MATRIX;
	torque_transition << TORQUE_TRANSITION;
	mass = MASS;
	min_motor_force = MOTOR_ABS_MIN;
	max_motor_force = MOTOR_ABS_MAX;

	rng.seed(RANDOM_SEED);

	pos << START_POS;
	attitude.w() = 1.0;
	attitude.x() = 0.0;
	attitude.y() = 0.0;
	attitude.z() = 0.0;

	/*//testing
	forces << 12.5, 12.5, 12.8, 12.5;
	started = true;*/

	ros::Rate loop_rate(1/PHYSICS_UPDATE_DT);
	while(ros::ok())
	{
		if(started){ physicsUpdate(PHYSICS_UPDATE_DT); }
#if !USE_GAZEBO
		publishState();
#endif
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}

void publishState()
{
#if !USE_GAZEBO
	boost::normal_distribution<> nd(0.0, POSITION_SIGMA);

	boost::variate_generator<boost::mt19937&,
	boost::normal_distribution<> > var_nor(rng, nd);

	Eigen::Vector3d pub_pos;
	pub_pos << var_nor(), var_nor(), var_nor();
	pub_pos += pos;

	boost::normal_distribution<> nd2(0.0, VEL_SIGMA);

	boost::variate_generator<boost::mt19937&,
	boost::normal_distribution<> > var_nor2(rng, nd2);

	Eigen::Vector3d pub_vel;
	pub_vel << var_nor2(), var_nor2(), var_nor2();
	pub_vel += vel;

	boost::normal_distribution<> nd3(0.0, OMEGA_SIGMA);

	boost::variate_generator<boost::mt19937&,
	boost::normal_distribution<> > var_nor3(rng, nd3);

	Eigen::Vector3d pub_omega;
	pub_omega << var_nor3(), var_nor3(), var_nor3();
	pub_omega += omega;

	boost::normal_distribution<> nd4(0.0, QUAT_SIGMA);

	boost::variate_generator<boost::mt19937&,
	boost::normal_distribution<> > var_nor4(rng, nd4);

	Eigen::Quaterniond pub_quat;
	pub_quat.w() = attitude.w() + var_nor4();
	pub_quat.x() = attitude.x() + var_nor4();
	pub_quat.y() = attitude.y() + var_nor4();
	pub_quat.z() = attitude.z() + var_nor4();

	pub_quat.normalize();

	geometry_msgs::PoseStamped pose;
	geometry_msgs::TwistStamped twist;

	pose.pose.orientation.w = pub_quat.w();
	pose.pose.orientation.x = pub_quat.x();
	pose.pose.orientation.y = pub_quat.y();
	pose.pose.orientation.z = pub_quat.z();

	pose.pose.position.x = pub_pos.x();
	pose.pose.position.y = pub_pos.y();
	pose.pose.position.z = pub_pos.z();

	twist.twist.linear.x = pub_vel.x();
	twist.twist.linear.y = pub_vel.y();
	twist.twist.linear.z = pub_vel.z();

	twist.twist.angular.x = pub_omega.x();
	twist.twist.angular.y = pub_omega.y();
	twist.twist.angular.z = pub_omega.z();

	pose.header.stamp = ros::Time::now();
	twist.header.stamp = ros::Time::now();
	pose_pub.publish(pose);
	twist_pub.publish(twist);

	//send a transform for visualization and other algorithms
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
	transform.setRotation(tf::Quaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w()));

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), WORLD_FRAME, BASE_FRAME));
#endif
}

void updateForces(const std_msgs::Float64MultiArrayConstPtr msg){
	ROS_ASSERT(msg->layout.dim[0].size == 4); // make sure that there is 4 elems in this data
	forces(0) = msg->data[0];
	forces(1) = msg->data[1];
	forces(2) = msg->data[2];
	forces(3) = msg->data[3];
	started = true;

	if(forces(0) > max_motor_force){
		forces(0) = max_motor_force;
		ROS_WARN("constrained force");
	}
	else if(forces(0) < min_motor_force){
		forces(0) = min_motor_force;
		ROS_WARN("constrained force");
	}

	if(forces(1) > max_motor_force){
		forces(1) = max_motor_force;
		ROS_WARN("constrained force");
	}
	else if(forces(1) < min_motor_force){
		forces(1) = min_motor_force;
		ROS_WARN("constrained force");
	}

	if(forces(2) > max_motor_force){
		forces(2) = max_motor_force;
		ROS_WARN("constrained force");
	}
	else if(forces(2) < min_motor_force){
		forces(2) = min_motor_force;
		ROS_WARN("constrained force");
	}

	if(forces(3) > max_motor_force){
		forces(3) = max_motor_force;
		ROS_WARN("constrained force");
	}
	else if(forces(3) < min_motor_force){
		forces(3) = min_motor_force;
		ROS_WARN("constrained force");
	}

	//physicsUpdate(0.01);
	//publishState();
}

void physicsUpdate(double dt)
{
	Eigen::Vector3d F_b, gravity, moment;

	Eigen::Vector4d torque_force = torque_transition * forces;

	gravity << 0,0,G;

	F_b << 0, 0, torque_force(0);
	moment << torque_force(1), torque_force(2), torque_force(3);

#if USE_GAZEBO
	gazebo_msgs::ApplyBodyWrench srv;
	srv.request.body_name = "quadrotor::base_link";
	srv.request.duration = ros::Duration(-1.0);
	srv.request.reference_frame = "world";
	srv.request.wrench.force.x = 0;
	srv.request.wrench.force.x = 0;
	srv.request.wrench.force.x = torque_force(0);
	srv.request.wrench.torque.x = torque_force(1);
	srv.request.wrench.torque.x = torque_force(2);
	srv.request.wrench.torque.x = torque_force(3);

	if(wrench_client.call(srv))
	{
		if(!srv.response.success)
		{
			ROS_ERROR_STREAM("call response fail: " << srv.response.status_message);
		}
	}
	else
	{
		ROS_ERROR("service call failed");
	}
#else
	//ROS_DEBUG_STREAM("total force: " << torque_force(0));

	Eigen::Vector3d accel = (1.0/mass)*(attitude * F_b) - gravity;

	Eigen::Vector3d alpha = J.inverse() * (moment - omega.cross(J*omega));

	Eigen::Vector3d theta = (0.5 * alpha * dt*dt + omega * dt);

	omega = omega + alpha * dt;

	Eigen::Quaterniond delta = Eigen::AngleAxisd(theta(0), Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(theta(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(theta(2), Eigen::Vector3d::UnitZ());

	attitude = attitude * delta;

	pos = 0.5 * accel * dt * dt + vel * dt + pos;

	vel = vel + accel * dt;

	ROS_INFO_STREAM("actual accel: " << accel.transpose());
#endif

}

