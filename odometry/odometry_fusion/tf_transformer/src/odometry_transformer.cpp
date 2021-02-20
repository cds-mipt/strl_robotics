#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <boost/bind.hpp>


tf2_ros::Buffer tfBuffer;
ros::Publisher pub;

bool reset_odom = false;
bool first_pose = true; 
Eigen::Matrix4d first_pose_inv;


void inverse(Eigen::Matrix4d& in, Eigen::Matrix4d& out) 
{
	// set result matrix to identity matrix
	out.setIdentity();
	// get rotation matrix
	Eigen::Matrix3d rot;
	rot = in.block(0, 0, 3, 3);
	// inverse rotation matrix
	Eigen::Matrix3d rot_inverse;
	rot_inverse = rot.transpose();
	// set inversed rotation matrix
	out.block(0, 0, 3, 3) = rot_inverse;

	// get translation
	Eigen::Vector3d t;
	t = in.block(0, 3, 3, 1);
	// inverse translation
	Eigen::Vector3d t_inv;
	t_inv = -1 * rot * t;
	// set inversed translation
	out.block(0, 3, 3, 1) = t_inv;
}


void transform_to_matrix(const geometry_msgs::TransformStamped& transformStamped, Eigen::Matrix4d& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t.x() = transformStamped.transform.translation.x;
	t.y() = transformStamped.transform.translation.y;
	t.z() = transformStamped.transform.translation.z;

	Eigen::Quaterniond q;
	q.x() = transformStamped.transform.rotation.x;
	q.y() = transformStamped.transform.rotation.y;
	q.z() = transformStamped.transform.rotation.z;
	q.w() = transformStamped.transform.rotation.w;

	matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();
	matrix.block(0,3,3,1) = t;
}


void pose_to_matrix(const geometry_msgs::Pose& pose, Eigen::Matrix4d& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t.x() = pose.position.x;
	t.y() = pose.position.y;
	t.z() = pose.position.z;

	Eigen::Quaterniond q;
	q.x() = pose.orientation.x;
	q.y() = pose.orientation.y;
	q.z() = pose.orientation.z;
	q.w() = pose.orientation.w;

	matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();
	matrix.block(0,3,3,1) = t;
}


void matrix_to_pose(const Eigen::Matrix4d& matrix, geometry_msgs::Pose& pose) 
{
	pose.position.x = matrix(0, 3);
	pose.position.y = matrix(1, 3);
	pose.position.z = matrix(2, 3);

	Eigen::Matrix3d rot;
	rot = matrix.block(0, 0, 3, 3);
	Eigen::Quaterniond q(rot);

	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
}


void transfrom_to_twist_matrix(const Eigen::Matrix4d& transfrom_matrix_inv, Eigen::MatrixXd& matrix) 
{
	matrix.setIdentity();

	Eigen::Vector3d t;
	t = transfrom_matrix_inv.block(0, 3, 3, 1);

	Eigen::Matrix3d t_hat;
	t_hat << 0, -t(2), t(1),
			 t(2), 0, -t(0),
    		-t(1), t(0), 0;


	Eigen::Matrix3d rot;
	rot = transfrom_matrix_inv.block(0, 0, 3, 3);

	matrix.block(0,0,3,3) = rot;
	matrix.block(3,3,3,3) = rot;
	matrix.block(0,3,3,3) = t_hat * rot;
}


void twist_to_vector(const geometry_msgs::Twist& twist, Eigen::VectorXd& twist_vector) 
{
	twist_vector(0) = twist.linear.x;
	twist_vector(1) = twist.linear.y;
	twist_vector(2) = twist.linear.z;

	twist_vector(3) = twist.angular.x;
	twist_vector(4) = twist.angular.y;
	twist_vector(5) = twist.angular.z;
}


void vector_to_twist(const Eigen::VectorXd& twist_vector, geometry_msgs::Twist& twist) 
{
	twist.linear.x = twist_vector(0);
	twist.linear.y = twist_vector(1);
	twist.linear.z = twist_vector(2);

	twist.angular.x = twist_vector(3);
	twist.angular.y = twist_vector(4);
	twist.angular.z = twist_vector(5);
}


void odomCallback(const nav_msgs::Odometry& msg) 
{
	nav_msgs::Odometry msg_result;

	std::string from_frame, to_frame;
	ros::param::get("~from_frame", from_frame);
	ros::param::get("~to_frame", to_frame);

	geometry_msgs::TransformStamped transformStamped;
	try
 	{
    	transformStamped = tfBuffer.lookupTransform(from_frame, to_frame, ros::Time(0));

    	// convert transform to matrix 
    	Eigen::Matrix4d transfrom_matrix;
    	transform_to_matrix(transformStamped, transfrom_matrix);
    	// get inverse transform matrix
    	Eigen::Matrix4d transfrom_matrix_inv;
    	inverse(transfrom_matrix, transfrom_matrix_inv);

    	// --------------------- pose transform --------------------- 
    	// convert pose to matrix
    	Eigen::Matrix4d pose_matrix;
    	pose_to_matrix(msg.pose.pose, pose_matrix);

    	// apply transfrom on pose
    	Eigen::Matrix4d pose_matrix_result;
    	pose_matrix_result = transfrom_matrix_inv * pose_matrix * transfrom_matrix;

    	// reset odom if needed
    	if (reset_odom) {
    		if (first_pose) {
    			inverse(pose_matrix_result, first_pose_inv);
    			first_pose = false;
    		}
    		pose_matrix_result = first_pose_inv * pose_matrix_result;
    	}

    	matrix_to_pose(pose_matrix_result, msg_result.pose.pose);

    	// --------------------- twist transform ---------------------
    	// convert transform to twist_transfrom
    	Eigen::MatrixXd twist_transfrom(6, 6);
    	transfrom_to_twist_matrix(transfrom_matrix_inv, twist_transfrom);

    	// convert twist to vector
    	Eigen::VectorXd twist_vector(6);
    	twist_to_vector(msg.twist.twist, twist_vector);

    	// apply transfrom on twist
    	Eigen::VectorXd twist_vector_result(6);
    	twist_vector_result = twist_transfrom * twist_vector;
    	vector_to_twist(twist_vector_result, msg_result.twist.twist);

    	// --------------------- header replacement ---------------------
    	// copy header
    	msg_result.header = msg.header;
    	msg_result.header.frame_id = "odom";
    	msg_result.child_frame_id = to_frame;

        // write pose noise covariance if needed
        if(ros::param::has("~covariance")) 
        {
            std::vector<double> covariance;
            ros::param::get("~covariance", covariance);
            for (int i = 0; i < 36 && i < covariance.size(); ++i) msg_result.pose.covariance[i] = covariance[i];
        }
    	// publish odometry
    	pub.publish(msg_result);
 	}
 	catch (tf2::TransformException &ex) 
 	{
   		ROS_WARN("%s",ex.what());
 	}
}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "odometry_transformer");
	ros::NodeHandle node;

	std::string in_topic, out_topic;
	ros::param::get("~in_topic", in_topic);
	ros::param::get("~out_topic", out_topic);
        ros::param::get("~reset_odom", reset_odom);

	tf2_ros::TransformListener tfListener(tfBuffer);

 	pub = node.advertise<nav_msgs::Odometry>(out_topic, 10);
 	ros::Subscriber sub = node.subscribe(in_topic, 10, odomCallback);

 	ros::spin();

	return 0;
}
