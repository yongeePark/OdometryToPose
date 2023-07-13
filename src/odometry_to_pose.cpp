#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace ros;
using namespace std;
using namespace Eigen;

class Changer
{
private:
	NodeHandle nh_;
	Publisher pub_;
	Subscriber sub_;
	
	bool use_t265_;

public:
	// constructor
	Changer(const string& subscribed_topic, const string& published_topic)
	{
		pub_ = nh_.advertise<geometry_msgs::PoseStamped>(published_topic,10);
		sub_ = nh_.subscribe(subscribed_topic,10,&Changer::OdometryCallback,this);

		ros::NodeHandle nhp("~");
		nhp.param("use_t265", use_t265_, false);

	}
	// Callback
	// It could also conduct rotation
	void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		if(!use_t265_)
		{

			geometry_msgs::PoseStamped pose_msg;
			pose_msg.header.stamp = ros::Time::now(); 
			pose_msg.header.frame_id = "world";

			//position transform
			Vector3d initial_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
			
			Eigen::Matrix3d rotation_z_m90;
			rotation_z_m90= AngleAxisd(-M_PI/2,Vector3d::UnitZ());

			Vector3d rotated_position = rotation_z_m90* initial_position;

			// rotation transform
			// quaternion order: w,x,y,z
			//
			// q_initial : real camera based
			Quaterniond q_initial(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

			Matrix3d matrix_initial = q_initial.matrix();

			// q_base : camera frame based
			// default : x 90 degree
			// camera angle : x 5 degree tilted
			Quaterniond q_base(AngleAxisd(0*M_PI/180,Vector3d::UnitX()));
			Quaterniond q_camera_tilt(AngleAxisd(M_PI/2,Vector3d::UnitX()));
			Matrix3d matrix_q_base = q_base.matrix() * q_camera_tilt.matrix() *  matrix_initial;

			Matrix3d rotate_cam_to_world;
			rotate_cam_to_world << 0, 0, 1,
					      -1, 0, 0,
					       0,-1, 0;

			// rotate
			//Matrix3d matrix_rotated = rotation_z_90 * q_matrix;
			//Matrix3d matrix_rotated = Matrix3d::Identity() * q_matrix;
			Matrix3d matrix_rotated = rotate_cam_to_world * matrix_q_base * rotate_cam_to_world.inverse(); 

			Quaterniond q_rotated(matrix_rotated);
			//Quaterniond q_rotated = q_initial * AngleAxisd(M_PI/2,Vector3d::UnitZ());
			//Quaterniond q_rotated = AngleAxisd(M_PI/2,Vector3d::UnitZ()) * q_initial;
			//Quaterniond q_rotated = q_rot * q_initial * q_rot.conjugate();
			//Quaterniond q_rotated = q_initial * q_rot;

			// publish
			pose_msg.pose.position.x = rotated_position(0);
			pose_msg.pose.position.y = rotated_position(1);
			pose_msg.pose.position.z = rotated_position(2);

			pose_msg.pose.orientation.x = q_rotated.x();
			pose_msg.pose.orientation.y = q_rotated.y();
			pose_msg.pose.orientation.z = q_rotated.z();
			pose_msg.pose.orientation.w = q_rotated.w();

			pub_.publish(pose_msg);
		}
		else // if use t265
		{
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.header.stamp = ros::Time::now(); 
			pose_msg.header.frame_id = msg->header.frame_id;
		
            // if t265 is facing backward
			//position transform
			Vector3d initial_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
			
			Eigen::Matrix3d rotation_offset;
			// rotation_offset = AngleAxisd(-M_PI/2*0,Vector3d::UnitY()); // facing above, y -90
			rotation_offset = AngleAxisd(M_PI,Vector3d::UnitZ()); // facing backward

			Vector3d rotated_position = rotation_offset* initial_position;

			// rotation transform
			// quaternion order: w,x,y,z
			// q_initial : real camera based
			// Quaterniond q_facing(AngleAxisd(M_PI/2,Vector3d::UnitY()));
			Quaterniond q_facing(AngleAxisd(0,Vector3d::UnitY()));

			Quaterniond q_initial(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

			Matrix3d matrix_initial = q_facing.matrix() * q_initial.matrix();

            Matrix3d matrix_offset;
            // facing upward, y -90
            // matrix_offset<< 0, 0,-1,
            //                 0, 1, 0,
            //                 1, 0, 0;

            // facing backward, z 180
            matrix_offset << -1, 0, 0,
                              0, -1,0,
                              0, 0, 1;
			Matrix3d matrix_rotated = matrix_offset * matrix_initial * matrix_offset.inverse(); 


			Quaterniond q_rotated(matrix_rotated);

			// publish
			pose_msg.pose.position.x = rotated_position(0);
			pose_msg.pose.position.y = rotated_position(1);
			pose_msg.pose.position.z = rotated_position(2);

			pose_msg.pose.orientation.x = q_rotated.x();
			pose_msg.pose.orientation.y = q_rotated.y();
			pose_msg.pose.orientation.z = q_rotated.z();
			pose_msg.pose.orientation.w = q_rotated.w();

			//pose_msg.pose = msg->pose.pose;
			pub_.publish(pose_msg);
		}
	}
};

int main(int argc, char** argv)
{
	init(argc,argv,"odometry_to_pose");
	if(argc < 3)
	{
		ROS_ERROR("Usage: odometry_converter sub_topic pub_topic");
		return -1;
	}

	string sub_topic = argv[1];
	string pub_topic = argv[2];
	std::cout<<"===================================="<<endl
		<<"subscribing topic : "<<sub_topic<<endl
		<<"publishing topic : "<<pub_topic<<endl;
	Changer changer(sub_topic,pub_topic);

	std::cout<<"Odometry to Pose node is ready"<<endl;

	ros::spin();
	return 0;
}
