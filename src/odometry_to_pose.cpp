#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace ros;
using namespace std;

class Changer
{
private:
	NodeHandle nh_;
	Publisher pub_;
	Subscriber sub_;

public:
	// constructor
	Changer(const string& subscribed_topic, const string& published_topic)
	{
		pub_ = nh_.advertise<geometry_msgs::PoseStamped>(published_topic,10);
		sub_ = nh_.subscribe(subscribed_topic,10,&Changer::OdometryCallback,this);
	}
	void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header = msg->header;
		pose_msg.pose = msg->pose.pose;
		pub_.publish(pose_msg);
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
