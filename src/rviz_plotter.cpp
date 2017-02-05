#include <ros/ros.h>
#include "RVIZ_markers.h"
#include "vicon_bridge/Markers.h"
#include <ros/console.h>

ros::Publisher pub;
visualization_msgs::Marker joints;//points as joints
RVIZ_Markers m;
geometry_msgs::Point local_joint;
void callback(const vicon_bridge::Markers& data)
{
joints.points.resize(data.markers.size());

m.points(joints);
for(int i=0;i<data.markers.size()-1;i++)
{
	joints.points.at(i).x=data.markers[i].translation.x/100;joints.points.at(i).y=data.markers[i].translation.y/100;joints.points.at(i).z=data.markers[i].translation.z/400;
}
// ROS_INFO_STREAM(data.markers.size());
pub.publish(joints);

}
int main(int argc, char **argv)
{
  	ros::init (argc, argv, "rviz_visualizations");
  	ros::NodeHandle nh("~");
 //  	local_joint.x=0;local_joint.y=0;local_joint.z=0;
	std_msgs::ColorRGBA color;
	color.a=1;color.b=1;color.r=0;color.g=0;
	joints.color=color;
	// m.add_point(joints,local_joint,color);
  	ros::Subscriber markers_subscriber=nh.subscribe("/vicon/markers",1,callback);
  	pub=nh.advertise<visualization_msgs::Marker>("/joints",100);
  	ros::spin();
}