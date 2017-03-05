#include <ros/ros.h>
#include "RVIZ_markers.h"
#include "vicon_bridge/Markers.h"
#include <ros/console.h>


ros::Publisher pub_joints,pub_link1, pub_link2, pub_link3;
visualization_msgs::Marker joints, link1, link2, link3;//points as joints, lines as links
RVIZ_Markers m;
geometry_msgs::Point local_joint;
std::string link1_name="static", link2_name="dynamic", link3_name="static";
void callback(const vicon_bridge::Markers& data)
{
joints.points.resize(data.markers.size());
int l1=0,l2=0,l3=0;
for(int i=0;i<data.markers.size()-1;i++)
{
	if (data.markers[i].segment_name==link1_name)
	{
		l1++;
	}
	if (data.markers[i].segment_name==link2_name)
	{
		l2++;
	}
	if (data.markers[i].segment_name==link3_name)
	{
		l3++;
	}
}
link1.points.resize(l1);
link2.points.resize(l2);
link3.points.resize(l3);
// links.points.resize(2*(data.markers.size()-1));
m.points(joints);
m.line_list(link1);m.line_list(link2);
ROS_INFO_STREAM(data.markers.size());
for(int i=0;i<data.markers.size()-1;i++)
{
	joints.points.at(i).x=data.markers[i].translation.x/100;joints.points.at(i).y=data.markers[i].translation.y/100;joints.points.at(i).z=data.markers[i].translation.z/200;
	// ROS_INFO_STREAM(data.markers[i].segment_name);
}
// links.points.at(0)=joints.points.at(0);
int j=1;
l1=0;l2=0;l3=0;
for(int i=0;i<joints.points.size()-1;i++)
{
	if (data.markers[i].segment_name==link1_name)
	{
		link1.points.at(l1)=joints.points.at(i);
		l1++;
	}
	if (data.markers[i].segment_name==link2_name)
	{
		link2.points.at(l2)=joints.points.at(i);
		l2++;
	}
	if (data.markers[i].segment_name==link3_name)
	{
		link3.points.at(l3)=joints.points.at(i);
		l3++;
	}
	// links.points.at(j)=joints.points.at(i);
	// links.points.at(j+1)=joints.points.at(i);
	// j=j+2;
}

// links.points.at(j)=joints.points.at(joints.points.size()-1);
// ROS_INFO_STREAM(data.markers.size());
pub_joints.publish(joints);
pub_link1.publish(link1);
pub_link2.publish(link2);
pub_link3.publish(link3);
}
int main(int argc, char **argv)
{
  	ros::init (argc, argv, "rviz_visualizations");
  	ros::NodeHandle nh("~");
	std_msgs::ColorRGBA color;
	color.a=1;color.b=1;color.r=0;color.g=0;
	joints.color=color;
	color.a=1;color.g=1;color.r=1;color.b=0;
	link1.color=color;
	color.a=1;color.g=0;color.r=1;color.b=0;
	link2.color=color;
	color.a=1;color.g=0;color.r=1;color.b=1;
	link3.color=color;
	// m.add_point(joints,local_joint,color);
  	ros::Subscriber markers_subscriber=nh.subscribe("/vicon/markers",1,callback);
  	pub_joints=nh.advertise<visualization_msgs::Marker>("/joints",100);
  	pub_link1=nh.advertise<visualization_msgs::Marker>("/link1",100);
  	pub_link2=nh.advertise<visualization_msgs::Marker>("/link2",100);
  	pub_link3=nh.advertise<visualization_msgs::Marker>("/link3",100);
  	ros::spin();
}