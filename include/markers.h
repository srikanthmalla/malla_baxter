//author: srikanth malla
//markers.h file
#ifndef MARKERS_H_
#define MARKERS_H_
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class Markers
{
public:
	void triangle_list(visualization_msgs::Marker &m);
	void line_list(visualization_msgs::Marker &m);
	void line_strip(visualization_msgs::Marker &m);
	void add_point(visualization_msgs::Marker &m, geometry_msgs::Point &p, std_msgs::ColorRGBA &color);
	void points(visualization_msgs::Marker &m);
    void cylinder(visualization_msgs::Marker &m);
    void arrow(visualization_msgs::Marker &m);
};
void Markers::triangle_list(visualization_msgs::Marker &m)
{
	m.header.frame_id = "/base"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "triangle_list";
    m.id = 0;
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
}
void Markers::add_point(visualization_msgs::Marker &m, geometry_msgs::Point &p, std_msgs::ColorRGBA &color)
{
	m.points.push_back(p);
	m.colors.push_back(color);
}
void Markers::line_list(visualization_msgs::Marker &m)
{
	m.header.frame_id = "/world"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "line_list";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.scale.z = 0.01;
}
void Markers::line_strip(visualization_msgs::Marker &m)
{
	m.header.frame_id = "/world"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "line_strip";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
}
void Markers::points(visualization_msgs::Marker &m)
{
	m.header.frame_id = "/right_wrist"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "line_strip";
    m.id = 0;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;	
}
void Markers::cylinder(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/world"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "cylinder";
    m.id = 0;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 5.0;
    m.pose.position.y = 5.0;
    m.pose.position.z = 3.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 2;
    m.scale.y = 2;
    m.scale.z = 3;    
}
void Markers::arrow(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/world"; // TODO
    m.header.stamp = ros::Time();
    m.ns = "arrow";
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;  
}

#endif