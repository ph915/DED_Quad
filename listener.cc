/*
 * Listener Script - Simultaneous Gazebo Subscriber and ROS Publisher
 * Author: Pablo Hermoso Moreno
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include "gazebo_plugins/gazebo_ros_gpu_laser.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>


ros::Publisher tera_2_pub_rplidar;
ros::Publisher tera_1_pub_up;
ros::Publisher tera_1_pub_down;

typedef const boost::shared_ptr<const gazebo::msgs::LaserScanStamped> ConstLaserScanStampedPtr;

void cb_rplidar(ConstLaserScanStampedPtr &_msg)
{

   // We got a new message from the Gazebo sensor.  Create Corresponding ROS message and publish it.
   sensor_msgs::LaserScan laser_msg;
   laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
   //laser_msg.header.frame_id = this->frame_name_;
   laser_msg.angle_min = _msg->scan().angle_min();
   laser_msg.angle_max = _msg->scan().angle_max();
   laser_msg.angle_increment = _msg->scan().angle_step();
   laser_msg.time_increment = 0;  // instantaneous simulator scan
   laser_msg.scan_time = 0;  // not sure whether this is correct
   laser_msg.range_min = _msg->scan().range_min();
   laser_msg.range_max = _msg->scan().range_max();
   laser_msg.ranges.resize(_msg->scan().ranges_size());
   std::copy(_msg->scan().ranges().begin(),
             _msg->scan().ranges().end(),
             laser_msg.ranges.begin());
   laser_msg.intensities.resize(_msg->scan().intensities_size());
   std::copy(_msg->scan().intensities().begin(),
                _msg->scan().intensities().end(),
             laser_msg.intensities.begin());
   //this->pub_queue_->push(laser_msg, this->pub_);
   
   //std::cout << laser_msg.range_max->DebugString();
   
   // Publish in ROS topic
   tera_2_pub_rplidar.publish(laser_msg);
 
}


void cb_up(ConstLaserScanStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();
  
   // We got a new message from the Gazebo sensor.  Stuff a
   // corresponding ROS message and publish it.
   sensor_msgs::LaserScan laser_msg;
   laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
   //laser_msg.header.frame_id = this->frame_name_;
   laser_msg.angle_min = _msg->scan().angle_min();
   laser_msg.angle_max = _msg->scan().angle_max();
   laser_msg.angle_increment = _msg->scan().angle_step();
   laser_msg.time_increment = 0;  // instantaneous simulator scan
   laser_msg.scan_time = 0;  // not sure whether this is correct
   laser_msg.range_min = _msg->scan().range_min();
   laser_msg.range_max = _msg->scan().range_max();
   laser_msg.ranges.resize(_msg->scan().ranges_size());
   std::copy(_msg->scan().ranges().begin(),
             _msg->scan().ranges().end(),
             laser_msg.ranges.begin());
   laser_msg.intensities.resize(_msg->scan().intensities_size());
   std::copy(_msg->scan().intensities().begin(),
                _msg->scan().intensities().end(),
             laser_msg.intensities.begin());
   //this->pub_queue_->push(laser_msg, this->pub_);
   
   //std::cout << laser_msg.range_max->DebugString();
   
   // Publish in ROS topic
   tera_1_pub_up.publish(laser_msg);
 
}


void cb_down(ConstLaserScanStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();
  
   // We got a new message from the Gazebo sensor.  Stuff a
   // corresponding ROS message and publish it.
   sensor_msgs::LaserScan laser_msg;
   laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
   //laser_msg.header.frame_id = this->frame_name_;
   laser_msg.angle_min = _msg->scan().angle_min();
   laser_msg.angle_max = _msg->scan().angle_max();
   laser_msg.angle_increment = _msg->scan().angle_step();
   laser_msg.time_increment = 0;  // instantaneous simulator scan
   laser_msg.scan_time = 0;  // not sure whether this is correct
   laser_msg.range_min = _msg->scan().range_min();
   laser_msg.range_max = _msg->scan().range_max();
   laser_msg.ranges.resize(_msg->scan().ranges_size());
   std::copy(_msg->scan().ranges().begin(),
             _msg->scan().ranges().end(),
             laser_msg.ranges.begin());
   laser_msg.intensities.resize(_msg->scan().intensities_size());
   std::copy(_msg->scan().intensities().begin(),
                _msg->scan().intensities().end(),
             laser_msg.intensities.begin());
   //this->pub_queue_->push(laser_msg, this->pub_);
   
   //std::cout << laser_msg.range_max->DebugString();
   
   // Publish in ROS topic
   tera_1_pub_down.publish(laser_msg);
 
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  
  ros::init(_argc, _argv, "tera_2_node");
  ros::NodeHandle n;
  
  tera_2_pub_rplidar = n.advertise<sensor_msgs::LaserScan>("tera_2_array", 1000);
  tera_1_pub_up = n.advertise<sensor_msgs::LaserScan>("tera_1_up", 1000);
  tera_1_pub_down = n.advertise<sensor_msgs::LaserScan>("tera_1_down", 1000);
  ros::Rate loop_rate(20);
  
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  // gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
  gazebo::transport::SubscriberPtr sub_rplidar = node->Subscribe("~/iris_opt_flow/rplidar/link/laser/scan",cb_rplidar);
  gazebo::transport::SubscriberPtr sub_up = node->Subscribe("~/iris_opt_flow/lidar2/link/laser/scan",cb_up);
  gazebo::transport::SubscriberPtr sub_down = node->Subscribe("~/iris_opt_flow/lidar/link/laser/scan",cb_down);
    

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
  
  
  ros::spinOnce();
  loop_rate.sleep();
  
}
