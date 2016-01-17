/*********************************************************************
* fake_gravity_imu_node.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_gravity_ft_node");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor", 1000);
  tf::TransformListener listener;
  geometry_msgs::Vector3Stamped force;
  geometry_msgs::Vector3Stamped torque;
  std::string source_frame, target_frame;
  double mass, distance_y;
  double gravity = 9.81;
  nh.getParam("source_frame", source_frame);
  nh.getParam("target_frame", target_frame);
  nh.getParam("mass", mass);
  nh.getParam("distance_y", distance_y);
  force.header.frame_id = source_frame;
  force.vector.z = - gravity * mass;
  tf::Vector3 distance(0, distance_y, 0);
  listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5));
  ros::Rate r(100);
  while(ros::ok()) {
    geometry_msgs::Vector3Stamped force_tool, torque_tool;
    try {
      listener.transformVector(target_frame, force, force_tool);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::WrenchStamped msg;
    msg.header= force_tool.header;
    msg.wrench.force = force_tool.vector;
    tf::Vector3 tf_force;
    tf::vector3MsgToTF(msg.wrench.force, tf_force);
    tf::Vector3 tf_torque;
    tf_torque = distance.cross(tf_force);
    tf::vector3TFToMsg(tf_torque, msg.wrench.torque);
    pub.publish(msg);
    r.sleep();
  }
  return 0;
}
