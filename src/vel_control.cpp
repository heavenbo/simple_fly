#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>

geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_fly");
  ros::NodeHandle n;
  // ros::NodeHandle nh;

  ros::Publisher vel_sp_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
  ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  ros::Rate rate(20.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::CommandSetMode offb_set_mode;
  // srv_setmode.request.base_mode = 0;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  set_mode_client.call(offb_set_mode);

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  arming_client.call(arm_cmd);

  ros::Time last_request = ros::Time::now();
  while (ros::ok() && (!current_state.armed || current_state.mode != "OFFBOARD"))
  {
    if (ros::Time::now() - last_request < ros::Duration(10.0))
    {
      ros::spinOnce();
      rate.sleep();
    }
    else
    {
      ROS_INFO("failed");
      return 1;
    }
  }

  vs.twist.linear.x = 0.0;
  vs.twist.linear.y = 0.0;
  vs.twist.linear.z = 0.0;
  vs.twist.angular.x = 0.0;
  vs.twist.angular.y = 0.0;
  vs.twist.angular.z = 0.0;

  vs_body_axis.twist.linear.x = 0.0;
  vs_body_axis.twist.linear.y = 0.0;
  vs_body_axis.twist.linear.z = 0.0;
  vs_body_axis.twist.angular.x = 0.0;
  vs_body_axis.twist.angular.y = 0.0;
  vs_body_axis.twist.angular.z = 0.0;

  last_request = ros::Time::now();
  while (ros::ok())
  {
    if (ros::Time::now() - last_request < ros::Duration(5.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }

    if (ros::Time::now() - last_request >= ros::Duration(5.0) && ros::Time::now() - last_request < ros::Duration(10.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0.3;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }

    if (ros::Time::now() - last_request >= ros::Duration(10.0) && ros::Time::now() - last_request < ros::Duration(15.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }

    if (ros::Time::now() - last_request >= ros::Duration(15.0) && ros::Time::now() - last_request < ros::Duration(20.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0.2;
    }

    if (ros::Time::now() - last_request >= ros::Duration(20.0) && ros::Time::now() - last_request < ros::Duration(25.0))
    {
      vs.twist.linear.x = 0.2;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }

    if (ros::Time::now() - last_request >= ros::Duration(25.0) && ros::Time::now() - last_request < ros::Duration(30.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0.2;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }

    if (ros::Time::now() - last_request >= ros::Duration(30.0) && ros::Time::now() - last_request < ros::Duration(35.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = -0.3;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }
    if (ros::Time::now() - last_request >= ros::Duration(35.0) && ros::Time::now() - last_request < ros::Duration(40.0))
    {
      vs.twist.linear.x = 0;
      vs.twist.linear.y = 0;
      vs.twist.linear.z = 0;
      vs.twist.angular.x = 0;
      vs.twist.angular.y = 0;
      vs.twist.angular.z = 0;
    }
    if (ros::Time::now() - last_request >= ros::Duration(40.0))
    {
      arm_cmd.request.value = false;
      arming_client.call(arm_cmd);
      break;
    }
    vs_body_axis.twist.linear.x = vs.twist.linear.x;
    vs_body_axis.twist.linear.y = vs.twist.linear.y;
    vs_body_axis.twist.linear.z = vs.twist.linear.z;
    vs_body_axis.twist.angular.x = vs.twist.angular.x;
    vs_body_axis.twist.angular.y = vs.twist.angular.y;
    vs_body_axis.twist.angular.z = vs.twist.angular.z;
    vs_body_axis.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs_body_axis);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}