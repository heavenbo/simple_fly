#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandSetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetTFListen.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
    ros::ServiceClient tf_listen_client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    // send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::CommandSetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client.call(offb_set_mode);

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;
    // arming_client.call(arm_cmd);

    mavros_msgs::SetTFListen tf_listen_cmd;
    tf_listen_cmd.request.value = true;
    tf_listen_client.call(tf_listen_cmd);

    ros::Time last_request = ros::Time::now();
    ROS_INFO("begin");
    // while (!current_state.armed || current_state.mode != "OFFBOARD" || !arm_cmd.response.success)
    while (!tf_listen_cmd.response.result)
    {
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("open failed!");
            return 1;
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    ROS_INFO("open!");
    tf_listen_cmd.request.value = false;
    tf_listen_client.call(tf_listen_cmd);

    // while (ros::ok())
    // {
    //     local_pos_pub.publish(pose);

    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}