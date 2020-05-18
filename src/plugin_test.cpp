/* Write a ROS publisher to test the aerodynamics pluging. This publisher should be able to publish the desired actuator values at once (Gradually or At once we'll see)*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <vector>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
void publish_inputs(float value[]) {
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plugin_test");

    ros::NodeHandle n;

    ros::Publisher actuator_pub[4];
    actuator_pub[0] = n.advertise<std_msgs::Float32>("/uav1/rotor_left_joint", 1);
    actuator_pub[1] = n.advertise<std_msgs::Float32>("/uav1/rotor_right_joint", 1);
    actuator_pub[2] = n.advertise<std_msgs::Float32>("/uav1/left_elevon_joint", 1);
    actuator_pub[3] = n.advertise<std_msgs::Float32>("/uav1/right_elevon_joint", 1);

    ros::Rate loop_rate(10);

    std_msgs::Float32 inputs[4];
    inputs[0].data = -5550;
    inputs[1].data = 5050;
    inputs[2].data = -0.2;
    inputs[3].data = 0.0;
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        // ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        for (int i = 0; i < 4; i++) {
            actuator_pub[i].publish(inputs[i]);
        }
        // chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
