#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <pigpiod_if2.h>

int pi;

// actuators callback
void steering_callback(const std_msgs::Float64 msg)
{
    set_servo_pulsewidth(pi, 24, 1500 + 500 * msg.data);
}

void throttle_callback(const std_msgs::Float64 msg)
{
    set_servo_pulsewidth(pi, 23, 1500 + 500 * msg.data);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "drive_actuators");
    ros::NodeHandle n;

    // Init GPIO
    pi = pigpio_start(NULL, NULL);

    set_servo_pulsewidth(pi, 23, 1500);
    set_servo_pulsewidth(pi, 24, 1500);

    // Subscribe to /actuator/drive
    ros::Subscriber steering = n.subscribe("/actuator/steering", 1, steering_callback);
    ros::Subscriber throttle = n.subscribe("/actuator/throttle", 1, throttle_callback);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    return 0;
}

