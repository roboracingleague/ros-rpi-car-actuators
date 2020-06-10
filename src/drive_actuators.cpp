#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <pigpiod_if2.h>

int pi;

// actuators callback
void drive_callback(const geometry_msgs::Twist twist)
{
    set_servo_pulsewidth(pi, 23, 1500 + 500 * twist.linear.x);
    set_servo_pulsewidth(pi, 24, 1500 + 500 * twist.angular.z);
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
    ros::Subscriber steering = n.subscribe("/actuator/drive", 1, drive_callback);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    return 0;
}

