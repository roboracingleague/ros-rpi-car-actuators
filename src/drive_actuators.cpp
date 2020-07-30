#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <pigpiod_if2.h>


class Servo
{
    int pi, gpio, middle = 1500, amplitude = 500;
    double offset = 0.0;

public:
    void update(double value);

    Servo(int in_pi, std::string prefix);
};

void Servo::update(double value)
{
    int width = (value + offset) * amplitude + middle;
    set_servo_pulsewidth(pi, gpio, width);
}

Servo::Servo(int in_pi, std::string prefix)
{
    ros::NodeHandle pH("~" + prefix);

    pi = in_pi;
    pH.getParam("gpio", gpio);
    pH.param("offset", offset, 0.0);

    set_mode(pi, gpio, PI_OUTPUT);

    update(0.0);
}

Servo *steering, *throttle;

// actuators callback
void steering_callback(const std_msgs::Float64 msg)
{
    steering->update(msg.data);
}

void throttle_callback(const std_msgs::Float64 msg)
{
    throttle->update(msg.data);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "drive_actuators");
    ros::NodeHandle n;

    // Init GPIO
    int pi = pigpio_start(NULL, NULL);

    steering = new Servo(pi, "steering");
    throttle = new Servo(pi, "throttle");

    // Subscribe to /actuator/drive
    ros::Subscriber steering = n.subscribe("/actuator/steering", 1, steering_callback);
    ros::Subscriber throttle = n.subscribe("/actuator/throttle", 1, throttle_callback);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    delete steering;
    delete throttle;

    return 0;
}

