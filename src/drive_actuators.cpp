#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Float64.h"
#include "car_actuators/Stop.h"
#include "car_actuators/ServoConfig.h"

#include <pigpiod_if2.h>

double odom = 0.0, odom_limit = 0.2;

class Servo
{
    bool first_reconfig = true;
    int pi, gpio, middle = 1500;
    dynamic_reconfigure::Server<car_actuators::ServoConfig> *dyn_conf_srv;

    double offset = 0.0;
    int min_value = 1000, max_value = 2000;

    void reconfigure_callback(car_actuators::ServoConfig &config, uint32_t level);
public:
    void update(double value);

    Servo(int in_pi, std::string prefix);
    ~Servo();
};

void Servo::update(double value)
{
    int amplitude;
    if (value >= 0)
    {
         amplitude = max_value - middle;
    } 
    else 
    {
         amplitude = middle - min_value;
    }
    int width = (value + offset) * amplitude + middle;
    set_servo_pulsewidth(pi, gpio, width);
}

void Servo::reconfigure_callback(car_actuators::ServoConfig &config, uint32_t level)
{
    if (first_reconfig)
    {
        config.min_value = min_value;
        config.max_value = max_value;
        config.offset = offset;
    
        first_reconfig = false;
    } 
    else 
    {
        min_value = config.min_value;
        max_value = config.max_value;
        offset = config.offset;
    }
}

Servo::Servo(int in_pi, std::string prefix)
{
    ros::NodeHandle pH("~" + prefix);

    pi = in_pi;
    pH.getParam("gpio", gpio);
    pH.param("offset", offset, 0.0);
    pH.param("min_value", min_value, 1000);
    pH.param("max_value", max_value, 2000);

    set_mode(pi, gpio, PI_OUTPUT);

    // Dynamic params
    dyn_conf_srv = new dynamic_reconfigure::Server<car_actuators::ServoConfig>(pH);
    dynamic_reconfigure::Server<car_actuators::ServoConfig>::CallbackType f;
    f = boost::bind(&Servo::reconfigure_callback, this, _1, _2);
    dyn_conf_srv->setCallback(f);

    update(0.0);
}

Servo::~Servo()
{
    delete dyn_conf_srv;
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

void odom_callback(const std_msgs::Float64 msg)
{
    odom = msg.data;
}

bool handle_stop_command(car_actuators::Stop::Request& req,
    car_actuators::Stop::Response& res)
{

    if (req.stop && odom > odom_limit)
    {
        throttle->update(-1);
        ros::Rate loop_rate(100);
        // wait until stop
        while (ros::ok() && odom > odom_limit)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    // Return a response message
    res.stopped = true;

    return true;
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

    ros::ServiceServer service = n.advertiseService("/actuator/stop", handle_stop_command);
    ros::Subscriber odom = n.subscribe("/odom", 1, odom_callback);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    delete steering;
    delete throttle;

    return 0;
}

