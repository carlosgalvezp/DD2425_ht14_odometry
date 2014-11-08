#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000

// ** Robot params
#define TICKS_PER_REV 360     // Ticks per revolution for encoders
#define WHEEL_RADIUS  0.05  // Wheel radius [m]
#define WHEEL_BASE    0.205    // Distance between wheels [m]

class Odometric_coordinates
{
public:

    Odometric_coordinates(const ros::NodeHandle& n);
    void run();

private:

    double delta_x, delta_y, delta_angle;
    double x, y, angle;

    ros::NodeHandle n_;

    ros::Subscriber encoder_sub_;

    // Callback func when encoder data received
    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
};

int main (int argc, char* argv[])
{  
    // ** Init node
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    // ** Create odometry object
    Odometric_coordinates odo(n);

    // ** Run
    odo.run();
}

Odometric_coordinates::Odometric_coordinates(const ros::NodeHandle &n)
    : n_(n)
{
    // initial coordinates and angle
    x = 0;
    y = 0;
    angle = 0;

    // Subscriber
    encoder_sub_ = n_.subscribe("/kobuki/encoders", QUEUE_SIZE,  &Odometric_coordinates::encodersCallback, this);
}

void Odometric_coordinates::encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    double d_left = (double) -(msg->delta_encoder1) / TICKS_PER_REV * 2 * M_PI * WHEEL_RADIUS;
    double d_right = (double) -(msg->delta_encoder2) / TICKS_PER_REV * 2 * M_PI * WHEEL_RADIUS;

    delta_x = (d_left + d_right) / 2 * cos(angle);
    delta_y = (d_left + d_right) / 2 * sin(angle);
    delta_angle = (d_left - d_right) / WHEEL_BASE;

    x += delta_x;
    y += delta_y;
    angle += delta_angle;
    angle = fmod(angle, 2*M_PI);

    // print
    std::cout << "x: " << x << " y: " << y << " angle: " << angle << std::endl;
}

void Odometric_coordinates::run()
{
    ros::spin();

    std::cout << "Exiting...\n";
}

