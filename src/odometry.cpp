#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Imu.h"
#include "ras_utils/ras_utils.h"
#define PUBLISH_RATE 50 // Hz
#define QUEUE_SIZE 1000

// ** Robot params
#define TICKS_PER_REV 360.0       // Ticks per revolution for encoders
#define WHEEL_RADIUS  0.05      // Wheel radius [m]
#define WHEEL_BASE    0.205     // Distance between wheels [m]

class Odometric_coordinates
{
public:

    Odometric_coordinates(const ros::NodeHandle& n);
    void run();

private:

    double delta_x, delta_y, delta_angle;
    double x, y, angle;

    ros::NodeHandle n_;

    // Publishes geometry_msgs message to topic: robot/pose2d with x, y coordinates and CONTINUOUS angle.
    ros::Publisher pose2d_pub_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_;

    // Callback func when encoder data received
    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::WallTime t_IMU;
    bool IMU_init_;
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
    : n_(n), IMU_init_(false)
{
    // initial coordinates and angle
    x = 0;
    y = 0;
    angle = 0;

    // Publisher
    pose2d_pub_ = n_.advertise<geometry_msgs::Pose2D>("/robot/odometry", QUEUE_SIZE);
    // Subscriber
//    encoder_sub_ = n_.subscribe("/arduino/encoders", QUEUE_SIZE,  &Odometric_coordinates::encodersCallback, this);
    imu_sub_ = n_.subscribe("/imu/data", QUEUE_SIZE,  &Odometric_coordinates::IMUCallback, this);

}

void Odometric_coordinates::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!IMU_init_)
    {
        IMU_init_ = true;
        t_IMU = ros::WallTime::now();
        return;
    }
    angle += msg->angular_velocity.z * RAS_Utils::time_diff_ms(t_IMU, ros::WallTime::now()) * 0.001;
    t_IMU = ros::WallTime::now();
}

void Odometric_coordinates::encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    double d_left  = -(msg->delta_encoder1 / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;
    double d_right = -(msg->delta_encoder2 / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;

    delta_x = 0.5 * (d_left + d_right) * cos(angle);
    delta_y = 0.5 * (d_left + d_right) * sin(angle);
    delta_angle = -(d_left - d_right) / WHEEL_BASE;

    // positive x - straight from initial pose, positive y - to the right from initial pose, positive angle - clockwise from x axis.
    x += delta_x;
    y += delta_y;
    angle += delta_angle;
    //angle = fmod(angle, 2*M_PI);
}

void Odometric_coordinates::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok())
    {
        geometry_msgs::Pose2D msg;

        msg.x = x;
        msg.y = y;
        msg.theta = angle;

        // print
        ROS_INFO("[Odometry] X: %.3f Y: %.3f Theta: %.3f", x,y,angle);
//        std::cout << "delta_x: " << delta_x << " delta_y: " << delta_y << " delta_theta: " << delta_angle << std::endl;
//        std::cout << "x: " << x << " y: " << y << " theta: " << angle << "\n" << std::endl;

        pose2d_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Exiting...\n";
}

