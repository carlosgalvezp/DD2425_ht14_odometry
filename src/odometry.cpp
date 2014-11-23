#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/Imu.h"
#include "ras_utils/ras_utils.h"
#include <tf/transform_broadcaster.h>

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
    ros::Publisher pose2d_pub_, marker_pub_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_;
    static tf::TransformBroadcaster br;

    // Callback func when encoder data received
    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void publish_transform(double x, double y, double theta);

    void publish_marker();

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
    pose2d_pub_ = n_.advertise<geometry_msgs::Pose2D>(TOPIC_ODOMETRY, QUEUE_SIZE);
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, QUEUE_SIZE);
    // Subscriber
    encoder_sub_ = n_.subscribe("/arduino/encoders", QUEUE_SIZE,  &Odometric_coordinates::encodersCallback, this);
    // imu_sub_ = n_.subscribe("/imu/data", QUEUE_SIZE,  &Odometric_coordinates::IMUCallback, this);

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

        pose2d_pub_.publish(msg);
        publish_transform(x,y,angle);

        publish_marker();
        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Exiting...\n";
}

void Odometric_coordinates::publish_transform(double x, double y, double theta)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), COORD_FRAME_WORLD, COORD_FRAME_ROBOT));
}

void Odometric_coordinates::publish_marker()
{
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(2); // Print object position and arrow

    visualization_msgs::Marker & marker_obj = msg.markers[0];
    visualization_msgs::Marker & marker_arrow = msg.markers[1];

    // Robot
    marker_obj.header.frame_id = COORD_FRAME_WORLD;
    marker_obj.header.stamp = ros::Time();
    marker_obj.ns = "Robot";
    marker_obj.id = 0;
    marker_obj.action = visualization_msgs::Marker::ADD;
    marker_obj.pose.position.x = x;
    marker_obj.pose.position.y = y;
    marker_obj.pose.position.z = 0;

    marker_obj.pose.orientation.x = 0.0;
    marker_obj.pose.orientation.y = 0.0;
    marker_obj.pose.orientation.z = angle;
    marker_obj.pose.orientation.w = 1.0;
    marker_obj.scale.x = 0.1;
    marker_obj.scale.y = 0.1;
    marker_obj.scale.z = 0.1;

    marker_obj.color.a = 1.0;
    marker_obj.color.r = 1.0;
    marker_obj.color.g = 0.0;
    marker_obj.color.b = 0.0;
    marker_obj.type = visualization_msgs::Marker::SPHERE;

    // Arrow
    marker_arrow.header.frame_id = COORD_FRAME_WORLD;
    marker_arrow.header.stamp = ros::Time();
    marker_arrow.ns = "Robot";
    marker_arrow.id = 1;
    marker_arrow.action = visualization_msgs::Marker::ADD;
    marker_arrow.pose.position.x = x;
    marker_arrow.pose.position.y = y;
    marker_arrow.pose.position.z = 0;

    marker_arrow.pose.orientation.x = 0.0;
    marker_arrow.pose.orientation.y = 0.0;
    marker_arrow.pose.orientation.z = angle;
    marker_arrow.pose.orientation.w = 1.0;
    marker_arrow.scale.x = 0.2;
    marker_arrow.scale.y = 0.02;
    marker_arrow.scale.z = 0.02;

    marker_arrow.color.a = 1.0;
    marker_arrow.color.r = 0.0;
    marker_arrow.color.g = 0.0;
    marker_arrow.color.b = 1.0;
    marker_arrow.type = visualization_msgs::Marker::ARROW;

    marker_pub_.publish( msg);

}
