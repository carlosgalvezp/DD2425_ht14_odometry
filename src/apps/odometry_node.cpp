#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/MarkerArray.h>

#include "sensor_msgs/Imu.h"
#include "ras_utils/ras_utils.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>

#include <odometry/localization.h>

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

    //double delta_x, delta_y, delta_angle;
    double x_, y_, angle_;

    ros::NodeHandle n_;

    // Publishes geometry_msgs message to topic: robot/pose2d with x, y coordinates and angle.
    ros::Publisher pose2d_pub_, marker_pub_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_;
    static tf::TransformBroadcaster br;

    // Callback func when encoder data received
    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void publish_transform(double x_, double y_, double theta);

    void publish_marker();

    ros::WallTime t_IMU;
    bool IMU_init_;

    Localization localization_;
    ros::WallTime t_;

    Eigen::Vector3f mu_;
    Eigen::Vector2f u_;
    Eigen::Matrix3f sigma_;
    Eigen::Vector2f z_;     // TODO: Nothing with z_ is implemented
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
    x_ = 0;
    y_ = 0;
    angle_ = 0;

    // Publisher
    pose2d_pub_ = n_.advertise<geometry_msgs::Pose2D>(TOPIC_ODOMETRY, QUEUE_SIZE);
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, QUEUE_SIZE);
    // Subscriber
    encoder_sub_ = n_.subscribe("/arduino/encoders", QUEUE_SIZE,  &Odometric_coordinates::encodersCallback, this);
    // imu_sub_ = n_.subscribe("/imu/data", QUEUE_SIZE,  &Odometric_coordinates::IMUCallback, this);

    z_ << 0.0, 0.0;
}

void Odometric_coordinates::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!IMU_init_)
    {
        IMU_init_ = true;
        t_IMU = ros::WallTime::now();
        return;
    }
    angle_ += msg->angular_velocity.z * RAS_Utils::time_diff_ms(t_IMU, ros::WallTime::now()) * 0.001;
    t_IMU = ros::WallTime::now();
}

void Odometric_coordinates::encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    ros::WallTime t (ros::WallTime::now());
    double deltaT = RAS_Utils::time_diff_ms(t_, t) * 0.001;     //deltaT = t - t_;
    t_ = t;

    double w_right = -( 2 * M_PI * msg->delta_encoder2 ) / ( TICKS_PER_REV * deltaT );
    double w_left = -( 2 * M_PI * msg->delta_encoder1 ) / ( TICKS_PER_REV * deltaT );
    double w = ( w_right - w_left ) * WHEEL_RADIUS / WHEEL_BASE;
    double v = ( w_right + w_left ) * WHEEL_RADIUS / 2;

    u_ << v * deltaT * cos( mu_(2,0) ), v * deltaT * sin( mu_(2,0) ), w * deltaT;

    localization_.updatePose(u_,z_,deltaT, mu_, sigma_);

    x_ = mu_(0,0);
    y_ = mu_(1,0);
    angle_ = mu_(1,0);
}

void Odometric_coordinates::run()
{
    t_ = ros::WallTime::now();

    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok())
    {
        geometry_msgs::Pose2D msg;

        msg.x = x_;
        msg.y = y_;
        msg.theta = angle_;

        pose2d_pub_.publish(msg);
        publish_transform(x_,y_,angle_);

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
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), COORD_FRAME_WORLD, "/odom"));

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
    marker_obj.pose.position.x = x_;
    marker_obj.pose.position.y = y_;
    marker_obj.pose.position.z = 0.05;

    marker_obj.pose.orientation.x = 0.0;
    marker_obj.pose.orientation.y = 0.0;
    marker_obj.pose.orientation.z = angle_;
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

    tf::Quaternion q;
    q.setRPY(0,0,angle_);
    marker_arrow.pose.position.x = x_;
    marker_arrow.pose.position.y = y_;
    marker_arrow.pose.position.z = 0.05;

    marker_arrow.pose.orientation.x = q.x();
    marker_arrow.pose.orientation.y = q.y();
    marker_arrow.pose.orientation.z = q.z();
    marker_arrow.pose.orientation.w = q.w();

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
