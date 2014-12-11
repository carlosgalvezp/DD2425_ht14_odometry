#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/MarkerArray.h>
#include <ras_utils/ras_sensor_utils.h>
#include <ras_srv_msgs/IRData.h>
#include <nav_msgs/OccupancyGrid.h>

#include "sensor_msgs/Imu.h"
#include "ras_utils/ras_utils.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Bool.h>

#include <odometry/localization.h>
#include <odometry/localization_ir_map.h>

#define PUBLISH_RATE 50 // Hz
#define QUEUE_SIZE 1000

// ** Robot params
#define TICKS_PER_REV 360.0       // Ticks per revolution for encoders
#define WHEEL_RADIUS  0.05      // Wheel radius [m]
#define WHEEL_BASE    0.205     // Distance between wheels [m]

// ** IR params
#define SIDE_IR_SENSORS_SEPARATION 18 // Separation between front and back sensor [cm]
#define N_IR_SAMPLES               10 // Number of samples over which to take the average to compute the initial pose
class Odometry
{
public:

    Odometry(const ros::NodeHandle& n);
    void run();

private:

    //double delta_x, delta_y, delta_angle;
    double x, y, theta;

    ros::NodeHandle n_;

    // Publishes geometry_msgs message to topic: robot/pose2d with x, y coordinates and angle.
    ros::Publisher pose2d_pub_, marker_pub_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber adc_sub_;
    ros::Subscriber object_position_sub_;

    ros::Subscriber map_sub_, localize_sub_, adc_filtered_sub_;

    static tf::TransformBroadcaster br;

    // Callback func when encoder data received
    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);
    void adcFilteredCallback(const ras_srv_msgs::IRData::ConstPtr & msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);

    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void objectPositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

    void publish_transform(double x, double y, double theta);

    void publish_marker(double x, double y, double theta);

    void localizationCallback(const std_msgs::Bool::ConstPtr &msg);

    ros::WallTime t_IMU;
    bool IMU_init_, encoders_init_, first_pose_estimate_init_;

    geometry_msgs::Pose2D current_pose_;

    Localization localization_;
    int timestamp_;
    int first_pose_counter_;
    std::vector<double> sensor_values_;

    bool localize_;

    nav_msgs::OccupancyGrid::ConstPtr map_msg_;
    ras_srv_msgs::IRData::ConstPtr adc_filtered_msg_;

    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;
    Eigen::Vector2f z_;     // TODO: Nothing with z_ is implemented

    Localization_IR_Map loc_ir_map_;
};

int main (int argc, char* argv[])
{  
    // ** Init node
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    // ** Create odometry object
    Odometry odo(n);

    // ** Run
    odo.run();
}

Odometry::Odometry(const ros::NodeHandle &n)
    : n_(n), IMU_init_(false), first_pose_estimate_init_(false), first_pose_counter_(0), localize_(true)
{
    // Publisher
    pose2d_pub_ = n_.advertise<geometry_msgs::Pose2D>(TOPIC_ODOMETRY, QUEUE_SIZE);
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>(TOPIC_MARKERS, QUEUE_SIZE);
    // Subscriber
    encoder_sub_ = n_.subscribe(TOPIC_ARDUINO_ENCODERS, QUEUE_SIZE,  &Odometry::encodersCallback, this);
    adc_sub_     = n_.subscribe(TOPIC_ARDUINO_ADC, QUEUE_SIZE, &Odometry::adcCallback, this);
//    object_position_sub_ = n_.subscribe(TOPIC_OBJECTS_EKF, 10, &Odometry::objectPositionCallback, this);
    // imu_sub_ = n_.subscribe("/imu/data", QUEUE_SIZE,  &Odometric_coordinates::IMUCallback, this);

    map_sub_ = n_.subscribe(TOPIC_MAP_OCC_GRID, QUEUE_SIZE, &Odometry::mapCallback, this);
    localize_sub_ = n_.subscribe(TOPIC_LOCALIZATION, 1, &Odometry::localizationCallback, this);
    adc_filtered_sub_ = n_.subscribe(TOPIC_ARDUINO_ADC_FILTERED, 1, &Odometry::adcFilteredCallback, this);

    z_ << 0.0, 0.0;
    mu_     << 0.0, 0.0, 0.0;
    sigma_  << SIGMA_0*SIGMA_0, 0.0, 0.0,
               0.0, SIGMA_0*SIGMA_0, 0.0,
               0.0, 0.0, SIGMA_0*SIGMA_0;

    sensor_values_.resize(4);
}

void Odometry::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok())
    {
//        ROS_INFO("[Odometry] %.3f, %.3f, %.3f",current_pose_.x, current_pose_.y, current_pose_.theta);
        pose2d_pub_.publish(current_pose_);

        publish_transform(current_pose_.x, current_pose_.y, current_pose_.theta);
        publish_marker(current_pose_.x, current_pose_.y, current_pose_.theta);
        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Odometry::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!IMU_init_)
    {
        IMU_init_ = true;
        t_IMU = ros::WallTime::now();
        return;
    }
    theta += msg->angular_velocity.z * RAS_Utils::time_diff_ms(t_IMU, ros::WallTime::now()) * 0.001;
    t_IMU = ros::WallTime::now();
}

void Odometry::encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    if(first_pose_estimate_init_)
    {
        double deltaT = msg->timestamp * 0.001;     //Timestamp is given in ms

        double w_right = -( 2 * M_PI * msg->delta_encoder2 ) / ( TICKS_PER_REV * deltaT );
        double w_left = -( 2 * M_PI * msg->delta_encoder1 ) / ( TICKS_PER_REV * deltaT );
        double w = ( w_right - w_left ) * WHEEL_RADIUS / WHEEL_BASE;
        double v = ( w_right + w_left ) * WHEEL_RADIUS / 2;

        Eigen::Vector2f u;
        u << v, w;
        localization_.updatePose(u,z_,deltaT, mu_, sigma_);
        // Reset z
        z_ << 0.0,0.0;

        current_pose_.x = mu_(0,0);
        current_pose_.y = mu_(1,0);
        current_pose_.theta = mu_(2,0);
    }
}

void Odometry::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
    // ** Convert ADC data
    double front_right_cm = RAS_Utils::sensors::shortSensorToDistanceInCM(msg->ch4);
    double back_right_cm = RAS_Utils::sensors::shortSensorToDistanceInCM(msg->ch3);
    double front_left_cm = RAS_Utils::sensors::shortSensorToDistanceInCM(msg->ch1);
    double back_left_cm = RAS_Utils::sensors::shortSensorToDistanceInCM(msg->ch2);

    sensor_values_[0] += front_right_cm/N_IR_SAMPLES;
    sensor_values_[1] += back_right_cm /N_IR_SAMPLES;
    sensor_values_[2] += front_left_cm /N_IR_SAMPLES;
    sensor_values_[3] += back_left_cm / N_IR_SAMPLES;

    first_pose_counter_ ++;
    if(first_pose_counter_ > N_IR_SAMPLES)
    {
        // ** Compute initial angle
        double theta1 =  atan2(sensor_values_[0] - sensor_values_[1], SIDE_IR_SENSORS_SEPARATION); // Right sensor
        double theta2 = -atan2(sensor_values_[2] - sensor_values_[3], SIDE_IR_SENSORS_SEPARATION); // Left sensor (negative angle)

        double theta = 0.5 * (theta1 + theta2);

        // ** Set initial mu
        mu_ << 0.0, 0.0, theta;

        // ** Unsubscribe
        adc_sub_.shutdown();
        first_pose_estimate_init_ = true;
    }
}

void Odometry::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    this->map_msg_ = msg;
}

void Odometry::adcFilteredCallback(const ras_srv_msgs::IRData::ConstPtr &msg)
{
//    this->adc_filtered_msg_ = msg;
//    double newTheta = loc_ir_map_.updateThetaAuto(msg, current_pose_.theta);
//    current_pose_.theta = newTheta;
//    mu_(2,0) = current_pose_.theta;
}

void Odometry::localizationCallback(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_INFO("RECEIVED LOCALIZATION REQUEST");
    this->localize_ = msg->data;
}

void Odometry::objectPositionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    z_ << msg->x, msg->theta;
}

void Odometry::publish_transform(double x, double y, double theta)
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

void Odometry::publish_marker(double x, double y, double theta)
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
    marker_obj.pose.position.z = 0.05;

    marker_obj.pose.orientation.x = 0.0;
    marker_obj.pose.orientation.y = 0.0;
    marker_obj.pose.orientation.z = theta;
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
    q.setRPY(0,0,theta);
    marker_arrow.pose.position.x = x;
    marker_arrow.pose.position.y = y;
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
