#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <ras_utils/ras_names.h>

#include <ras_utils/pcl_utils.h>

#define PUBLISH_RATE 50 // Hz

class Test_TF
{
public:

    Test_TF(const ros::NodeHandle& n);
    void run();

private:

    ros::NodeHandle n_;
    tf::TransformBroadcaster br;
    tf::TransformListener tf_listener;


    void publish_transform(double x, double y, double theta);
};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    // ** Create odometry object
    Test_TF odo(n);

    // ** Run
    odo.run();
}

Test_TF::Test_TF(const ros::NodeHandle &n)
    : n_(n)
{
}

void Test_TF::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok())
    {
//        publish_transform(0.63, 1.2, M_PI/3);

        // Read transform
        tf::Transform tf;
        PCL_Utils::readTransform(COORD_FRAME_ROBOT, COORD_FRAME_WORLD, tf_listener, tf);

        Eigen::Matrix4f t_matrix;
        PCL_Utils::convertTransformToEigen4x4(tf, t_matrix);
        std::cout << t_matrix << std::endl;

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void Test_TF::publish_transform(double x, double y, double theta)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), COORD_FRAME_WORLD, COORD_FRAME_ROBOT));
}
