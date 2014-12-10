#ifndef LOCALIZATION_IR_MAP_H
#define LOCALIZATION_IR_MAP_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_srv_msgs/IRData.h>
// RAS
#include <ras_utils/ras_utils.h>
#include <ras_utils/ras_sensor_utils.h>
#include <ras_utils/pcl_utils.h>
#include <ras_utils/occupancy_map_utils.h>
#include <tf/tf.h>

#define SENSOR_BASELINE     0.18 // [m] Distance between side sensors

#define SEARCH_REGION_H     0.1
#define SEARCH_REGION_W     0.1
#define SEARCH_RESOLUTION   0.001

#define SHORT_SENSOR_OFFSET_X           0.09 // [m]
#define SHORT_SENSOR_OFFSET_Y           0.057 // [m]
#define LONG_SENSOR_OFFSET_X            0.115 - 0.088 // [m] Robot-center - measurement

#define MAX_SENSOR_DISTANCE             0.1

class Localization_IR_Map
{    
public:
    Localization_IR_Map();
    Localization_IR_Map(const nav_msgs::OccupancyGrid::ConstPtr &map);

    void updatePose(const ras_srv_msgs::IRData::ConstPtr &adc_msg,
                    const geometry_msgs::Pose2D::ConstPtr &current_pose,
                          geometry_msgs::Pose2D::Ptr      &updated_pose);

    double updateTheta(const ras_srv_msgs::IRData::ConstPtr &adc_data, double currentTheta);

private:

    void updateXY(const geometry_msgs::Pose2D::ConstPtr &currentPose,
                  const ras_srv_msgs::IRDataConstPtr &adc_data,
                  const nav_msgs::OccupancyGrid::ConstPtr &map,
                  double updatedTheta, geometry_msgs::Pose2D::Ptr &updatedPose);

    double computeCost(const ras_srv_msgs::IRData::ConstPtr &adc_data,
                     const nav_msgs::OccupancyGrid::ConstPtr &map,
                     const geometry_msgs::Pose2D &test_pose);

    void raytrace(const geometry_msgs::Pose2D &test_pose,
                  const nav_msgs::OccupancyGrid::ConstPtr &map,
                  ras_srv_msgs::IRData &raytraced_data);

    void computeSensorXYThetaWorld(const geometry_msgs::Pose2D &robot_pose, int sensor_id,
                                     double & sensor_x_pos, double & sensor_y_pos, double & sensor_theta);

    double raytraceSensor(const geometry_msgs::Pose2D &test_pose, int sensor_id,
                          const nav_msgs::OccupancyGrid & occ_grid);
    nav_msgs::OccupancyGrid::ConstPtr map_;

};

#endif // LOCALIZATION_IR_MAP_H
