#ifndef LOCALIZATION_IR_MAP_H
#define LOCALIZATION_IR_MAP_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>

// RAS
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_utils/ras_utils.h>
#include <ras_utils/ras_sensor_utils.h>

#define SENSOR_BASELINE     0.18 // [m] Distance between side sensors

#define SEARCH_REGION_H     0.1
#define SEARCH_REGION_W     0.1
#define SEARCH_RESOLUTION   0.001
class Localization_IR_Map
{
    struct ADC_Data_M
    {
        double front_right_;
        double back_right_;
        double front_left_;
        double back_left_;
        double front_;
        double back_;

        ADC_Data_M(){}
        ADC_Data_M(double front_right, double back_right, double front_left, double back_left, double front, double back)
            : front_right_(front_right), back_right_(back_right), front_left_(front_left), back_left_(back_left), front_(front), back_(back)
        {}
    };

public:
    Localization_IR_Map();
    Localization_IR_Map(const nav_msgs::OccupancyGrid::ConstPtr &map);

    void updatePose(const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                    const geometry_msgs::Pose2D::ConstPtr &current_pose,
                          geometry_msgs::Pose2D::Ptr      &updated_pose);

    double updateTheta(const ADC_Data_M &adc_data, double currentTheta);

//    void setADC_data(const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg);
private:

    void updateXY(const geometry_msgs::Pose2D::ConstPtr &currentPose,
                  const ADC_Data_M &adc_data,
                  const nav_msgs::OccupancyGrid::ConstPtr &map,
                  double updatedTheta, geometry_msgs::Pose2D::Ptr &updatedPose);

    void constructADCData(const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                ADC_Data_M &adc_data);

    double computeCost(const ADC_Data_M &adc_data,
                     const nav_msgs::OccupancyGrid::ConstPtr &map,
                     const geometry_msgs::Pose2D &test_pose);

    void raytrace(const geometry_msgs::Pose2D &test_pose,
                  const nav_msgs::OccupancyGrid::ConstPtr &map,
                  ADC_Data_M &raytraced_data);

    nav_msgs::OccupancyGrid::ConstPtr map_;

};

#endif // LOCALIZATION_IR_MAP_H
