#include <odometry/localization_ir_map.h>

Localization_IR_Map::Localization_IR_Map()
{
}

Localization_IR_Map::Localization_IR_Map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    this->map_ = map;
}

void Localization_IR_Map::updatePose(const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                     const geometry_msgs::Pose2D::ConstPtr &current_pose,
                                     geometry_msgs::Pose2D::Ptr &updated_pose)
{
    // ** Transform ADC into meters
    ADC_Data_M adc_data;
    this->constructADCData(adc_msg, adc_data);

    // ** Update theta
    double newTheta = this->updateTheta(adc_data, current_pose->theta);

    // ** Update pose
    this->updateXY(current_pose, adc_data, this->map_, newTheta, updated_pose);
}

double Localization_IR_Map::updateTheta(const ADC_Data_M &adc_data, double currentTheta)
{
    // ** Find closest wall
    double d_right = 0.5*(adc_data.front_right_ + adc_data.back_right_);
    double d_left  = 0.5*(adc_data.front_left_  + adc_data.back_left_);

    double d1 = d_right < d_left ? adc_data.back_right_ : adc_data.front_left_;
    double d2 = d_right < d_left ? adc_data.front_right_: adc_data.back_left_;

    // ** Get theta0
    std::vector<double> theta0v = {-M_PI, -M_PI/2.0, 0.0, M_PI/2.0, M_PI};
    double currentTheta0;
    double minDiff = 100;
    for(std::size_t i = 0; i < theta0v.size(); ++i)
    {
        double d = fabs(RAS_Utils::normalize_angle(currentTheta-theta0v[i]));
        if(d < minDiff)
        {
            minDiff = d;
            currentTheta0 = theta0v[i];
        }
    }

    double delta_theta = atan2(d2 - d1, SENSOR_BASELINE);
    double real_theta = RAS_Utils::normalize_angle(currentTheta0 + delta_theta);

    return real_theta;
}

void Localization_IR_Map::updateXY(const geometry_msgs::Pose2D::ConstPtr &currentPose,
                                   const ADC_Data_M &adc_data, const nav_msgs::OccupancyGrid::ConstPtr &map,
                                   double updatedTheta, geometry_msgs::Pose2D::Ptr &updatedPose)
{
    double x_optimal, y_optimal, min_cost = std::numeric_limits<double>::infinity();

    for(double x = currentPose->x - SEARCH_REGION_W/2.0; x < currentPose->x + SEARCH_REGION_W/2.0; x+=SEARCH_RESOLUTION)
    {
        for(double y = currentPose->y - SEARCH_REGION_H/2.0; y < currentPose->y + SEARCH_REGION_H/2.0; y+=SEARCH_RESOLUTION)
        {
            geometry_msgs::Pose2D test_pose;
            test_pose.x = x;
            test_pose.y = y;
            test_pose.theta = updatedTheta;
            double cost = computeCost(adc_data, map, test_pose);

            if(cost < min_cost)
            {
                min_cost = cost;
                x_optimal = x;
                y_optimal = y;
            }
        }
    }

    updatedPose->x = x_optimal;
    updatedPose->y = y_optimal;
    updatedPose->theta = updatedTheta;
}

double Localization_IR_Map::computeCost(const ADC_Data_M &adc_data,
                                      const nav_msgs::OccupancyGrid::ConstPtr &map,
                                      const geometry_msgs::Pose2D &test_pose)
{
    ADC_Data_M raytraced_data;
    raytrace(test_pose, map, raytraced_data);

    double cost  = pow(adc_data.back_        - raytraced_data.back_,        2) +
                   pow(adc_data.back_left_   - raytraced_data.back_left_,   2) +
                   pow(adc_data.back_right_  - raytraced_data.back_right_,  2) +
                   pow(adc_data.front_       - raytraced_data.front_,       2) +
                   pow(adc_data.front_left_  - raytraced_data.front_left_,  2) +
                   pow(adc_data.front_right_ - raytraced_data.front_right_, 2);

    return cost;
}

void Localization_IR_Map::raytrace(const geometry_msgs::Pose2D &test_pose,
                                   const nav_msgs::OccupancyGrid::ConstPtr &map,
                                   ADC_Data_M &raytraced_data)
{
    ROS_ERROR("[Localization_IR_Map::raytrace] TO DO");
}

void Localization_IR_Map::constructADCData(const ras_arduino_msgs::ADConverter::ConstPtr &adc_msg,
                                           ADC_Data_M &adc_data)
{
    // ** CHECK THAT THE CHANNELS ARE THE ONES THAT ARE SUPPOSED TO BE!!
    double d_front_right = RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch1) * 0.01;
    double d_back_right  = RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch2) * 0.01;
    double d_back_left   = RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch3) * 0.01;
    double d_front_left  = RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch4) * 0.01;

    double d_front       = RAS_Utils::sensors::longSensorToDistanceInCM(adc_msg->ch7) * 0.01;
    double d_back        = RAS_Utils::sensors::longSensorToDistanceInCM(adc_msg->ch8) * 0.01;

    adc_data = ADC_Data_M(d_front_right, d_back_right, d_front_left, d_back_left, d_front, d_back);
}

