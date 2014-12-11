#include <odometry/localization_ir_map.h>

Localization_IR_Map::Localization_IR_Map()
{
}

Localization_IR_Map::Localization_IR_Map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    this->map_ = map;
}

void Localization_IR_Map::updatePose(const ras_srv_msgs::IRData::ConstPtr &adc_msg,
                                     const geometry_msgs::Pose2D &current_pose,
                                     geometry_msgs::Pose2D &updated_pose)
{
    ros::WallTime t1(ros::WallTime::now());
    // ** Update theta
    double newTheta = this->updateTheta(adc_msg, current_pose.theta);
    updated_pose = current_pose;
    updated_pose.theta = newTheta;
    // ** Update pose
//    this->updateXY(current_pose, adc_msg, this->map_, newTheta, updated_pose);
    ROS_INFO("[Localization] %.3f ms", RAS_Utils::time_diff_ms(t1, ros::WallTime::now()));
}

double Localization_IR_Map::updateTheta(const ras_srv_msgs::IRData::ConstPtr &adc_data, double currentTheta)
{
    // ** Find closest wall
    double d_right = 0.5*(adc_data->front_right + adc_data->back_right);
    double d_left  = 0.5*(adc_data->front_left  + adc_data->back_left);

    double d1 = d_right < d_left ? adc_data->back_right : adc_data->front_left;
    double d2 = d_right < d_left ? adc_data->front_right: adc_data->back_left;

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

void Localization_IR_Map::updateXY(const geometry_msgs::Pose2D &currentPose,
                                   const ras_srv_msgs::IRData::ConstPtr &adc_data,
                                   const nav_msgs::OccupancyGrid::ConstPtr &map,
                                   geometry_msgs::Pose2D &updatedPose)
{
    double x_optimal, y_optimal, min_cost = std::numeric_limits<double>::infinity();

    for(double x = currentPose.x - SEARCH_REGION_W/2.0; x < currentPose.x + SEARCH_REGION_W/2.0; x+=SEARCH_RESOLUTION)
    {
        for(double y = currentPose.y - SEARCH_REGION_H/2.0; y < currentPose.y + SEARCH_REGION_H/2.0; y+=SEARCH_RESOLUTION)
        {
            geometry_msgs::Pose2D test_pose;
            test_pose.x = x;
            test_pose.y = y;
            test_pose.theta = currentPose.theta;
            double cost = computeCost(adc_data, map, test_pose);

            if(cost < min_cost)
            {
                min_cost = cost;
                x_optimal = x;
                y_optimal = y;
            }
        }
    }

    updatedPose.x = x_optimal;
    updatedPose.y = y_optimal;
}

double Localization_IR_Map::computeCost(const ras_srv_msgs::IRData::ConstPtr &adc_data,
                                        const nav_msgs::OccupancyGrid::ConstPtr &map,
                                        const geometry_msgs::Pose2D &test_pose)
{
    ras_srv_msgs::IRData raytraced_data;
    raytrace(test_pose, map, raytraced_data);

    double cost = 0.0;

    // ** Add to the cost only the sensors that are close to the wall
    if(adc_data->back < MAX_SENSOR_DISTANCE )
        cost += pow(adc_data->back - raytraced_data.back, 2);

    if(adc_data->front < MAX_SENSOR_DISTANCE)
        cost += pow(adc_data->front - raytraced_data.front, 2);

    if(adc_data->back_left < MAX_SENSOR_DISTANCE)
        cost += pow(adc_data->back_left - raytraced_data.back_left, 2);

    if(adc_data->back_right < MAX_SENSOR_DISTANCE)
        cost += pow(adc_data->back_right - raytraced_data.back_right, 2);

    if(adc_data->front_left < MAX_SENSOR_DISTANCE)
        cost += pow(adc_data->front_left - raytraced_data.front_left, 2);

    if(adc_data->front_right < MAX_SENSOR_DISTANCE)
        cost += pow(adc_data->front_right - raytraced_data.front_right, 2);

    return cost;
}

void Localization_IR_Map::raytrace(const geometry_msgs::Pose2D &test_pose,
                                   const nav_msgs::OccupancyGrid::ConstPtr &map,
                                   ras_srv_msgs::IRData &raytraced_data)
{
    raytraced_data.front_right = raytraceSensor(test_pose, IR_SENSOR_FRONT_RIGHT, *map);
    raytraced_data.front_left  = raytraceSensor(test_pose, IR_SENSOR_FRONT_LEFT,  *map);
    raytraced_data.back_right  = raytraceSensor(test_pose, IR_SENSOR_BACK_RIGHT,  *map);
    raytraced_data.back_left   = raytraceSensor(test_pose, IR_SENSOR_BACK_LEFT,   *map);
    raytraced_data.front       = raytraceSensor(test_pose, IR_SENSOR_FRONT,       *map);
    raytraced_data.back        = raytraceSensor(test_pose, IR_SENSOR_BACK,        *map);
}

double Localization_IR_Map::raytraceSensor(const geometry_msgs::Pose2D &test_pose, int sensor_id,
                                         const nav_msgs::OccupancyGrid & occ_grid)
{
    double sensor_x_pos, sensor_y_pos, sensor_angle;
    computeSensorXYThetaWorld(test_pose, sensor_id, sensor_x_pos, sensor_y_pos, sensor_angle);

    double stepper = 0.001;
    for(double current_distance = stepper; current_distance < 0.3; current_distance += stepper)
    {
        double new_x_pos = sensor_x_pos + cos(sensor_angle) * current_distance;
        double new_y_pos = sensor_y_pos + sin(sensor_angle) * current_distance;
        bool pos_is_wall = RAS_Utils::occ_grid::isWall(occ_grid, new_x_pos, new_y_pos);
        if(pos_is_wall)
        {
            return current_distance;
        }
    }
    return -1;
}

void Localization_IR_Map::computeSensorXYThetaWorld(const geometry_msgs::Pose2D &robot_pose, int sensor_id,
                                 double & sensor_x_pos, double & sensor_y_pos, double & sensor_theta)
{
    tf::Transform tf_robot_to_world;
    tf::Quaternion q;
    q.setRPY(0,0,robot_pose.theta);
    tf::Vector3 origin(robot_pose.x, robot_pose.y, 0.0);

    tf_robot_to_world.setOrigin(origin);
    tf_robot_to_world.setRotation(q);

    Eigen::Matrix4f tf_eigen;
    PCL_Utils::convertTransformToEigen4x4(tf_robot_to_world, tf_eigen);

    pcl::PointXYZ p;
    p.z = 0.0;
    switch (sensor_id)
    {
    case IR_SENSOR_FRONT_RIGHT:
        p.x =  SHORT_SENSOR_OFFSET_X;
        p.y = -SHORT_SENSOR_OFFSET_Y;
        sensor_theta = RAS_Utils::normalize_angle(robot_pose.theta - M_PI/2.0);
        break;

    case IR_SENSOR_BACK_RIGHT:
        p.x = -SHORT_SENSOR_OFFSET_X;
        p.y = -SHORT_SENSOR_OFFSET_Y;
        sensor_theta = RAS_Utils::normalize_angle(robot_pose.theta - M_PI/2.0);
        break;

    case IR_SENSOR_FRONT_LEFT:
        p.x =  SHORT_SENSOR_OFFSET_X;
        p.y =  SHORT_SENSOR_OFFSET_Y;
        sensor_theta = RAS_Utils::normalize_angle(robot_pose.theta + M_PI/2.0);
        break;


    case IR_SENSOR_BACK_LEFT:
        p.x = -SHORT_SENSOR_OFFSET_X;
        p.y =  SHORT_SENSOR_OFFSET_Y;
        sensor_theta = RAS_Utils::normalize_angle(robot_pose.theta + M_PI/2.0);
        break;

    case IR_SENSOR_FRONT:
        p.x =  LONG_SENSOR_OFFSET_X;
        p.y =  0.0;
        sensor_theta = robot_pose.theta;
        break;

    case IR_SENSOR_BACK:
        p.x = -LONG_SENSOR_OFFSET_X;
        p.y =  0.0;
        sensor_theta = -robot_pose.theta;
        break;
    }
    PCL_Utils::transformPoint(p, tf_eigen, p);
    sensor_x_pos = p.x;
    sensor_y_pos = p.y;
}
