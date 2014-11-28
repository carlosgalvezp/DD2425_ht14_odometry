#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ras_utils/graph/position.h>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

// ** Error model
#define Q_XY0       0.05    // [m]   Process error (odometry in XY coordinates)
#define Q_THETA0    0.1     // [rad] Process error (odometry in theta coordinate)

#define R_R         0.01    // [m]     Measurement error (distance to object)
#define R_THETA     0.01    // [rad]   Measurement error (bearing to object)

#define SIGMA_0     0.01    // [m or rad] Initial uncertainty.
                            // A low value means we know where we start

/**
 * @brief The Localization class
 * Allows for a better localization of the robot.
 * It uses an EKF to fuse together the information from the odometry (which
 * might drift over time) and the information from the camera. In particular,
 * the position will be updated whenever the robot sees and object that was in the
 * map before.
 * This assumes that, of course, a perfect map and location of the objects are known
 */
class Localization
{
public:
    Localization();

    /**
     * @brief Updates the pose of the robot using Extended Kalman Filter
     * @param u Control input (odometry): u = [v, w]^T
     * @param z Measurement from the camera: z = [r, theta]^T, where:
     *              r: distance to object (in robot frame)
     *              theta: bearing to object (in robot frame)
     *
     * @param mu:       updated pose
     * @param sigma:    updated covariance
     */
    void updatePose(const Eigen::Vector2f &u, const Eigen::Vector2f &z, double delta_t,
                          Eigen::Vector3f &mu, Eigen::Matrix3f &sigma);

private:
    Eigen::Vector3f mu_;        // Internal representation of the state mean
    Eigen::Matrix3f sigma_;     // Internal representation of the state covariance

    Eigen::Matrix3f Q_;         // Process noise
    Eigen::Matrix3f R_;         // Measurement noise

    Eigen::Matrix3f I3;         // Identity 3x3

    std::vector<Position> objects_map_; // Contains the positions of the objects in the map (world coordinates)

    /**
     * @brief Predict phase of the EKF
     * @param mu            current mu
     * @param sigma         current sigma
     * @param u             control input u = [v w]^T
     * @param delta_t       time interval [s]
     * @param mu_bar        prediction on mu
     * @param sigma_bar     prediction on sigma
     */
    void predict(const Eigen::Vector3f &mu, const Eigen::Matrix3f &sigma, const Eigen::Vector2f &u, double delta_t,
                       Eigen::Vector3f &mu_bar, Eigen::Matrix3f &sigma_bar);

    /**
     * @brief Update phase of the EKF
     * @param mu_bar        predicted mu    (from predict step)
     * @param sigma_bar     predicted sigma (from predict step)
     * @param z             measurement  z = [r theta]^T (range and bearing)
     * @param mu            updated mu
     * @param sigma         updated sigma
     */
    void update(const Eigen::Vector3f &mu_bar, const Eigen::Matrix3f &sigma_bar,
                              const Eigen::Vector2f &z, Eigen::Vector3f &mu, Eigen::Matrix3f &sigma);

    /**
     * @brief Motion model: f(mu_t-1, u_t)
     * @param mu            current mu
     * @param u             current control command
     * @param delta_t       time interval [s]
     * @param mu_bar        predicted mu
     * @param F             Jacobian of f at mu = mu_(t-1), u_t
     */
    void motion_model(const Eigen::Vector3f &mu, const Eigen::Vector2f &u, double delta_t,
                            Eigen::Vector3f &mu_bar, Eigen::Matrix3f &F);

    /**
     * @brief Measurement model given the map and object index: h(mu_bar, M, k)
     * @param mu_bar        predicted mu
     * @param map           set of x,y Position for every object in the map
     * @param z_hat         predicted most likely measurement
     * @param H             Jacobian of h at mu = mu_bar
     */
    void measurement_model(const Eigen::Vector3f &mu_bar, const std::vector<Position> &map,
                           std::size_t index, Eigen::Vector2f &z_hat, Eigen::MatrixXf &H);

    /**
     * @brief Maximum Likelihood Association
     * @param mu_bar
     * @param S
     * @param H
     */
    void measurement_model_ML(const Eigen::Vector3f &mu_bar, const Eigen::Matrix3f &sigma_bar, const std::vector<Position> &map, const Eigen::Vector2f &z,
                                           Eigen::Vector2f z_hat, Eigen::MatrixXf &H, Eigen::MatrixXf &S);

};

#endif // LOCALIZATION_H
