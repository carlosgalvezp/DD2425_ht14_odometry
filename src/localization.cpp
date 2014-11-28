#include <odometry/localization.h>
using namespace Eigen;

Localization::Localization()
{
    mu_     << 0.0, 0.0, 0.0;
    sigma_  << SIGMA_0*SIGMA_0, 0.0, 0.0,
               0.0, SIGMA_0*SIGMA_0, 0.0,
               0.0, 0.0, SIGMA_0*SIGMA_0;

    Q_      << Q_XY0*Q_XY0, 0.0, 0.0,
               0.0, Q_XY0*Q_XY0, 0.0,
               0.0, 0.0, Q_THETA0*Q_THETA0;

    R_      << R_R*R_R, 0.0,
               0.0,     R_THETA*R_THETA;

    I3 = Eigen::Matrix3f::Identity();
}

void Localization::updatePose(const Eigen::Vector2f &u, const Eigen::Vector2f &z, double delta_t,
                         Eigen::Vector3f &mu, Eigen::Matrix3f &sigma)
{
    Eigen::Vector3f mu_bar;
    Eigen::Matrix3f sigma_bar;

    // ** Predict
    predict( mu, sigma, u, delta_t, mu_bar, sigma_bar );

    // ** Update
    update( mu_bar, sigma_bar, z, mu, sigma );
}

void Localization::predict(const Eigen::Vector3f &mu, const Eigen::Matrix3f &sigma, const Eigen::Vector2f &u, double delta_t,
                                 Eigen::Vector3f &mu_bar, Eigen::Matrix3f &sigma_bar)
{
    Eigen::Matrix3f F;
    motion_model(mu, u, delta_t, mu_bar, F);
    sigma_bar = F * sigma * F.transpose() + Q_;
}

void Localization::update(const Eigen::Vector3f &mu_bar, const Eigen::Matrix3f &sigma_bar,
                          const Eigen::Vector2f &z, Eigen::Vector3f &mu, Eigen::Matrix3f &sigma)
{
    Eigen::Vector2f z_hat, y;
    Eigen::MatrixXf H, S;
    measurement_model_ML(mu_bar, sigma_bar, objects_map_, z, z_hat, H, S);

    y = z - z_hat;
    Eigen::MatrixXf K = sigma_bar * H.transpose() * S.inverse();

    mu    = mu_bar + K * y;
    sigma = (I3 - K * H) * sigma_bar;
}

void Localization::motion_model(const Eigen::Vector3f &mu, const Eigen::Vector2f &u, double delta_t,
                                      Eigen::Vector3f &mu_bar, Eigen::Matrix3f &F)
{
    double x, y, theta, v, w;
    double x_bar, y_bar, theta_bar;

    x = mu(0,0);    y = mu(1,0);    theta = mu(2,0);
    v = u(0,0);     w = u(1,0);

    x_bar     = x + v*cos(theta)*delta_t;
    y_bar     = y + v*sin(theta)*delta_t;
    theta_bar = theta + w*delta_t;

    mu_bar(0,0) = x_bar;
    mu_bar(1,0) = y_bar;
    mu_bar(2,0) = theta_bar;

    F << 1.0, 0.0, -v*sin(theta)*delta_t,
         0.0, 1.0, v*cos(theta)*delta_t,
         0.0, 0.0, 1.0;
}

void Localization::measurement_model(const Eigen::Vector3f &mu_bar, const std::vector<Position> &map,
                                     std::size_t index, Eigen::Vector2f &z_hat, Eigen::MatrixXf &H)
{
    z_hat(0,0) = sqrt( pow(map[index].x_ - mu_bar(0,0), 2) + pow( map[index].y_ - mu_bar(1,0), 2 ) );
    z_hat(1,0) = atan2( map[index].y_ - mu_bar(1,0), map[index].x_ - mu_bar(0,0) ) - mu_bar(2,0);

    H(0,0) = ( mu_bar(0,0) - map[index].x_ ) / z_hat(0,0);
    H(1,0) = - ( mu_bar(1,0) - map[index].y_ ) / pow( z_hat(0,0), 2 );
    H(0,1) = ( mu_bar(1,0) - map[index].y_ ) / z_hat(0,0);
    H(1,1) = ( mu_bar(0,0) - map[index].x_ ) / pow( z_hat(0,0), 2 );
    H(0,2) = 0.0;
    H(1,2) = -1.0;
}


void Localization::measurement_model_ML(const Eigen::Vector3f &mu_bar, const Eigen::Matrix3f &sigma_bar, const std::vector<Position> &map,
                                        const Eigen::Vector2f &z, Eigen::Vector2f z_hat, Eigen::MatrixXf &H, Eigen::MatrixXf &S)
{
    // ** Analize all positions in the map
    double max_likelihood = 0.0;

    for(std::size_t i = 0; i < map.size(); ++i)
    {
        Eigen::Vector2f z_hat_i, nu_i;
        Eigen::MatrixXf H_i, S_i;
        measurement_model(mu_bar, map, i, z_hat_i, H_i);

        S_i = H_i * sigma_bar * H_i.transpose() + R_; // It's Q in the pdf
        nu_i = z - z_hat_i;

        double det = std::pow((2*M_PI*S_i).determinant(), -0.5);
        double phi_i = det*std::exp(-0.5 * nu_i.transpose()*S_i.inverse()*nu_i);

        if(phi_i > max_likelihood)
        {
            max_likelihood = phi_i;
            z_hat = z_hat_i;
            H = H_i;
            S = S_i;
        }
    }
}








