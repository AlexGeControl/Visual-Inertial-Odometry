//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class LandmarkGenerator {
public:
    LandmarkGenerator(
        const double XY_MIN, const double XY_MAX,
        const double Z_MIN, const double Z_MAX
    ) : 
        xy_uniform_(XY_MIN, XY_MAX), 
        z_uniform_(Z_MIN, Z_MAX)
    {}

    Eigen::Vector3d Get(void) {
        return Eigen::Vector3d(
            xy_uniform_(generator_),
            xy_uniform_(generator_),
            z_uniform_(generator_)
        );
    }

private:
    std::default_random_engine generator_;

    std::uniform_real_distribution<double> xy_uniform_;
    std::uniform_real_distribution<double> z_uniform_;
};

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R),qwc(R),twc(t) {};

    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;
};

int main()
{
    // num. of camera poses:
    const size_t N = 10;
    // num. of landmarks:
    const size_t M = 20;
    
    // Hessian dimensions:
    int D = N * 6 + M * 3;

    // camera intrinsics:
    double fx = 1.;
    double fy = 1.;

    // generate camera poses:
    std::vector<Pose> camera_pose;
    double radius = 8;
    for (size_t n = 0; n < N; ++n) {
        // current heading:
        double theta = n * 2 * M_PI / ( N * 4);

        // rotate around Z-axis:
        // a. orientation:
        Eigen::Matrix3d R = Eigen::AngleAxisd(
            // angle:
            theta, 
            // axis:
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();
        // b. position:
        Eigen::Vector3d t(
            radius * cos(theta) - radius, 
            radius * sin(theta), 
            1 * sin(2 * theta)
        );

        camera_pose.push_back(Pose(R,t));
    }

    // initialize Hessian:
    Eigen::MatrixXd H(D,D);
    H.setZero();

    // observe landmarks:
    LandmarkGenerator landmark_gen(-4.0, 4.0, 8.0, 10.0);
    for(size_t m = 0; m < M; ++m)
    {
        Eigen::Vector3d Pw = landmark_gen.Get();

        for (size_t n = 0; n < N; ++n) {
            // observe landmark:
            Eigen::Matrix3d Rcw = camera_pose.at(n).Rwc.transpose();
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose.at(n).twc);
            
            // intermediate variables:
            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z * z;

            // reprojection Jacobian with respect to normalized P:
            Eigen::Matrix<double,2,3> jacobian_uv_Pc;
            jacobian_uv_Pc << 
                fx/z,    0, -x * fx/z_2,
                   0, fy/z, -y * fy/z_2;

            // reprojection Jacobian with respect to landmark position Pw:
            Eigen::Matrix<double,2,3> jacobian_Pj = jacobian_uv_Pc * Rcw;

            Eigen::Matrix<double,2,6> jacobian_Ci;

            Eigen::Matrix3d P_hat;
            P_hat << 
                 0, -z, +y,
                +z,  0, -x,
                -y, +x,  0;

            jacobian_Ci.block<2, 3>(0, 0) = jacobian_uv_Pc * Rcw;
            jacobian_Ci.block<2, 3>(0, 3) = jacobian_uv_Pc * P_hat;

            // update Hessian:
            H.block<6, 6>(      n*6,       n*6) += jacobian_Ci.transpose() * jacobian_Ci;
            H.block<6, 3>(      n*6, N*6 + m*3) += jacobian_Ci.transpose() * jacobian_Pj;
            H.block<3, 6>(N*6 + m*3,       n*6) += jacobian_Pj.transpose() * jacobian_Ci;
            H.block<3, 3>(N*6 + m*3, N*6 + m*3) += jacobian_Pj.transpose() * jacobian_Pj;
        }
    }

    // compute SVD:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto singular_values = svd.singularValues();
    
    std::cout << "Singular Values:" << std::endl;

    for (size_t n = 0; n < singular_values.size(); ++n) {
        std::cout << "\t" << n + 1 << ": " << singular_values(n) << std::endl;
    }
  
    return EXIT_SUCCESS;
}
