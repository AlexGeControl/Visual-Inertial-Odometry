//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};

    // orientation:
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    // position:
    Eigen::Vector3d twc;
    // observation:
    Eigen::Vector2d uv;
};


int main()
{
    // camera pose config:
    // a. camera intrinsics
    const double fx = 1.;
    const double fy = 1.;
    // b. motion params:
    double R = 8;
    // c. num samples:
    const size_t N = 10;

    // generate camera poses:
    std::vector<Pose> camera_pose;
    for(size_t n = 0; n < N; ++n ) {
        // orientation:
        double theta = n * 2 * M_PI / ( N * 4); // 1/4 圆弧
        // position:
        double x = R * cos(theta) - R;
        double y = R * sin(theta);
        double z = 1 * sin(2 * theta);

        // format:
        Eigen::Matrix3d R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Vector3d t = Eigen::Vector3d(x, y, z);

        // update:
        camera_pose.push_back(Pose(R,t));
    }

    // generate landmark:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4.0,  4.0);
    std::uniform_real_distribution<double>  z_rand( 8.0, 10.0);

    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz =  z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);

    // observe:
    const size_t start_frame_id = 3;
    const size_t end_frame_id = N;

    for (size_t i = start_frame_id; i < end_frame_id; ++i) {
        // project to camera frame:
        Eigen::Matrix3d Rcw = camera_pose.at(i).Rwc.transpose();
        Eigen::Vector3d Pc = Rcw*(Pw - camera_pose.at(i).twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        // project to pixel frame:
        camera_pose.at(i).uv = Eigen::Vector2d(
            x/z,
            y/z
        );
    }
    
    // triangulation:
    const size_t M = end_frame_id - start_frame_id;
    Eigen::MatrixXd D(2*M, 4);

    for (size_t i = 0; i < M; ++i) {
        size_t index = start_frame_id + i;

        Eigen::Matrix3d Rcw = camera_pose.at(index).Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw*camera_pose.at(index).twc;
        Eigen::MatrixXd Pcw(3, 4);

        Pcw.block<3, 3>(0, 0) = Rcw;
        Pcw.block<3, 1>(0, 3) = tcw;

        double u = camera_pose.at(index).uv(0);
        double v = camera_pose.at(index).uv(1);

        D.block<1, 4>(      i << 1, 0) = u*Pcw.block<1, 4>(2, 0) - Pcw.block<1, 4>(0, 0);
        D.block<1, 4>((i << 1) + 1, 0) = v*Pcw.block<1, 4>(2, 0) - Pcw.block<1, 4>(1, 0); 
    }
    
    // here SVD solver is used since the eigen vector of D.transpose()*D is just V:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd u_min = svd.matrixV().col(3);
    // normalize:
    Eigen::Vector3d P_est(
        u_min(0) / u_min(3),
        u_min(1) / u_min(3),
        u_min(2) / u_min(3)
    );

    // format output:
    std::cout.precision(4);

    std::cout << "[Singular Values]:" << std::endl;
    Eigen::VectorXd singular_values = svd.singularValues();
    for (size_t i = 0; i < 4; ++i) {
        std::cout << "\t" << i + 1 << ":" << std::fixed << singular_values(i) << std::endl;
    }
    std::cout << "\tQuality Ratio: " << std::fixed << singular_values(2) / singular_values(3) << std::endl;

    std::cout << "[Result Summary]:" << std::endl;
    std::cout << "\tGround Truth: \n\t\t"<< std::fixed << Pw.transpose() << std::endl;
    std::cout << "\tEstimation: \n\t\t"<< std::fixed << P_est.transpose() << std::endl;
    std::cout << "\tError: \n\t\t" << std::fixed << (P_est - Pw).norm() << std::endl;
    
    return EXIT_SUCCESS;
}
