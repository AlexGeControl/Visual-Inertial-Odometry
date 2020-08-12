//
// Created by hyj on 18-11-11.
//
#include <boost/program_options.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <random> 

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "CSVWriter.h"

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


int main(int argc, char *argv[])
{
    // parse command line options:
    try
    {
        boost::program_options::options_description desc{"Visual Frontend Triangulation Evaluation"};
        desc.add_options()
            ("help,h", "Show help info")
            ("std_max,m", boost::program_options::value<double>()->default_value(1.0), "Max standard deviation of measurement noise.")
            ("std_delta,d", boost::program_options::value<double>()->default_value(0.1), "Change step size of measurement noise standard deviation.")
            ("start_frame_id,s", boost::program_options::value<size_t>()->default_value(0), "Sliding window start frame id.")
            ("output,o", boost::program_options::value<std::string>()->default_value("."), "Output Directory");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
        } else {
            // parse arguments:
            const double &STD_MAX = vm["std_max"].as<double>();
            const double &STD_DELTA = vm["std_delta"].as<double>();
            const size_t &START_FRAME_ID = vm["start_frame_id"].as<size_t>();
            const std::string  &OUTPUT_DIR = vm["output"].as<std::string>();

            std::cout << "[Configuration]:" << std::endl;
            std::cout << "\tMax. Std. of Measurement Noise: " << STD_MAX << std::endl;
            std::cout << "\tStepsize of Measurement Noise Std. Change: " << STD_DELTA << std::endl;
            std::cout << "\tStart Frame ID of Sliding Window: " << START_FRAME_ID << std::endl;
            std::cout << "\tOutput Directory: " << OUTPUT_DIR << std::endl;
            std::cout << std::endl;

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

            // generate random landmark:
            std::default_random_engine generator;

            std::uniform_real_distribution<double> xy_rand(-4.0,  4.0);
            std::uniform_real_distribution<double>  z_rand( 8.0, 10.0);

            Eigen::Vector3d Pw(
                xy_rand(generator), 
                xy_rand(generator), 
                 z_rand(generator)
            );

            // init output:
            CSVWriter csv(",");
            csv.enableAutoNewRow(4);
            csv << "measurement_noise_stddev" 
                << "sliding_window_size" 
                << "quality_ratio" 
                << "relative_error";

            // generate measurement:
            for (double std_measurement = 0.0; std_measurement <= STD_MAX; std_measurement += STD_DELTA) {
                for (size_t start_frame_id = START_FRAME_ID; start_frame_id < N - 1; ++start_frame_id) {
                    // measurement noise:
                    std::normal_distribution<double> uv_rand(0.0, std_measurement);

                    for (size_t i = start_frame_id; i < N; ++i) {
                        // project to camera frame:
                        Eigen::Matrix3d Rcw = camera_pose.at(i).Rwc.transpose();
                        Eigen::Vector3d Pc = Rcw*(Pw - camera_pose.at(i).twc);

                        double x = Pc.x();
                        double y = Pc.y();
                        double z = Pc.z();

                        // project to pixel frame:
                        camera_pose.at(i).uv = Eigen::Vector2d(
                            x/z + uv_rand(generator),
                            y/z + uv_rand(generator)
                        );
                    }
                    
                    // triangulation:
                    const size_t M = N - start_frame_id;
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

                    // calculate KPIs:
                    Eigen::VectorXd singular_values = svd.singularValues();
                    double quality_ratio = singular_values(2) / singular_values(3);
                    double relative_error = 100.0 * (P_est - Pw).norm() / Pw.norm();

                    // append:
                    csv << std_measurement
                        << M
                        << quality_ratio
                        << relative_error;
                }
            }
            
            // write results:
            const std::string OUTPUT_FILENAME = OUTPUT_DIR+"/evaluation-results.csv";
            csv.writeToFile(OUTPUT_FILENAME);

            std::cout << "The evaluation results is available at " << OUTPUT_FILENAME << std::endl;

            return EXIT_SUCCESS;
        }
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
