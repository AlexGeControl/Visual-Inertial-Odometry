//
// Created by Ge Yao on 22/06/20
//

#include <fstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <cmath>

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

#include <pangolin/pangolin.h>

struct PoseStamped {
    PoseStamped(double timestamp, Eigen::Quaterniond &q, const Eigen::Vector3d &t)
        : stamp(timestamp),
        pose(q, t) {
    }

    double stamp;
    Sophus::SE3d pose;
};

struct PoseAligned {
    PoseAligned(const PoseStamped &s, const PoseStamped &t)
        : source(s),
        target(t) {
    }

    PoseStamped source;
    PoseStamped target;
};

typedef std::vector<PoseAligned> TrajectoryAligned;

struct OptimizationState {
    OptimizationState(void) 
        : cost_curr(std::numeric_limits<double>::max()),
        cost_prev(std::numeric_limits<double>::max()) {
    }

    double cost_curr;
    double cost_prev;
    Sophus::SE3d T;
};  

void LoadTrajectoryAligned(std::string input_file_name, TrajectoryAligned &traj) {
    // load all values into buffer:
    std::vector<double> values;

    std::ifstream input_file(input_file_name);
    std::istream_iterator<double> input(input_file);
    std::copy(
        input, std::istream_iterator<double>(), std::back_inserter(values)
    );

    // format as pixel coordinates:
    size_t N = values.size() / 16;
    for (size_t i = 0; i < N; ++i) {
        // parse source:
        double timestamp_source{values.at(16*i + 0)};
        Eigen::Quaterniond q_source{values.at(16*i + 7), values.at(16*i + 4), values.at(16*i + 5), values.at(16*i + 6)};
        Eigen::Vector3d t_source{values.at(16*i + 1), values.at(16*i + 2), values.at(16*i + 3)};
        PoseStamped pose_source{timestamp_source, q_source, t_source};

        // parse target:
        double timestamp_target{values.at(16*i + 8)};
        Eigen::Quaterniond q_target{values.at(16*i + 15), values.at(16*i + 12), values.at(16*i + 13), values.at(16*i + 14)};
        Eigen::Vector3d t_target{values.at(16*i + 9), values.at(16*i + 10), values.at(16*i + 11)};
        PoseStamped pose_target{timestamp_target, q_target, t_target};

        PoseAligned pose_aligned{pose_source, pose_target};
        traj.push_back(pose_aligned);
    }
}

void DrawTrajectory(
    const TrajectoryAligned &traj,
    const Sophus::SE3d &pose
) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("ICP Registration", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    const size_t N = traj.size();
    const auto &R = pose.rotationMatrix();
    const auto &t = pose.translation();
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < N - 1; i++) {
            // estimated:
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto te1 = R * traj.at(i).source.pose.translation() + t;
            auto te2 = R * traj.at(i+1).source.pose.translation() + t;
            glVertex3d(te1[0], te1[1], te1[2]);
            glVertex3d(te2[0], te2[1], te2[2]);
            glEnd();
            // ground truth:
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto tg1 = traj.at(i).target.pose.translation();
            auto tg2 = traj.at(i+1).target.pose.translation();
            glVertex3d(tg1[0], tg1[1], tg1[2]);
            glVertex3d(tg2[0], tg2[1], tg2[2]);
            glEnd();            
        }
        pangolin::FinishFrame();
    }
}

const std::string INPUT_FILE_NAME = "../data/04-icp/compare.txt";

int main(int argc, char **argv) {
    // load aligned trajectory:
    TrajectoryAligned traj;

    LoadTrajectoryAligned(INPUT_FILE_NAME, traj);

    // start optimization:
    size_t N = traj.size();
    size_t MAX_ITERATIONS = 100;
    std::cout << "[ICP G-N]: Trajectory length " << N << ", Max Num. of iterations: " << MAX_ITERATIONS << std::endl;

    // init estimated pose
    OptimizationState state;

    for (size_t i = 0; i < MAX_ITERATIONS; ++i) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

        state.cost_curr = 0.0;
        for (size_t j = 0; j < N; j++) {
            // re-project::
            Eigen::Vector3d t_source = traj.at(j).source.pose.translation();
            Eigen::Vector3d t_target_esti = state.T.rotationMatrix()*t_source + state.T.translation();
            Eigen::Vector3d t_target = traj.at(j).target.pose.translation();

            // calcuate error:
            Eigen::Vector3d e = t_target - t_target_esti;

            // update cost:
            state.cost_curr += 0.5*e.squaredNorm();

	        // compute jacobian
            Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
            
            double x{t_target_esti(0)}, y{t_target_esti(1)}, z{t_target_esti(2)};

            // first part:
            J(0, 0) = J(1, 1) = J(2, 2) = 1.0;
            // second part:
            J(0, 4) = +z;
            J(0, 5) = -y;
            J(1, 3) = -z;
            J(1, 5) = +x;
            J(2, 3) = +y;
            J(2, 4) = -x;
        
            J *= -1.0;

            // update G-N:
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Eigen::Matrix<double, 6, 1> dT = H.fullPivHouseholderQr().solve(b);

        // validity check:
        if (std::isnan(dT[0])) {
            std::cout << "[ICP G-N]: Result is nan at iteration " << i << ". Aborted." << std::endl;
            break;
        }

        // early stopping:
        if (i > 0 && state.cost_curr >= state.cost_prev) {
            // cost increase, update is not good
            std::cout << "[ICP G-N]: Cost increased. current: " << state.cost_curr << ", last: " << state.cost_prev << ". Aborted." << std::endl;
            break;
        }

        std::cout << "\tIteration " << i << " current cost: " << std::cout.precision(12) << state.cost_curr << " last cost: " << std::cout.precision(12) << state.cost_prev << std::endl;

        // update pose:
        state.T = Sophus::SE3d::exp(dT) * state.T;
        
        state.cost_prev = state.cost_curr;
    }

    std::cout << "[ICP G-N]: estimated pose: \n" << state.T.matrix() << std::endl;

    DrawTrajectory(traj, state.T);

    return EXIT_SUCCESS;
}