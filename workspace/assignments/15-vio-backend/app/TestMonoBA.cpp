#include <iostream>
#include <random>
#include <math.h>
#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/edge_prior.h"
#include "backend/edge_reprojection.h"
#include "backend/problem.h"

using namespace myslam::backend;
using namespace std;

/*
 * Frame : 保存每帧的姿态和观测
 */
struct Frame {
    Frame(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    unordered_map<int, Eigen::Vector3d> featurePerId; // 该帧观测到的特征以及特征id
};

/*
 * 产生世界坐标系下的虚拟数据: 相机姿态, 特征点, 以及每帧观测
 */
void GetSimDataInWorldFrame(
    vector<Frame> &cameraPoses, 
    vector<Frame> &cameraPosesGT,
    vector<Eigen::Vector3d> &points
) {
    int featureNums = 20;  // 特征数目，假设每帧都能观测到所有的特征
    int poseNums = 3;     // 相机数目

    double radius = 8;

    std::default_random_engine generator;

    for (int n = 0; n < poseNums; ++n) {
        std::uniform_real_distribution<double> xyz_rand(
            -0.05, +0.05
        );
        std::uniform_real_distribution<double> theta_rand(
            -0.105, +0.105
        );

        // 绕 z轴 旋转 -- 1/4 圆弧:
        double theta_gt = n * 2 * M_PI / (poseNums * 4);
        double theta_obs = theta_gt + theta_rand(generator); 

        Eigen::Matrix3d R_gt, R_obs;
        R_gt = Eigen::AngleAxisd(theta_gt, Eigen::Vector3d::UnitZ());
        R_obs = Eigen::AngleAxisd(theta_obs, Eigen::Vector3d::UnitZ());

        double x_gt = radius * cos(theta_gt) - radius;
        double y_gt = radius * sin(theta_gt);
        double z_gt = 1 * sin(2 * theta_gt);
        double x_obs = x_gt + xyz_rand(generator);
        double y_obs = y_gt + xyz_rand(generator);
        double z_obs = z_gt + xyz_rand(generator);
        Eigen::Vector3d t_gt = Eigen::Vector3d(x_gt, y_gt, z_gt);
        Eigen::Vector3d t_obs = Eigen::Vector3d(x_obs, y_obs, z_obs);

        cameraPosesGT.push_back(Frame(R_gt, t_gt));
        cameraPoses.push_back(Frame(R_obs, t_obs));
    }

    std::normal_distribution<double> noise_pdf(0., 1. / 1000.);  // 2pixel / focal
    for (int j = 0; j < featureNums; ++j) {
        // 随机数生成三维特征点
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(4., 8.);

        Eigen::Vector3d Pw(xy_rand(generator), xy_rand(generator), z_rand(generator));
        points.push_back(Pw);

        // 在每一帧上的观测量
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Vector3d Pc = cameraPosesGT[i].Rwc.transpose() * (Pw - cameraPosesGT[i].twc);
            Pc = Pc / Pc.z();  // 归一化图像平面
            Pc[0] += noise_pdf(generator);
            Pc[1] += noise_pdf(generator);
            cameraPoses[i].featurePerId.insert(make_pair(j, Pc));
        }
    }
}

int main() {
    // generate camera poses & feature points:
    vector<Frame> cameras, cameras_gt;
    vector<Eigen::Vector3d> points;
    GetSimDataInWorldFrame(cameras, cameras_gt, points);

    // transform from camera to inertial frame:
    const double PRIOR_WEIGHT = 10000.0;

    Eigen::Quaterniond qic(1, 0, 0, 0);
    Eigen::Vector3d tic(0, 0, 0);

    // init problem:
    Problem problem(Problem::ProblemType::SLAM_PROBLEM);

    // add poses:
    MatXX H(MatXX::Zero(6, 6));
    for (size_t i = 0; i < 6; ++i) {
        H(i, i) = PRIOR_WEIGHT;
    }

    vector<shared_ptr<VertexPose> > vertexCams_vec;
    for (size_t i = 0; i < cameras.size(); ++i) {
        shared_ptr<VertexPose> vertexCam(new VertexPose());
        Eigen::VectorXd pose(7);
        pose << cameras[i].twc, cameras[i].qwc.x(), cameras[i].qwc.y(), cameras[i].qwc.z(), cameras[i].qwc.w();
        vertexCam->SetParameters(pose);

        // fix the first & second camera frame to remove 7 DoF uncertainty:
        if(i < 2) {
            // strategy 1: gauge fixation
            // vertexCam->SetFixed();
        }

        problem.AddVertex(vertexCam);
        vertexCams_vec.push_back(vertexCam);

        if(i < 2) {
            // strategy 2: gauge prior
            shared_ptr<EdgeSE3Prior> edge(
                new EdgeSE3Prior(
                    cameras_gt[i].twc, cameras_gt[i].qwc
                )
            );

            std::vector<std::shared_ptr<Vertex>> edge_vertex;
            edge_vertex.push_back(vertexCams_vec[i]);
            edge->SetVertex(edge_vertex);
            
            // set information matrix:
            edge->SetInformation(H);

            problem.AddEdge(edge);
        }
    }

    // 所有 Point 及 edge
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0, 1.);
    double noise = 0;
    vector<double> noise_invd;
    vector<shared_ptr<VertexInverseDepth> > allPoints;
    for (size_t i = 0; i < points.size(); ++i) {
        // use camera 0 as reference frame:
        Eigen::Vector3d Pw = points[i];
        Eigen::Vector3d Pc = cameras[0].Rwc.transpose() * (Pw - cameras[0].twc);
        // add noise:
        noise = noise_pdf(generator);
        double inverse_depth = 1. / (Pc.z() + noise);
        noise_invd.push_back(inverse_depth);

        // 初始化特征 vertex
        shared_ptr<VertexInverseDepth> verterxPoint(new VertexInverseDepth());
        VecX inv_d(1);
        inv_d << inverse_depth;
        verterxPoint->SetParameters(inv_d);
        problem.AddVertex(verterxPoint);
        allPoints.push_back(verterxPoint);

        // 每个特征对应的投影误差, 第 0 帧为起始帧
        for (size_t j = 1; j < cameras.size(); ++j) {
            // add reprojection error:
            Eigen::Vector3d pt_i = cameras[0].featurePerId.find(i)->second;
            Eigen::Vector3d pt_j = cameras[j].featurePerId.find(i)->second;
            shared_ptr<EdgeReprojection> edge(new EdgeReprojection(pt_i, pt_j));
            edge->SetTranslationImuFromCamera(qic, tic);

            std::vector<std::shared_ptr<Vertex>> edge_vertex;
            edge_vertex.push_back(verterxPoint);
            edge_vertex.push_back(vertexCams_vec[0]);
            edge_vertex.push_back(vertexCams_vec[j]);
            edge->SetVertex(edge_vertex);

            problem.AddEdge(edge);
        }
    }

    problem.Solve(100);

    std::cout.precision(4);
    std::cout << "\nCompare MonoBA results after optimization..." << std::endl;
    std::cout << "--------------- landmark position --------------------" <<std::endl;
    double error_landmark = 0.0;
    for (size_t k = 0; k < allPoints.size(); ++k) {
        double error = (allPoints[k]->Parameters()(0) - 1.0 / points[k].z());

        error_landmark += error * error;
        
        std::cout << "landmark " << k + 1 << ":\n"
                  << "\tground truth :" << std::fixed << 1. / points[k].z() 
                  << "\twith noise :" << std::fixed << noise_invd[k]
                  << "\topt " << std::fixed << allPoints[k]->Parameters() << std::endl;
    }
    error_landmark /= allPoints.size();

    std::cout << "------------ camera pose, translation ----------------" <<std::endl;
    double error_camera_pose = 0.0;
    for (size_t i = 0; i < vertexCams_vec.size(); ++i) {
        double error = (vertexCams_vec[i]->Parameters().head(3).transpose() - cameras_gt[i].twc.transpose()).squaredNorm();

        error_camera_pose += error;

        std::cout <<"camera " << i + 1 << ":\n" 
                  << "\tground truth: " << std::fixed << cameras_gt[i].twc.transpose() << std::endl
                  << "\toptimized: " << std::fixed << vertexCams_vec[i]->Parameters().head(3).transpose() <<std::endl;
    }
    error_camera_pose /= vertexCams_vec.size();

    std::cout << "------------ estimation error ----------------" << std::endl;
    std::cout << "\tprior weight: " << std::fixed << PRIOR_WEIGHT << std::endl;
    std::cout << "\tlandmark: " << std::fixed << sqrt(error_landmark) << std::endl;
    std::cout << "\tcamera pose: " << std::fixed << sqrt(error_camera_pose) << std::endl;

    /// 优化完成后，第一帧相机的 pose 平移（x,y,z）不再是原点 0,0,0. 说明向零空间发生了漂移。
    /// 解决办法：  或者加上非常大的先验值。

    problem.TestMarginalize();

    return 0;
}

