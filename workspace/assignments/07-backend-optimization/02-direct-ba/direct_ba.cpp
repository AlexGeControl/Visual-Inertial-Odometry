//
// Created by xiang on 1/4/18.
// this program shows how to perform direct bundle adjustment
//

// I/O utilities:
#include <iostream>
#include <vector>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

// camera estimation:
#include "camera.hpp"

// landmark estimation:
#include "landmark.hpp"

// g2o optimization:
#include "graph.hpp"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
// visualization:
#include <pangolin/pangolin.h>


using namespace std;


// global variables
string pose_file = "../data/02-direct-ba/poses.txt";
string points_file = "../data/02-direct-ba/points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// half patch size:
int HALF_PATCH_SIZE = 2;
int MAX_ITERATIONS = 200;
int ITERATION_STEP_SIZE = 100;
double HUBER_KERNEL_THRESHOLD = 12.0;

// plot the poses and points for you, need pangolin
void Draw(std::vector<CameraWithObservation> &camera_observations, std::vector<Landmark> &landmarks) {
    if (camera_observations.empty() || landmarks.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (const auto &T: camera_observations) {
            glPushMatrix();
            Sophus::Matrix4f m = T.GetPose().to_homogeneous_matrix().inverse().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (const auto &l: landmarks) {
            Eigen::Vector3d p = l.GetPosition();
            glColor3f(0.0, p.z()/4, 1.0-p.z()/4);
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char **argv) {
    // read camera observations:
    std::vector<CameraWithObservation> camera_observations;

    ifstream fin(pose_file);
    boost::format observation_pattern("../data/02-direct-ba/%d.png");
    for (int i = 0; i < 7; ++i) {
        if (fin.eof()) break;

        double timestamp = 0;
        double data[7];

        // parse timestamp:
        fin >> timestamp;
        if (timestamp == 0) break;

        // parse camera pose:
        for (auto &d: data) fin >> d;

        CameraWithObservation camera_observation(
            // camera intrinsics:
            fx, fy, cx, cy,
            // camera pose:
            Eigen::Quaterniond(data[6], data[3], data[4], data[5]), Eigen::Vector3d(data[0], data[1], data[2]),
            // camera observation:
            cv::imread((observation_pattern % i).str(), 0)
        );
        
        camera_observations.push_back(camera_observation);

        if (!fin.good()) break;
    }
    fin.close();

    // read landmarks:
    std::vector<Landmark> landmarks;
    std::vector<double> W;

    fin.open(points_file);
    for (int i = 0; i < 4118; ++i) {
        
        double position[3];
        double intensities[16];

        // parse landmark position:
        for (auto &p: position) fin >> p;
        // parse surrounding patch intensity:
        for (auto &i: intensities) fin >> i;

        W.clear();
        for (auto &i: intensities) W.push_back(i);

        Landmark landmark(
            // position:
            Eigen::Vector3d(position[0], position[1], position[2]),
            // half patch size:
            HALF_PATCH_SIZE,
            // patch intensities:
            W
        );

        landmarks.push_back(landmark);

        if (!fin.good()) break;
    }
    fin.close();

    cout << "[Direct BA]: num. of observations -- " << camera_observations.size() << ", num. of landmarks -- " << landmarks.size() << endl;

    // pose dimension 6, landmark is 3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    // use LM
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex:
    std::vector<VertexLandmark *> vertex_landmarks;
    for (size_t i = 0; i < landmarks.size(); ++i) {
        VertexLandmark *v = new VertexLandmark();
        
        v->setId(i);
        v->setEstimate(landmarks.at(i));
        v->setMarginalized(true);

        optimizer.addVertex(v);

        vertex_landmarks.push_back(v);
    }
    std::vector<VertexCamera *> vertex_cameras;
    for (size_t i = 0; i < camera_observations.size(); ++i) {
        VertexCamera *v = new VertexCamera();
        
        v->setId(i + landmarks.size());
        v->setEstimate(camera_observations.at(i));

        // use first camera frame as local frame:
        if (i == 0) {
            v->setFixed(true);
        }

        optimizer.addVertex(v);

        vertex_cameras.push_back(v);
    }

    // initial optimization:
    std::vector<EdgeDirectProjection *> edges;
    for (size_t i = 0; i < landmarks.size(); ++i) {
        for (size_t j = 0; j < camera_observations.size(); ++j) {
            // parse pose and point:
            const CameraWithObservation &camera_observation = camera_observations.at(j);
            const Landmark &landmark = landmarks.at(i);

            // get patch anchor:
            Eigen::Vector3d X = landmark.GetPosition();
            Eigen::Vector2d p = camera_observation.Project(X);
            const int HALF_PATCH_SIZE = landmark.GetHalfPatchSize();

            if (!camera_observation.IsValidPatch(p, HALF_PATCH_SIZE)) {
                continue;
            }

            EdgeDirectProjection *e = new EdgeDirectProjection();

            e->setVertex(0, vertex_cameras.at(j));
            e->setVertex(1, vertex_landmarks.at(i));
            
            e->setInformation(Eigen::Matrix<double, 16, 16>::Identity());

            g2o::RobustKernelHuber *robust_kernel = new g2o::RobustKernelHuber();
            robust_kernel->setDelta(HUBER_KERNEL_THRESHOLD);
            e->setRobustKernel(robust_kernel);

            optimizer.addEdge(e);

            edges.push_back(e);
        }
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(ITERATION_STEP_SIZE);

    // iterative optimization with bad edges removed:
    size_t num_bad = 0;
    size_t num_iterations = ITERATION_STEP_SIZE;
    while (num_bad < edges.size() && num_iterations < MAX_ITERATIONS) {
        // remove invalid edges:
        for (size_t i = 0; i < edges.size(); ++i) {
            // do not handle invalid edge:
            if (!edges.at(i)) continue;

            auto vertices = edges.at(i)->vertices();

            const VertexCamera *vertex_camera = static_cast<VertexCamera *>(vertices.at(0));
            const VertexLandmark *vertex_landmark = static_cast<VertexLandmark *>(vertices.at(1));

            // parse pose and point:
            const CameraWithObservation &camera_observation = vertex_camera->estimate();
            const Landmark &landmark = vertex_landmark->estimate();

            // get patch anchor:
            Eigen::Vector3d X = landmark.GetPosition();
            Eigen::Vector2d p = camera_observation.Project(X);
            const int HALF_PATCH_SIZE = landmark.GetHalfPatchSize();

            if (!camera_observation.IsValidPatch(p, HALF_PATCH_SIZE)) {
                optimizer.removeEdge(edges.at(i));
                edges.at(i) = static_cast<EdgeDirectProjection *>(NULL);
                ++num_bad;
            }
        }

        std::cout << "[Optimization Status]: num. bad -- " << num_bad << ", num iterations -- " << num_iterations << std::endl;

        optimizer.initializeOptimization();
        optimizer.optimize(ITERATION_STEP_SIZE);

        num_iterations += ITERATION_STEP_SIZE;
    }

    // fetch data from the optimizer
    for (size_t i = 0; i < vertex_cameras.size(); ++i) {
        camera_observations.at(i) = vertex_cameras.at(i)->estimate();
    }
    for (size_t i = 0; i < vertex_landmarks.size(); ++i) {
        landmarks.at(i) = vertex_landmarks.at(i)->estimate();
    }

    // plot the optimized points and poses
    Draw(camera_observations, landmarks);

    return EXIT_SUCCESS;
}