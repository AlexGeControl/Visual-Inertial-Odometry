#pragma once

#include <iostream>

#include "camera.hpp"
#include "landmark.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include "sophus/se3.hpp"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>


typedef Eigen::Matrix<double,16,1> Vector16d;


class VertexCamera : public g2o::BaseVertex<6, CameraWithObservation> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexCamera() {}

    virtual void setToOriginImpl() override {
        _estimate = CameraWithObservation();
    }

    virtual void oplusImpl(const double *update) override {
        // parse updates:
        Eigen::Vector3d dt(
            update[0], update[1], update[2]
        );
        Eigen::Matrix3d dR = Sophus::SO3d::exp(
            Eigen::Vector3d(update[3], update[4], update[5])
        ).matrix();

        // format as lie algebra:
        g2o::SE3Quat dT(dR, dt);

        _estimate.UpdatePose(dT);
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};

class VertexLandmark : public g2o::BaseVertex<3, Landmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexLandmark() {}

    virtual void setToOriginImpl() override {
        _estimate = Landmark();
    }

    virtual void oplusImpl(const double *update) override {
        Eigen::Vector3d dP(update[0], update[1], update[2]);

        _estimate.UpdatePosition(dP);
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};

class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, VertexCamera, VertexLandmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        // parse vertices:
        auto vertex_camera = static_cast<VertexCamera *>(_vertices[0]);
        auto vertex_landmark = static_cast<VertexLandmark *>(_vertices[1]);

        // parse pose and point:
        const CameraWithObservation &camera_observation = vertex_camera->estimate();
        const Landmark &landmark = vertex_landmark->estimate();

        // get patch anchor:
        Eigen::Vector3d X = landmark.GetPosition();
        Eigen::Vector2d p = camera_observation.Project(X);
        const int HALF_PATCH_SIZE = landmark.GetHalfPatchSize();

        if (!camera_observation.IsValidPatch(p, HALF_PATCH_SIZE)) {
            // stop optimization:
            setLevel(1);
            _error = Vector16d::Zero();
            return;
        }

        // get projection:
        Vector16d I_camera;
        for (int dx = -HALF_PATCH_SIZE; dx < HALF_PATCH_SIZE; ++dx) {
            for (int dy = -HALF_PATCH_SIZE; dy < HALF_PATCH_SIZE; ++dy) {
                // get linear index:
                int i = (dx + HALF_PATCH_SIZE) * (HALF_PATCH_SIZE << 1) + (dy + HALF_PATCH_SIZE);

                // get deviation:
                Eigen::Vector2d dp(dx, dy);

                // pixel intensity in observation:
                I_camera[i] = camera_observation.GetIntensity(p + dp);
            }
        }

        // set error:
        _error = I_camera - landmark.GetIntensities();
    }

    // use analytic Jacobian:
    virtual void linearizeOplus() override {
        // trivial case -- skip:
        if (level()==1) {
            _jacobianOplusXi = Eigen::Matrix<double, 16, 6>::Zero();
            _jacobianOplusXj = Eigen::Matrix<double, 16, 3>::Zero();
            return;
        }

        // parse vertices:
        auto vertex_camera = static_cast<VertexCamera *>(_vertices[0]);
        auto vertex_landmark = static_cast<VertexLandmark *>(_vertices[1]);

        // parse pose and point:
        const CameraWithObservation &camera_observation = vertex_camera->estimate();
        const Landmark &landmark = vertex_landmark->estimate();

        // get patch anchor:
        Eigen::Vector3d X = landmark.GetPosition();
        Eigen::Vector2d p = camera_observation.Project(X);
        const int HALF_PATCH_SIZE = landmark.GetHalfPatchSize();

        // get component matrices from camera observation:
        Eigen::Matrix<double, 2, 6> J_pose;
        Eigen::Matrix<double, 2, 3> J_position;
        camera_observation.GetJacobians(X, J_pose, J_position);

        // get component matrices from landmark:
        Eigen::Matrix3d J_params;
        landmark.GetJacobian(J_params);
        J_position = J_position * J_params;
        
        // get projection:
        for (int dx = -HALF_PATCH_SIZE; dx < HALF_PATCH_SIZE; ++dx) {
            for (int dy = -HALF_PATCH_SIZE; dy < HALF_PATCH_SIZE; ++dy) {
                // get linear index:
                int i = (dx + HALF_PATCH_SIZE) * (HALF_PATCH_SIZE << 1) + (dy + HALF_PATCH_SIZE);

                // get patch location:
                Eigen::Vector2d dp(dx, dy);

                // get image gradient:
                Eigen::Vector2d J_I = camera_observation.GetImageGradient(p + dp);

                // get Jacobian with respect to camera observation:
                Vector6d J_camera = J_I.transpose() * J_pose;

                // get Jacobian with respect to landmark:
                Eigen::Vector3d J_landmark = J_I.transpose() * J_position;

                if (!vertex_camera->fixed()) {
                    _jacobianOplusXi.block<1, 6>(i, 0) = J_camera;
                }
                if (!vertex_landmark->fixed()) {
                    _jacobianOplusXj.block<1, 3>(i, 0) = J_landmark;
                }
            }   
        }     
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};