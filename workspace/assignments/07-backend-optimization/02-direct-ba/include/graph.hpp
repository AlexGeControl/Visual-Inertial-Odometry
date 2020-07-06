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
        auto vertex_camera = static_cast<VertexCamera *>(_vertices[0]);
        auto vertex_landmark = static_cast<VertexLandmark *>(_vertices[1]);

        // get patch anchor:
        Eigen::Vector3d X = vertex_landmark->estimate().GetPosition();
        const int HALF_PATCH_SIZE = vertex_landmark->estimate().GetHalfPatchSize();

        if (!vertex_camera->estimate().IsValidLandmark(X, HALF_PATCH_SIZE)) {
            // stop optimization:
            setLevel(1);
            _error = Vector16d::Zero();
            return;
        }

        Eigen::Vector2d p_anchor = vertex_camera->estimate().Project(X);

        // get projection:
        Vector16d I_camera;
        for (int dx = -HALF_PATCH_SIZE; dx < HALF_PATCH_SIZE; ++dx) {
            for (int dy = -HALF_PATCH_SIZE; dy < HALF_PATCH_SIZE; ++dy) {
                // get linear index:
                int i = (dx + HALF_PATCH_SIZE) * (HALF_PATCH_SIZE << 1) + (dy + HALF_PATCH_SIZE);

                // get current pixel:
                Eigen::Vector2d p(
                    p_anchor.x() + dx,
                    p_anchor.y() + dy
                );

                // pixel intensity in observation:
                I_camera[i] = vertex_camera->estimate().GetIntensity(p);
            }
        }

        // set error:
        _error = I_camera - _measurement;
    }

    // use analytic Jacobian:
    virtual void linearizeOplus() override {
        auto vertex_camera = static_cast<VertexCamera *>(_vertices[0]);
        auto vertex_landmark = static_cast<VertexLandmark *>(_vertices[1]);

        Eigen::Vector3d X = vertex_landmark->estimate().GetPosition();
        const int HALF_PATCH_SIZE = vertex_landmark->estimate().GetHalfPatchSize();

        // shall the optimization be stopped:
        if (level()==1) {
            _jacobianOplusXi = Eigen::Matrix<double, 16, 6>::Zero();
            _jacobianOplusXj = Eigen::Matrix<double, 16, 3>::Zero();
            return;
        }

        Eigen::Matrix<double, 2, 6> J_pose;
        Eigen::Matrix<double, 2, 3> J_position;
        vertex_camera->estimate().GetJacobians(X, J_pose, J_position);
        Eigen::Matrix3d J_position_parameterization;
        vertex_landmark->estimate().GetJacobian(J_position_parameterization);
        J_position = J_position * J_position_parameterization;
        
        Eigen::Vector2d p_anchor = vertex_camera->estimate().Project(X);
        for (int dx = -HALF_PATCH_SIZE; dx < HALF_PATCH_SIZE; ++dx) {
            for (int dy = -HALF_PATCH_SIZE; dy < HALF_PATCH_SIZE; ++dy) {
                // get linear index:
                int i = (dx + HALF_PATCH_SIZE) * (HALF_PATCH_SIZE << 1) + (dy + HALF_PATCH_SIZE);

                // get current pixel:
                Eigen::Vector2d p(
                    p_anchor.x() + dx,
                    p_anchor.y() + dy
                );

                Eigen::Vector2d J_I = vertex_camera->estimate().GetImageGradient(p);

                Vector6d J_camera = J_I.transpose() * J_pose;
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