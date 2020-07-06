#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <math.h>

typedef Eigen::Matrix<double,16,1> Vector16d;


class Landmark {
public:
    Landmark(void) {}

    Landmark(const Landmark &other) {
        P_ = other.P_;
        HALF_PATCH_SIZE_ = other.HALF_PATCH_SIZE_;

        W_.clear();
        for (const double &w: other.W_) {
            W_.push_back(w);
        }
    }

    Landmark &operator=(const Landmark &other) {
        P_ = other.P_;
        HALF_PATCH_SIZE_ = other.HALF_PATCH_SIZE_;

        W_.clear();
        for (const double &w: other.W_) {
            W_.push_back(w);
        }
    }

    Landmark(const Eigen::Vector3d &X, const int HALF_PATCH_SIZE, const std::vector<double> &W)
    : HALF_PATCH_SIZE_(HALF_PATCH_SIZE), W_(W) {
        // convert to inverse depth representation:
        double &theta = P_[0];
        double &fie = P_[1];
        double &rho = P_[2];

        theta = atan2(X.y(), X.x());
        rho = 1.0/X.norm();
        fie = acos(X.z()*rho);
    }

    Eigen::Vector3d GetPosition(void) const { 
        Eigen::Vector3d X;

        const double &theta = P_[0];
        const double &fie = P_[1];
        const double &rho = P_[2];

        X.x() = 1.0/rho*cos(theta)*sin(fie);
        X.y() = 1.0/rho*sin(theta)*sin(fie);
        X.z() = 1.0/rho*cos(fie);

        return X;
    }

    void GetJacobian(Eigen::Matrix3d &J) const {
        const double &theta = P_[0];
        const double &fie = P_[1];
        const double &rho = P_[2];

        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double sin_fie = sin(fie);
        double cos_fie = cos(fie);
        double rho_2 = rho * rho;

        J(0, 0) = -sin_fie*sin(theta)/rho;
        J(0, 1) = cos_fie*cos(theta)/rho;
        J(0, 2) = -sin_fie*cos(theta)/rho_2;
        J(1, 0) = sin_fie*cos(theta)/rho;
        J(1, 1) = sin(theta)*cos_fie/rho;
        J(1, 2) = -sin_fie*sin(theta)/rho_2;
        J(2, 0) = 0;
        J(2, 1) = -sin_fie/rho;
        J(2, 2) = -cos_fie/rho_2;
    }

    int GetHalfPatchSize(void) const { return HALF_PATCH_SIZE_; }

    Vector16d GetIntensities(void) const {
        Vector16d I;

        for (int dx = -HALF_PATCH_SIZE_; dx < HALF_PATCH_SIZE_; ++dx) {
            for (int dy = -HALF_PATCH_SIZE_; dy < HALF_PATCH_SIZE_; ++dy) {
                // get linear index:
                int i = (dx + HALF_PATCH_SIZE_) * (HALF_PATCH_SIZE_ << 1) + (dy + HALF_PATCH_SIZE_);

                // get error:
                I[i] = W_.at(i);
            }
        }

        return I;
    }

    void UpdatePosition(const Eigen::Vector3d dP) {
        P_ += dP;
    }

private:
    // here inverse depth parameterization is used:
    Eigen::Vector3d P_;
    int HALF_PATCH_SIZE_;
    std::vector<double> W_;
};