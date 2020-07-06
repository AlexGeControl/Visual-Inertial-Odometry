#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>


typedef Eigen::Matrix<double,16,1> Vector16d;


class Landmark {
public:
    Landmark(void) {}

    Landmark(const Landmark &other) {
        X_ = other.X_;
        HALF_PATCH_SIZE_ = other.HALF_PATCH_SIZE_;

        W_.clear();
        for (const double &w: other.W_) {
            W_.push_back(w);
        }
    }

    Landmark &operator=(const Landmark &other) {
        X_ = other.X_;
        HALF_PATCH_SIZE_ = other.HALF_PATCH_SIZE_;

        W_.clear();
        for (const double &w: other.W_) {
            W_.push_back(w);
        }
    }

    Landmark(const Eigen::Vector3d &X, const int HALF_PATCH_SIZE, const std::vector<double> &W)
    : X_(X), HALF_PATCH_SIZE_(HALF_PATCH_SIZE), W_(W) {}

    Eigen::Vector3d GetPosition(void) const { return X_; }

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

    void UpdatePosition(const Eigen::Vector3d dX) {
        X_ += dX;
    }

private:
    Eigen::Vector3d X_;
    int HALF_PATCH_SIZE_;
    std::vector<double> W_;
};