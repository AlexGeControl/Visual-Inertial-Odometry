#pragma once

// OpenCV:
#include <opencv2/opencv.hpp>
// pose manipulation:
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/se3_ops.h>

typedef Eigen::Matrix<double, 2, 3> Matrix23d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct CameraIntrinsics {
    CameraIntrinsics(void) {}

    CameraIntrinsics(const CameraIntrinsics &other) {
        fx_ = other.fx_;
        fy_ = other.fy_;
        cx_ = other.cx_;
        cy_ = other.cy_;
    }

    CameraIntrinsics &operator=(const CameraIntrinsics &other) {
        fx_ = other.fx_;
        fy_ = other.fy_;
        cx_ = other.cx_;
        cy_ = other.cy_;
    }

    CameraIntrinsics(
        double fx, double fy,
        double cx, double cy
    ): fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

    double GetFx(void) const { return fx_; }
    double GetFy(void) const { return fy_; }
    double GetCx(void) const { return cx_; }
    double GetCy(void) const { return cy_; }

    double fx_, fy_;
    double cx_, cy_;
};

class ImageWithGradient {
public:
    ImageWithGradient(void) {}

    ImageWithGradient(const cv::Mat &image) 
        : H_(image.rows), W_(image.cols)
    {
        I_ = image.clone();
        cv::Scharr(image, I_x_, CV_64F, 1, 0);
        cv::Scharr(image, I_y_, CV_64F, 0, 1);
    }

    ImageWithGradient(const ImageWithGradient &other) {
        H_ = other.H_;
        W_ = other.W_;
        I_ = other.I_.clone();
        I_x_ = other.I_x_.clone();
        I_y_ = other.I_y_.clone();        
    }

    ImageWithGradient &operator=(const ImageWithGradient &other) {
        H_ = other.H_;
        W_ = other.W_;
        I_ = other.I_.clone();
        I_x_ = other.I_x_.clone();
        I_y_ = other.I_y_.clone();  
    }

    bool IsValidPatch(const Eigen::Vector2d &p, int HALF_PATCH_SIZE) const {
        // parse:
        double x = p.x(), y = p.y();

        int x0 = int(x), y0 = int(y);
        int x1 = x0 + 1, y1 = y0 + 1;

        return (
            (HALF_PATCH_SIZE + 1 <= x0) && (x1 <= W_ - HALF_PATCH_SIZE - 1) &&
            (HALF_PATCH_SIZE + 1 <= y0) && (y1 <= H_ - HALF_PATCH_SIZE - 1)
        );
    }

    double GetPixelValue(const Eigen::Vector2d &p) const {
        // parse:
        double x = p.x(), y = p.y();

        // initialize grid:
        int x0 = int(x), y0 = int(y);
        int x1 = x0 + 1, y1 = y0 + 1;

        // initialize coefficient:
        double xx = x - x0;
        double yy = y - y0;
        
        double I = (
                (1 - xx) * (1 - yy) * I_.at<uchar>(y0, x0) +
                    (xx) * (1 - yy) * I_.at<uchar>(y0, x1) +
                (1 - xx) *     (yy) * I_.at<uchar>(y1, x0) +
                    (xx) *     (yy) * I_.at<uchar>(y1, x1)
        );

        return I;
    }

    Eigen::Vector2d GetGradient(const Eigen::Vector2d &p) const {
        // parse:
        double x = p.x(), y = p.y();    

        // initialize grid:
        int x0 = int(x), y0 = int(y);
        int x1 = x0 + 1, y1 = y0 + 1;

        // initialize coefficient:
        double xx = x - x0;
        double yy = y - y0;

        // Jabobian, x:
        double J_x = (
            (1.0 - xx) * (1.0 - yy) * I_x_.at<double>(y0, x0) +
                  (xx) * (1.0 - yy) * I_x_.at<double>(y0, x1) + 
            (1.0 - xx) *       (yy) * I_x_.at<double>(y1, x0) +
                  (xx) *       (yy) * I_x_.at<double>(y1, x1)
        );        
        // Jabobian, y:
        double J_y = (
            (1.0 - xx) * (1.0 - yy) * I_y_.at<double>(y0, x0) +
                  (xx) * (1.0 - yy) * I_y_.at<double>(y0, x1) + 
            (1.0 - xx) *       (yy) * I_y_.at<double>(y1, x0) +
                  (xx) *       (yy) * I_y_.at<double>(y1, x1)
        );

        Eigen::Vector2d J(J_x, J_y);

        return J / 26.0;
    }

private:
    size_t H_;
    size_t W_;
    cv::Mat I_;
    cv::Mat I_x_;
    cv::Mat I_y_;
};

class CameraWithObservation {
public:
    CameraWithObservation(void) {}

    CameraWithObservation(const CameraWithObservation &other) {
        intrinsics_ = other.intrinsics_;
        T_ = other.T_;
        observation_ = other.observation_;
    }

    CameraWithObservation &operator=(const CameraWithObservation &other) {
        intrinsics_ = other.intrinsics_;
        T_ = other.T_;
        observation_ = other.observation_;
    }

    CameraWithObservation(
        // camera intrinsics:
        double fx, double fy,
        double cx, double cy,
        // camera pose:
        const Eigen::Quaterniond &q, const Eigen::Vector3d &t,
        // image observation:
        const cv::Mat &image
    ) : intrinsics_(fx, fy, cx, cy), 
        T_(q, t),
        observation_(image) {
    }

    Eigen::Vector2d Project(const Eigen::Vector3d &X) const {
        // map to camera coordinates:
        Eigen::Vector3d P = T_.map(X);
        // normalize:
        double x_normalized = P.x() / P.z();
        double y_normalized = P.y() / P.z();
        // map to pixel coordinates:
        Eigen::Vector2d p(
            intrinsics_.GetFx()*x_normalized + intrinsics_.GetCx(),
            intrinsics_.GetFy()*y_normalized + intrinsics_.GetCy()
        );

        return p;
    }

    bool IsValidPatch(const Eigen::Vector2d &p, int HALF_PATCH_SIZE) const {
        return observation_.IsValidPatch(p, HALF_PATCH_SIZE);
    }

    g2o::SE3Quat GetPose(void) const {
        return T_;
    }

    double GetIntensity(const Eigen::Vector2d &p) const {
        return observation_.GetPixelValue(p);
    }

    void GetJacobians(const Eigen::Vector3d &X, Eigen::Matrix<double, 2, 6> &J_pose, Eigen::Matrix<double, 2, 3> &J_position) const {
        //
        // intermediate variables
        //
        // map to camera coordinates:
        Eigen::Vector3d P = T_.map(X);

        // get intermediate variables:
        double fx = intrinsics_.GetFx();
        double fy = intrinsics_.GetFy();

        double P_X = P.x(), P_Y = P.y(), P_Z = P.z();

        double P_Z_2 = P_Z*P_Z;

        Matrix23d J_intermediate;

        J_intermediate(0, 0) = fx/P_Z;
        J_intermediate(0, 1) = 0;
        J_intermediate(0, 2) = -P_X*fx/P_Z_2;
        J_intermediate(1, 0) = 0;
        J_intermediate(1, 1) = fy/P_Z;
        J_intermediate(1, 2) = -P_Y*fy/P_Z_2;

        //
        // a. Jacobian with respect to camera pose
        //
        // Jacobian with respect to camera pose:
        J_pose.block<2, 3>(0, 0) = J_intermediate;
        J_pose.block<2, 3>(0, 3) = -J_intermediate * g2o::skew(P);

        //
        // b. Jacobian with respect to landmark position
        //
        J_position = J_intermediate * T_.rotation().toRotationMatrix();
    }

    Eigen::Vector2d GetImageGradient(const Eigen::Vector2d &p) const {
        return observation_.GetGradient(p);
    }

    void UpdatePose(const g2o::SE3Quat &dT) {
        T_ = dT * T_;
    }

private:
    CameraIntrinsics intrinsics_;
    g2o::SE3Quat T_;
    ImageWithGradient observation_;
};