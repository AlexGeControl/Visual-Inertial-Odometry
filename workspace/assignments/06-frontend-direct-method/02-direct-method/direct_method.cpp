#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../data/02-direct-method/left.png";
string disparity_file = "../data/02-direct-method/disparity.png";
boost::format fmt_others("../data/02-direct-method/%06d.png");    // other files
boost::format output_format("./%06d.png");    // output files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Camera {
public:
    Camera(
        double fx, double fy,
        double cx, double cy
    ) : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
    }

    double GetFx(void) const { return fx_; }
    double GetFy(void) const { return fy_; }
    double GetCx(void) const { return cx_; }
    double GetCy(void) const { return cy_; }

    Eigen::Vector3d ToCameraFrame(const Eigen::Vector2d &p, double depth) const {
        double u = p.x(), v = p.y();

        double x_normalized = (u - cx_) / fx_;
        double y_normalized = (v - cy_) / fy_;

        return Eigen::Vector3d(
            x_normalized * depth,
            y_normalized * depth,
            depth
        );
    }

    Eigen::Vector2d ToPixelFrame(const Eigen::Vector3d &p) const {
        double x_normalized = p.x() / p.z();
        double y_normalized = p.y() / p.z();

        return Eigen::Vector2d(
            fx_*x_normalized + cx_,
            fy_*y_normalized + cy_
        );
    }

    Matrix26d GetJacobian(const Eigen::Vector3d &p) const {
        double x = p.x(), y = p.y(), z = p.z();

        Matrix26d J;

        J(0, 0) = +fx_/z;
        J(0, 1) = 0.0;
        J(0, 2) = -fx_*x/(z*z);
        J(0, 3) = -fx_*x*y/(z*z);
        J(0, 4) = +fx_*(1 + x*x/(z*z));
        J(0, 5) = -fx_*(y/z);
        J(1, 0) = 0.0;
        J(1, 1) = +fy_/z;
        J(1, 2) = -fy_*y/(z*z);
        J(1, 3) = -fy_*(1 + y*y/(z*z));
        J(1, 4) = +fy_*x*y/(z*z);
        J(1, 5) = +fy_*x/z;

        return J;
    }

private:
    double fx_, fy_;
    double cx_, cy_;
};

class ImageWithGradient {
public:
    ImageWithGradient(const cv::Mat &image) 
        : H_(image.rows), W_(image.cols)
    {
        I_ = image.clone();
        cv::Scharr(image, I_x_, CV_64F, 1, 0);
        cv::Scharr(image, I_y_, CV_64F, 0, 1);
    }

    bool IsValidPatch(const Eigen::Vector2d &p, int HALF_PATCH_SIZE) const {
        // parse:
        double x = p.x(), y = p.y();

        int x0 = int(x), y0 = int(y);
        int x1 = x0 + 1, y1 = y0 + 1;

        return (
            (HALF_PATCH_SIZE <= x0) && (x1 <= W_ - HALF_PATCH_SIZE) &&
            (HALF_PATCH_SIZE <= y0) && (y1 <= H_ - HALF_PATCH_SIZE)
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

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const Camera &camera,
    const vector<Eigen::Vector3d> &keypoints,
    Sophus::SE3d &T21,
    int MAX_ITERATIONS 
) {
    // parameters
    int HALF_PATCH_SIZE = 4;

    // initialize image handlers:
    ImageWithGradient T(img1);
    ImageWithGradient I(img2);

    double cost = 0, lastCost = 0;
    // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        goodProjection.clear();

        auto R = T21.rotationMatrix();
        auto t = T21.translation();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < keypoints.size(); i++) {
            const Eigen::Vector3d &P_target = keypoints.at(i);

            // project to target frame:
            Eigen::Vector2d p_target = camera.ToPixelFrame(P_target);

            // project to source frame:
            Eigen::Vector3d P_source = R*P_target + t;
            Eigen::Vector2d p_source = camera.ToPixelFrame(P_source);

            // check validity:
            if (I.IsValidPatch(p_source, HALF_PATCH_SIZE)) {
                goodProjection.push_back(p_source);
            } else {
                continue;
            }

            // Jacobian of pixel coordinates with respect to se3:
            Matrix26d J_pixel_xi = camera.GetJacobian(P_source);

            // and compute error and jacobian
            for (int x = -HALF_PATCH_SIZE; x < HALF_PATCH_SIZE; x++) {
                for (int y = -HALF_PATCH_SIZE; y < HALF_PATCH_SIZE; y++) {
                    Eigen::Vector2d p_target_ = Eigen::Vector2d(p_target.x() + x, p_target.y() + y);
                    Eigen::Vector2d p_source_ = Eigen::Vector2d(p_source.x() + x, p_source.y() + y);

                    double error = T.GetPixelValue(p_target_) - I.GetPixelValue(p_source_);

                    // image Jacobian:
                    Eigen::Vector2d J_img_pixel = I.GetGradient(p_source_);    

                    // total jacobian
                    Vector6d J = J_img_pixel.transpose() * J_pixel_xi;

                    // update H, b and cost:
                    H += J * J.transpose();
                    b += error * J;

                    cost += 0.5 * error*error;
                }
            }
        }

        const size_t N = goodProjection.size();

        // average over all good projections:
        cost /= N;

        // solve update and put it into estimation
        Vector6d update = H.fullPivHouseholderQr().solve(b);
        T21 = Sophus::SE3d::exp(update) * T21;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost >= lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        // cout << "cost = " << cost << ", good = " << goodProjection.size() << " at " << iter << endl;
    }
    // cout << "good projection: " << goodProjection.size() << endl;
    // cout << "T21 = \n" << T21.matrix() << endl;
}

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
    const cv::Mat &image_target,
    const cv::Mat &image_source,
    Camera &camera,
    std::vector<Eigen::Vector3d> &keypoints,
    Sophus::SE3d &T21,
    int MAX_ITERATIONS 
) {
    // parameters
    int NUM_PYRAMIDS = 4;
    double PYRAMID_SCALE = 0.5;

    // create image pyramids
    vector<cv::Mat> pyramid_target, pyramid_source;

    pyramid_target.push_back(image_target);
    pyramid_source.push_back(image_source);

    for (int level = 0; level < NUM_PYRAMIDS; ++level) {
        const cv::Mat &image_target_ = pyramid_target.at(level);
        const cv::Mat &image_source_ = pyramid_source.at(level);

        cv::Mat image_target_downsampled;
        cv::Mat image_source_downsampled;

        cv::pyrDown(
            image_target_, image_target_downsampled, 
            cv::Size(PYRAMID_SCALE*image_target_.cols, PYRAMID_SCALE*image_target_.rows)
        );
        cv::pyrDown(
            image_source_, image_source_downsampled, 
            cv::Size(PYRAMID_SCALE*image_source_.cols, PYRAMID_SCALE*image_source_.rows)
        );

        pyramid_target.push_back(image_target_downsampled);
        pyramid_source.push_back(image_source_downsampled);
    }

    for (int level = NUM_PYRAMIDS - 1; level >= 0; --level) {
        double scale = pow(PYRAMID_SCALE, level);

        cv::Mat &image_target_ = pyramid_target.at(level);
        cv::Mat &image_source_ = pyramid_source.at(level);

        Camera camera_(
            scale*camera.GetFx(), scale*camera.GetFy(), scale*camera.GetCx(), scale*camera.GetCy()
        );

        DirectPoseEstimationSingleLayer(image_target_, image_source_, camera_, keypoints, T21, MAX_ITERATIONS);
    }
}

void DrawProjection(
    const cv::Mat &image, 
    const vector<Eigen::Vector3d> &keypoints, 
    const Sophus::SE3d &T,
    const Camera &camera,
    cv::Mat &canvas
) {
    // initialize canvas:
    cv::cvtColor(image, canvas, CV_GRAY2BGR);

    for (auto &P_target: keypoints) {
        auto P_source = T.rotationMatrix()*P_target + T.translation();
        auto p_source = camera.ToPixelFrame(P_source);

        if (
            (2 <= p_source.x()) && (p_source.x() < image.cols - 2) && 
            (2 <= p_source.y()) && (p_source.y() < image.rows - 2)
        ) {
            cv::rectangle(
                canvas, 
                cv::Point2f(p_source.x() - 2, p_source.y() - 2), 
                cv::Point2f(p_source.x() + 2, p_source.y() + 2),
                cv::Scalar(0, 250, 0)
            );
        }
    }
}

int main(int argc, char **argv) {
    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);
    Camera camera(fx, fy, cx, cy);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    vector<Eigen::Vector3d> keypoints;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        // sample in target:
        int x = rng.uniform(boarder, left_img.cols - boarder);  
        int y = rng.uniform(boarder, left_img.rows - boarder);
        // restore depty:
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity;
        // add to keypoints:
        keypoints.push_back(
            camera.ToCameraFrame(
                Eigen::Vector2d(x, y), depth
            )
        );
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;

    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        // single layer:
        DirectPoseEstimationSingleLayer(left_img, img, camera, keypoints, T_cur_ref, 100);
        // multi-layer:
        // DirectPoseEstimationMultiLayer(left_img, img, camera, keypoints, T_cur_ref, 20);
        cout << "Image " << (fmt_others % i).str() << ", Pose = \n" << T_cur_ref.matrix() << endl;
        cv::Mat tracked;
        DrawProjection(img, keypoints, T_cur_ref, camera, tracked);
        cv::imwrite((output_format % i).str() ,tracked);
    }
}
