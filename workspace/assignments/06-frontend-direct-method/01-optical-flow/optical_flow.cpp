#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class ImageWithGradient {
public:
    ImageWithGradient(const cv::Mat &image) 
        : H_(image.rows), W_(image.cols)
    {
        I_ = image.clone();
        cv::Scharr(image, I_x_, CV_64F, 1, 0);
        cv::Scharr(image, I_y_, CV_64F, 0, 1);
    }

    bool IsValidPatch(double x, double y, int HALF_PATCH_SIZE) {
        int x0 = int(x), y0 = int(y);
        int x1 = x0 + 1, y1 = y0 + 1;

        return (
            (HALF_PATCH_SIZE <= x0) && (x1 <= W_ - HALF_PATCH_SIZE) &&
            (HALF_PATCH_SIZE <= y0) && (y1 <= H_ - HALF_PATCH_SIZE)
        );
    }

    double GetPixelValue(double x, double y) {
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

    Eigen::Vector2d GetGradient(double x, double y) {
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

struct State {
    State(double dx, double dy) {
        p = Eigen::Vector2d(dx, dy);
        cost = std::numeric_limits<double>::max();
        success = false;
        is_inverse_initialized = false;
    }

    bool IsValidUpdate(const Eigen::Vector2d &dp) {
        return !(isnan(dp(0)) || isnan(dp(1)));
    }

    bool Update(const Eigen::Vector2d &dp, const double &cost) {
        // whether the update is NaN:
        if (!IsValidUpdate(dp)) {
            success = false;
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "[Optical Flow]: Parameter update is NaN. Aborted." << endl;

            return false;
        }

        // cost increased. trigger early stopping:
        if (this->cost <= cost) {
            // cout << "[Optical Flow]: Cost increased: current--" 
            //      << cost << ", last--" << this->cost 
            //      << ". Trigger early stopping." << endl;
            return false;
        }

        // update parameter:
        p = p + dp;
        success = true;

        return true;
    }

    Eigen::Vector2d p;
    double cost;
    bool success;
    bool is_inverse_initialized;
};

// this program shows how to use optical flow

string file_1 = "../data/01-optical-flow/1.png";  // first image
string file_2 = "../data/01-optical-flow/2.png";  // second image

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);


int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single_forward;
    vector<bool> success_single_forward;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single_forward, success_single_forward, false);

    vector<KeyPoint> kp2_single_inverse;
    vector<bool> success_single_inverse;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single_inverse, success_single_inverse, true);

    // then test multi-level LK
    vector<KeyPoint> kp2_multi_forward;
    vector<bool> success_multi_forward;
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi_forward, success_multi_forward, false);

    vector<KeyPoint> kp2_multi_inverse;
    vector<bool> success_multi_inverse;
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi_inverse, success_multi_inverse, true);

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

    // plot the differences of those functions
    Mat img2_single_forward;
    cv::cvtColor(img2, img2_single_forward, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single_forward.size(); i++) {
        if (success_single_forward[i]) {
            cv::circle(img2_single_forward, kp2_single_forward[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single_forward, kp1[i].pt, kp2_single_forward[i].pt, cv::Scalar(0, 250, 0));
        }
    }
    Mat img2_single_inverse;
    cv::cvtColor(img2, img2_single_inverse, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single_inverse.size(); i++) {
        if (success_single_inverse[i]) {
            cv::circle(img2_single_inverse, kp2_single_inverse[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single_inverse, kp1[i].pt, kp2_single_inverse[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_multi_forward;
    cv::cvtColor(img2, img2_multi_forward, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi_forward.size(); i++) {
        if (success_multi_forward[i]) {
            cv::circle(img2_multi_forward, kp2_multi_forward[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi_forward, kp1[i].pt, kp2_multi_forward[i].pt, cv::Scalar(0, 250, 0));
        }
    }
    Mat img2_multi_inverse;
    cv::cvtColor(img2, img2_multi_inverse, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi_inverse.size(); i++) {
        if (success_multi_inverse[i]) {
            cv::circle(img2_multi_inverse, kp2_multi_inverse[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi_inverse, kp1[i].pt, kp2_multi_inverse[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("Tracked Single Level, Forward", img2_single_forward);
    cv::imwrite("optical-flow--single-level-forward.png", img2_single_forward);

    cv::imshow("Tracked Single Level, Inverse", img2_single_inverse);
    cv::imwrite("optical-flow--single-level-inverse.png", img2_single_inverse);

    cv::imshow("Tracked Multi Level, Forward", img2_multi_forward);
    cv::imwrite("optical-flow--multi-level-forward.png", img2_multi_forward);

    cv::imshow("Tracked Multi Level, Inverse", img2_multi_inverse);
    cv::imwrite("optical-flow--multi-level-inverse.png", img2_multi_inverse);

    cv::imshow("Tracked by OpenCV", img2_CV);
    cv::imwrite("optical-flow--opencv.png", img2_multi_inverse);

    cv::waitKey(0);

    return 0;
}


void OpticalFlowSingleLevel(
        const Mat &image_target,
        const Mat &image_source,
        const vector<KeyPoint> &keypoints_target,
        vector<KeyPoint> &keypoints_source,
        vector<bool> &success,
        bool inverse
) {
    // parameters
    int HALF_PATCH_SIZE = 4;
    int MAX_ITERATIONS = 10;
    bool have_initial = !keypoints_source.empty();

    // initialize images:
    ImageWithGradient T(image_target);
    ImageWithGradient I(image_source);

    for (size_t i = 0; i < keypoints_target.size(); ++i) {
        const auto &keypoint = keypoints_target[i];

        // initialize x0 and y0:
        double x0{keypoint.pt.x}, y0{keypoint.pt.y};

        // initialize dx and dy:
        double dx{0.0}, dy{0.0};
        if (have_initial) {
            dx = keypoints_source.at(i).pt.x - x0;
            dy = keypoints_source.at(i).pt.y - y0;
        }

        // initialize Hessian:
        Eigen::Matrix2d H;

        State state(dx, dy);

        // Gauss-Newton iterations
        if (T.IsValidPatch(x0, y0, HALF_PATCH_SIZE)) {
            for (int j = 0; j < MAX_ITERATIONS; j++) {
                dx = state.p(0), dy = state.p(1);
                double x = x0 + dx;
                double y = y0 + dy;

                // whether the current patch is inside the image:
                if (!I.IsValidPatch(x, y, HALF_PATCH_SIZE)) {
                    state.success = false;
                    break;
                }

                Eigen::Vector2d b = Eigen::Vector2d::Zero();

                if (!inverse) {
                    H = Eigen::Matrix2d::Zero();
                } else {
                    if (!state.is_inverse_initialized) {
                        // for inverse method the approximated Hessian will not change for each patch
                        H = Eigen::Matrix2d::Zero();
                        for (int du = -HALF_PATCH_SIZE; du < HALF_PATCH_SIZE; ++du) {
                            for (int dv = -HALF_PATCH_SIZE; dv < HALF_PATCH_SIZE; ++dv) {
                                Eigen::Vector2d J = T.GetGradient(x0 + du, y0 + dv);
                                H += J * J.transpose(); 
                            }
                        }
                        state.is_inverse_initialized = true;
                    }
                }   
                
                double cost = 0.0;

                // compute cost and jacobian:
                Eigen::Vector2d J;
                for (int du = -HALF_PATCH_SIZE; du < HALF_PATCH_SIZE; ++du) {
                    for (int dv = -HALF_PATCH_SIZE; dv < HALF_PATCH_SIZE; ++dv) {
                        // compute error:
                        double error = T.GetPixelValue(x0 + du, y0 + dv) - I.GetPixelValue(x + du, y + dv);

                        // compute Jacobian:
                        if (!inverse) {
                            J = I.GetGradient(x + du, y + dv);
                            H += J * J.transpose();
                        } else {
                            J = T.GetGradient(x0 + du, y0 + dv);
                        }

                        // update b and cost;
                        b += error * J;

                        cost += 0.5 * error*error;
                    }
                }

                // compute update
                Eigen::Vector2d dp = H.fullPivHouseholderQr().solve(b);

                if (!state.Update(dp, cost)) {
                    break;
                }
            }
        }

        success.push_back(state.success);

        // set source keypoint:
        dx = state.p(0), dy = state.p(1);
        if (have_initial) {
            keypoints_source.at(i).pt = keypoint.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = keypoint;
            tracked.pt += cv::Point2f(dx, dy);
            keypoints_source.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &image_target,
        const Mat &image_source,
        const vector<KeyPoint> &keypoints_target,
        vector<KeyPoint> &keypoints_source,
        vector<bool> &success,
        bool inverse
) {
    // parameters
    int NUM_PYRAMIDS = 4;
    double PYRAMID_SCALE = 0.5;

    // get input config:
    const size_t N = keypoints_target.size();
    bool has_initial = (keypoints_source.size() != 0);

    // create pyramids
    vector<Mat> pyramid_target, pyramid_source;
    vector<cv::KeyPoint> keypoints_target_;

    pyramid_target.push_back(image_target);
    pyramid_source.push_back(image_source);

    for (int i = 0; i < NUM_PYRAMIDS - 1; i++) {
        const cv::Mat &image_target_ = pyramid_target.at(i);
        const cv::Mat &image_source_ = pyramid_source.at(i);

        cv::Mat image_target_downsampled;
        cv::Mat image_source_downsampled;

        cv::pyrDown(image_target_, image_target_downsampled, cv::Size(PYRAMID_SCALE*image_target_.cols, PYRAMID_SCALE*image_target_.rows));
        cv::pyrDown(image_source_, image_source_downsampled, cv::Size(PYRAMID_SCALE*image_source_.cols, PYRAMID_SCALE*image_source_.rows));

        pyramid_target.push_back(image_target_downsampled);
        pyramid_source.push_back(image_source_downsampled);
    }

    // perform coarse-to-fine estimation:
    for (int i = NUM_PYRAMIDS - 1; 0 <= i; --i) {
        // compute current scale:
        double scale = pow(PYRAMID_SCALE, i);
        
        const cv::Mat &image_target_ = pyramid_target.at(i);
        const cv::Mat &image_source_ = pyramid_source.at(i);

        // scale keypoints:
        for (size_t j = 0; j < N; ++j) {
            cv::KeyPoint keypoint_target_ = keypoints_target.at(j);

            keypoint_target_.pt.x *= scale;
            keypoint_target_.pt.y *= scale;

            keypoints_target_.push_back(keypoint_target_);
        }

        // optical flow estimation:
        OpticalFlowSingleLevel(
            image_target_, image_source_,
            keypoints_target_, keypoints_source,
            success, inverse
        );

        // already reach the original scale, terminate:
        if (i == 0) { break; }
        
        // clear keypoint target:
        keypoints_target_.clear();

        // scale back for next iteration:
        for (size_t j = 0; j < N; ++j) {
            keypoints_source.at(j).pt.x /= PYRAMID_SCALE;
            keypoints_source.at(j).pt.y /= PYRAMID_SCALE;
        }

        // clear indicators:
        success.clear();
    }
}
