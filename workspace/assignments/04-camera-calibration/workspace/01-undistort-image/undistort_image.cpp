#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

// default input image:
string image_file = "./test.png";

int main(int argc, char **argv) {
    // undistortion params:
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // intrinsic params:
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    // original image, CV_8UC1
    cv::Mat image = cv::imread(image_file,0);   
    int rows = image.rows, cols = image.cols;
    // undistorted image, CV_8UC1
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); 

    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {
            // normalized camera coordinates:
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;

            // radius:
            double r_squared = x * x + y * y;
            double r_quad = r_squared * r_squared;

            // undistorted camera coordinates:
            double x_undistorted = x*(1 + k1*r_squared + k2*r_quad) + 2*p1*x*y + p2*(r_squared + 2*x*x);
            double y_undistorted = y*(1 + k1*r_squared + k2*r_quad) + 2*p2*x*y + p1*(r_squared + 2*y*y);

            // undistorted pixel coordinates:
            double u_distorted = fx*x_undistorted + cx;
            double v_distorted = fy*y_undistorted + cy;

            // nearest neighbor assignment:
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    // visualize undistorted image
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();

    // save undistorted image:
    cv::imwrite("undistorted.png", image_undistort);

    return 0;
}
