#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// loss function for parameter estimation:
struct LOSS {
    LOSS(double x, double y): _x(x), _y(y) {
    }

    template <typename T>
    bool operator() (
        const T *const params,
        T *residual
    ) const {
        residual[0] = _y - ceres::exp(params[0]*T(_x)*T(_x) + params[1]*T(_x) + params[2]);

        return true;
    }

    const double _x, _y;
};

int main(int argc, char **argv) {
    // ground truth params:
    double ground_truth[3] = {1.0, 2.0, 1.0};

    // num points: 
    int N = 100;
    // noise standard deviation:                                 
    double w_sigma = 1.0;                        
    // OpenCV random number generator:
    cv::RNG rng;                                 

    // observations:
    vector<double> x_data, y_data;      
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ground_truth[0]*x*x + ground_truth[1]*x + ground_truth[2]) + rng.gaussian(w_sigma));
    }

    // params:
    double estimation[3];

    // init problem:
    ceres::Problem problem;
    for (int i = 0; i < N; ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<LOSS, 1, 3>(
                new LOSS(x_data[i], y_data[i])
            ),
            nullptr,
            estimation
        );
    }

    // config solver:
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point end_time = chrono::steady_clock::now();

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(end_time - start_time);
    cout << "Ceres solver costs " << time_used.count() << " seconds." << endl;

    // compare ground truth with estimation:
    cout << summary.BriefReport() << endl;
    cout << "Ground Truth: " << endl;
    for (auto g: ground_truth) {
        cout << g << ", ";
    }
    cout << endl;
    cout << "Estimated: " << endl;
    for (auto e: estimation) {
        cout << e << ", ";
    }
    cout << endl;  

    return 0;
}