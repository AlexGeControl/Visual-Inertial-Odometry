#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    // ground truth params:
    double ar = 1.0, br = 2.0, cr = 1.0;

    // num points: 
    int N = 100;
    // noise standard deviation:                                 
    double w_sigma = 1.0;                        
    // OpenCV random number generator:
    cv::RNG rng;                                 

    // observations:
    vector<double> x_data, y_data;      
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    // Gauss-Newton iteration:
    // a. initial params:
    double ae = 2.0, be = -1.0, ce = 5.0;
    // b. max iterations:
    int iterations = 100;    
    // c. costs:
    double cost = 0, lastCost = 0;  

    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++) {
        MatrixXd H = MatrixXd::Zero(3, 3);             // Hessian = J^T J in Gauss-Newton
        VectorXd b = VectorXd::Zero(3);             // bias
        cost = 0;

        for (int i = 0; i < N; i++) {
            // observation i:
            double xi = x_data[i], yi = y_data[i];
            double zi = exp(ae*xi*xi + be*xi + ce);
            // error i:
            double error = yi - zi;   

            // Jacobian i:
            VectorXd J = VectorXd::Zero(3); 
            J[0] = -zi * xi*xi;
            J[1] = -zi * xi;
            J[2] = -zi;

            // Hessian i:
            H += J * J.transpose(); 
            b += - error * J;

            cost += error * error;
        }

        // Solve Hx=d using LDLT:
        H /= N; b /= N;

        VectorXd dx = H.ldlt().solve(b);

        // solution is nan: abort
        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        // increased cost: abort
        if (iter > 0 && cost > lastCost) {
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update estimation:
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << endl;
    }
    chrono::steady_clock::time_point end_time = chrono::steady_clock::now();

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(end_time - start_time);
    cout << "Gauss-Newton solver costs " << time_used.count() << " seconds." << endl;

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}