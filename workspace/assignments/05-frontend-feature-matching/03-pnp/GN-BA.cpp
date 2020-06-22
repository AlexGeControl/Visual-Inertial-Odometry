//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <iomanip>

#include <sophus/se3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p2d_file = "../data/03-pnp/p2d.txt";
string p3d_file = "../data/03-pnp/p3d.txt";

void LoadP2d(std::string input_file_name, VecVector2d &p2d) {
    // load all values into buffer:
    std::vector<double> values;

    std::ifstream input_file(input_file_name);
    std::istream_iterator<double> input(input_file);
    std::copy(
        input, std::istream_iterator<double>(), std::back_inserter(values)
    );

    // format as pixel coordinates:
    size_t N = values.size() / 2;
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector2d p{values.at(2*i + 0), values.at(2*i + 1)};
        p2d.push_back(p);
    }
}

void LoadP3d(std::string input_file_name, VecVector3d &p3d) {
    // load all values into buffer:
    std::vector<double> values;

    std::ifstream input_file(input_file_name);
    std::istream_iterator<double> input(input_file);
    std::copy(
        input, std::istream_iterator<double>(), std::back_inserter(values)
    );

    // format as point coordinates:
    size_t N = values.size() / 3;
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3d p{values.at(3*i + 0), values.at(3*i + 1), values.at(3*i + 2)};
        p3d.push_back(p);
    }
}

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    // camera params:
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    // intrinsic matrix:
    K << fx,  0, cx, 
          0, fy, cy, 
          0,  0,  1;

    // load points in to p3d and p2d 
    LoadP2d(p2d_file, p2d);
    LoadP3d(p3d_file, p3d);

    assert(p3d.size() == p2d.size());
    const size_t N = p3d.size();
    int iterations = 100;
    cout << "[PnP G-N]: Num. of points: " << N << ". Max Iternation:" << iterations << endl;

    // init estimated pose
    double cost{0.0}, lastCost{std::numeric_limits<double>::max()};
    Sophus::SE3<double, Eigen::AutoAlign> T_esti; 

    for (int iter = 0; iter < iterations; iter++) {
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;

        for (int i = 0; i < N; i++) {
            // update cost:
            Vector3d P = T_esti.rotationMatrix()*p3d.at(i) + T_esti.translation();
            Vector3d P_esti = K*P;
            Vector2d p_esti{P_esti(0)/P_esti(2), P_esti(1)/P_esti(2)};

            Vector2d e = p2d.at(i) - p_esti;

            cost += 0.5*e.squaredNorm();

	        // compute jacobian
            Matrix<double, 2, 6> J;
            
            double x{P(0)}, y{P(1)}, z{P(2)};

            J(0, 0) = +fx/z;
            J(0, 1) = 0.0;
            J(0, 2) = -fx*x/(z*z);
            J(0, 3) = -fx*x*y/(z*z);
            J(0, 4) = +fx*(1 + x*x/(z*z));
            J(0, 5) = -fx*(y/z);
            J(1, 0) = 0.0;
            J(1, 1) = +fy/z;
            J(1, 2) = -fy*y/(z*z);
            J(1, 3) = -fy*(1 + y*y/(z*z));
            J(1, 4) = +fy*x*y/(z*z);
            J(1, 5) = +fy*x/z;

            J *= -1.0;

            // update G-N:
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Vector6d dT = H.fullPivHouseholderQr().solve(b);

        // validity check:
        if (isnan(dT[0])) {
            cout << "[PnP G-N]: Result is nan at iteration " << iter << ". Aborted." << endl;
            break;
        }

        // early stopping:
        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "[PnP G-N]: Cost increased. current: " << cost << ", last: " << lastCost << ". Aborted." << endl;
            break;
        }

        cout << "\tIteration " << iter << " current cost: " << cout.precision(12) << cost << " last cost: " << cout.precision(12) << lastCost << endl;

        // update pose:
        T_esti = Sophus::SE3<double, Eigen::AutoAlign>::exp(dT) * T_esti;
        
        lastCost = cost;
    }

    cout << "[PnP G-N]: estimated pose: \n" << T_esti.matrix() << endl;

    return EXIT_SUCCESS;
}
