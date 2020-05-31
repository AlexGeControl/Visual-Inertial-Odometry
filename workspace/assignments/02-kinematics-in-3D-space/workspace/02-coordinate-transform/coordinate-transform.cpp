#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main () {
    // camera frame 1:
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
    Eigen::Matrix3d R1 = q1.toRotationMatrix(); 
    Eigen::Vector3d t1(0.7, 1.1, 0.2);

    // camera frame 2:
    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    Eigen::Matrix3d R2 = q2.toRotationMatrix(); 
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    // observation in camera frame 1:
    Eigen::Vector3d p1(0.5, -0.1, 0.2);

    // observation in camera frame 2:
    Eigen::Vector3d p2 = R2 * R1.transpose() * (p1 - t1) + t2;

    // display:
    cout << "Observation in camera frame 2 is: " << p2 << endl;
}