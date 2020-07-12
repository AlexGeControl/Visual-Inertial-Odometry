#include <iostream>
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/so3.hpp>

class RandomAngle {
public:
    RandomAngle(double lower, double upper) : unif_(lower, upper){}
    double Get(void) { return unif_(re_); }
private:
    std::uniform_real_distribution<double> unif_;
    std::default_random_engine re_;
};

int main(int argc, char *argv[]) {
    // generate random rotation matrix:
    RandomAngle random_angle(-M_PI, +M_PI);
    Eigen::AngleAxisd angle_axis(
        // angle:
        random_angle.Get(),
        // axis: 
        Eigen::Vector3d::Random().normalized()
    );

    // format as quaternion:
    Eigen::Quaterniond q(angle_axis);
    // format as SO3:
    Sophus::SO3d phi(q);

    // define update:
    const Eigen::Vector3d dW(0.01, 0.02, 0.03);

    // apply update, quaternion:
    q = q * Eigen::Quaterniond(1.0, 0.5*dW.x(), 0.5*dW.y(), 0.5*dW.z());
    Sophus::SO3d so3_quaternion(q);
    // apply update, SO3:
    Sophus::SO3d so3_rotation_matrix = phi * Sophus::SO3d::exp(dW);

    // error:
    Eigen::Vector3d error = so3_rotation_matrix.log() - so3_quaternion.log();
    double relative_error = 100.0 * error.norm() / phi.log().norm();

    // generate output:
    std::cout.precision(2);
    std::cout << "relative error in SO3: " << std::fixed << relative_error << "%" << std::endl;

    return EXIT_SUCCESS;
}