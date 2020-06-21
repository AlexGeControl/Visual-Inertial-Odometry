//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.020361855052347700, -0.40071100381184450, -0.03324074249824097,
         +0.393927077821636900, -0.03506401846698079, +0.58571103037210150,
         -0.006788487241438284, -0.58154342729156860, -0.01438258684486258;

    // SVD:
    BDCSVD<Matrix3d> svd(E, Eigen::ComputeFullU|Eigen::ComputeFullV);
    const MatrixXd &U = svd.matrixU();
    const MatrixXd &V = svd.matrixV();
    double sigma = (svd.singularValues()(0) + svd.singularValues()(1)) / 2.0;
    DiagonalMatrix<double, 3> S(sigma, sigma, 0.0);

    // rotation matrices:
    Vector3d axis{0.0, 0.0, 1.0};
    auto R_z_pos = AngleAxisd(+M_PI/2, axis).toRotationMatrix();
    auto R_z_neg = AngleAxisd(-M_PI/2, axis).toRotationMatrix();

    // set t1, t2, R1, R2 
    Matrix3d t_wedge1 = U * R_z_pos * S * U.transpose();
    Matrix3d t_wedge2 = U * R_z_neg * S * U.transpose();

    Matrix3d R1 = U * R_z_pos.transpose() * V.transpose();
    Matrix3d R2 = U * R_z_neg.transpose() * V.transpose();

    // show results:
    cout << "[Essential Matrix to Relative Pose]:" << endl;
    cout << "\tR1 = \n" << R1 << endl;
    cout << "\tt1 = \n" << +1.0 * Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "\tR2 = \n" << R1 << endl;
    cout << "\tt2 = \n" << -1.0 * Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "\tR3 = \n" << R2 << endl;
    cout << "\tt3 = \n" << +1.0 * Sophus::SO3d::vee(t_wedge2) << endl;
    cout << "\tR4 = \n" << R2 << endl;
    cout << "\tt4 = \n" << -1.0 * Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "\t Reconstructed, from t^R =\n" << tR << endl;

    // compute relative error:
    double relative_error = 100.0 * (tR - E).norm() / E.norm();
    cout << "\t Relative error, F-norm: " << relative_error << endl;

    return 0;
}