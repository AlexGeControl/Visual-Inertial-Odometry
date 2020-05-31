#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main()
{
    const int N = 100;

    // random Hermitian, semi-definite matrix A:
    MatrixXf A = MatrixXf::Random(N, N);
    A = A * A.transpose();
    
    // random vector b:
    VectorXf b = VectorXf::Random(N);
    double b_norm = b.norm();

    // LU decomposition:
    VectorXf x_lu = A.fullPivLu().solve(b);
    double e_lu = 100.0 * (A*x_lu - b).norm() / b_norm;
    // QR decomposition:
    VectorXf x_qr = A.householderQr().solve(b);
    double e_qr = 100.0 * (A*x_qr - b).norm() / b_norm;
    // Cholesky decomposition:
    VectorXf x_cholesky = A.ldlt().solve(b);
    double e_cholesky = 100.0 * (A*x_cholesky - b).norm() / b_norm;

    // solution summary:
    cout << "Relative error of LU: " << e_lu << endl;
    cout << "Relative error of QR: " << e_qr << endl;
    cout << "Relative error of Cholesky: " << e_cholesky << endl;
}