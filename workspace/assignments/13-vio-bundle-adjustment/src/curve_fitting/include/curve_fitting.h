#ifndef CURVE_FITTING_H
#define CURVE_FITTING_H

using namespace myslam::backend;
using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}  // abc: 三个参数， Vertex 是 3 维的
    virtual std::string TypeInfo() const { return "abc"; }
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingExpEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingExpEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // 计算曲线模型误差
    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();  // 估计的参数
        residual_(0) = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_;  // 构建残差
    }

    // 计算残差对变量的雅克比
    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );

        Eigen::Matrix<double, 1, 3> jaco_abc;  // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        jaco_abc << x_ * x_ * exp_y, x_ * exp_y , 1 * exp_y;
        jacobians_[0] = jaco_abc;
    }
    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "CurveFittingExpEdge"; }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};

class CurveFittingPolyEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    CurveFittingPolyEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // 计算曲线模型误差
    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();  // 估计的参数
        residual_(0) = abc(0)*x_*x_ + abc(1)*x_ + abc(2) - y_;  // 构建残差
    }

    // 计算残差对变量的雅克比
    virtual void ComputeJacobians() override
    {
        Eigen::Matrix<double, 1, 3> jaco_abc;  // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        jaco_abc << x_*x_, x_, 1;
        jacobians_[0] = jaco_abc;
    }
    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "CurveFittingPolyEdge"; }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};

#endif // CURVE_FITTING_H