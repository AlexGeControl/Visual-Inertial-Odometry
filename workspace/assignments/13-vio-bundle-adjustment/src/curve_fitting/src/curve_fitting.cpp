#include <ros/ros.h>

#include <iostream>
#include <random>

#include "backend/problem.h"
#include "curve_fitting.h"

using namespace myslam::backend;
using namespace std;

struct Config {
    // curve:
    double a, b, c;
    // measurement:
    int N;
    double sigma;
    // problem:
    string type;
    // optimization:
    int strategy;
    int max_iterations;
};

int main(int argc, char** argv) {
    std::string node_name{"curve_fitting_node"};
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    // load params:
    Config config;
    // curve:
    nh.param("curve_fitting/curve/a", config.a, 1.0);
    nh.param("curve_fitting/curve/b", config.b, 2.0);
    nh.param("curve_fitting/curve/c", config.c, 1.0);
    // measurement:
    nh.param("curve_fitting/measurement/N", config.N, 5000);
    nh.param("curve_fitting/measurement/sigma", config.sigma, 1.0);
    // problem:
    nh.param("curve_fitting/problem", config.type, string("exp"));
    // optimization:
    nh.param("curve_fitting/optimization/lambda_scheduling", config.strategy, static_cast<int>(LambdaStrategy::NIELSEN));
    nh.param("curve_fitting/optimization/max_iterations", config.max_iterations, 50);

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, config.sigma);

    // 构建 problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr<CurveFittingVertex> vertex(new CurveFittingVertex());

    // 设定待估计参数 a, b, c初始值
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));
    // 将待估计的参数加入最小二乘问题
    problem.AddVertex(vertex);

    LambdaStrategy strategy = static_cast<LambdaStrategy>(config.strategy);

    // 构造 N 次观测
    for (int i = 0; i < config.N; ++i) {
        double x = i/(double)(config.N);
        double n = noise(generator);

        shared_ptr<Edge> edge;
        if ("exp" == config.type) {
            // 观测 y
            double y = std::exp(config.a*x*x + config.b*x + config.c) + n;

            // 每个观测对应的残差函数
            std::unique_ptr<Edge> problem_edge(new CurveFittingExpEdge(x,y));
            edge = std::move(problem_edge); 
        } else {
            // 观测 y
            double y = config.a*x*x + config.b*x + config.c + n;

            // 每个观测对应的残差函数
            std::unique_ptr<Edge> problem_edge(new CurveFittingPolyEdge(x,y));
            edge = std::move(problem_edge); 
        }

        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        // 把这个残差添加到最小二乘问题
        problem.AddEdge(edge);
    }

    /// 使用 LM 求解
    problem.Solve(config.max_iterations, strategy);

    ROS_WARN("-------Estimation:");
    ROS_WARN("\t %f %f %f", 
        vertex->Parameters()(0),
        vertex->Parameters()(1),
        vertex->Parameters()(2)
    );
    ROS_WARN("-------Ground Truth:");
    ROS_WARN("\t %f %f %f", 
        config.a,
        config.b,
        config.c
    );

    // finally
    return EXIT_SUCCESS;
}