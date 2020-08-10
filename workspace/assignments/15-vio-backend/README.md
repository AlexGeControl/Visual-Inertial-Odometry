# Visual Inertial Odometry: Backend Solver -- 从零开始手写VIO: Backend Solver

This is the solution of Assignment 05 of Hands on VIO from [深蓝学院](https://www.shenlanxueyuan.com/course/247).

深蓝学院从零开始手写VIO第5节Backend Solver答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Solutions

---

### 1. Implement Backend Solver
### 1. 实现Backend Solver Problem.cc中的如下函数

#### Results

**First** comes the result of `camera pose & landmark position optimization`

```bash
Compare MonoBA results after optimization...
--------------- landmark position --------------------
landmark 1:
	ground truth :0.2209	with noise :0.2271	opt 0.2210
landmark 2:
	ground truth :0.2343	with noise :0.3144	opt 0.2343
landmark 3:
	ground truth :0.1423	with noise :0.1297	opt 0.1424
landmark 4:
	ground truth :0.2143	with noise :0.2785	opt 0.2145
landmark 5:
	ground truth :0.1306	with noise :0.1301	opt 0.1306
landmark 6:
	ground truth :0.1914	with noise :0.1675	opt 0.1915
landmark 7:
	ground truth :0.1668	with noise :0.1659	opt 0.1669
landmark 8:
	ground truth :0.2016	with noise :0.2256	opt 0.2019
landmark 9:
	ground truth :0.1680	with noise :0.1558	opt 0.1680
landmark 10:
	ground truth :0.2189	with noise :0.2097	opt 0.2188
landmark 11:
	ground truth :0.2057	with noise :0.1432	opt 0.2056
landmark 12:
	ground truth :0.1279	with noise :0.1221	opt 0.1278
landmark 13:
	ground truth :0.1679	with noise :0.1433	opt 0.1679
landmark 14:
	ground truth :0.2167	with noise :0.1853	opt 0.2169
landmark 15:
	ground truth :0.1800	with noise :0.1842	opt 0.1800
landmark 16:
	ground truth :0.2269	with noise :0.2457	opt 0.2271
landmark 17:
	ground truth :0.1574	with noise :0.1765	opt 0.1576
landmark 18:
	ground truth :0.1825	with noise :0.1473	opt 0.1823
landmark 19:
	ground truth :0.1557	with noise :0.1823	opt 0.1557
landmark 20:
	ground truth :0.1465	with noise :0.2406	opt 0.1466
------------ camera pose, translation ----------------
camera 1:
	ground truth: 0.0000 0.0000 0.0000
	optimized: 0.0000 0.0000 0.0000
camera 2:
	ground truth: -1.0718  4.0000  0.8660
	optimized: -1.0718  4.0000  0.8660
camera 3:
	ground truth: -4.0000  6.9282  0.8660
	optimized: -4.0000  6.9282  0.8660
```

It can bee seen from the output that **the estimation has converged to ground truth**, which proves the correctness of backend solver implementation.

**Next** comes the result of `marginalization test`:

```bash
---------- TEST Marg: before marg------------
     100.0000 -100.0000    0.0000
    -100.0000  136.1111  -11.1111
       0.0000  -11.1111   11.1111
---------- TEST Marg: target variable has been moved to bottom right------------
     100.0000    0.0000 -100.0000
       0.0000   11.1111  -11.1111
    -100.0000  -11.1111  136.1111
---------- TEST Marg: after marg------------
    26.5306 -8.1633
    -8.1633 10.2041
```

---

#### a. Problem::MakeHessian()
#### a. Problem::MakeHessian()

The full source code can be found [here](backend/problem.cc). Below is the code snippet for core computing logic:

```c++
    for (auto &edge: edges_) {
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            if (v_i->IsFixed()) continue;    // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            MatXX JtW = jacobian_i.transpose() * edge.second->Information();
            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;

                // update Hessian matrix:
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;

                // also update the diagonal symmetric block:
                if (j != i) {
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            b.segment(index_i, dim_i).noalias() -= JtW * edge.second->Residual();
        }
    }
```

#### b. Problem::SolveLinearSystem()
#### b. Problem::SolveLinearSystem()

The full source code can be found [here](backend/problem.cc). Below is the code snippet for core computing logic:

```c++
    // SLAM 问题采用舒尔补的计算方式
    // step1: schur marginalization --> Hpp, bpp
    int reserve_size = ordering_poses_;
    int marg_size = ordering_landmarks_;

    // get sub matrix handlers:
    MatXX Hmm = Hessian_.block(
        reserve_size, reserve_size, 
        marg_size, marg_size
    );
    MatXX Hpm = Hessian_.block(
        0, reserve_size, 
        reserve_size, marg_size
    );
    MatXX Hmp = Hessian_.block(
        reserve_size, 0, 
        marg_size, reserve_size
    );

    VecX bpp = b_.segment(
        0,
        reserve_size
    );
    VecX bmm = b_.segment(
        reserve_size,
        marg_size
    );

    // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
    MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
    for (auto landmarkVertex : idx_landmark_vertices_) {
        int idx = landmarkVertex.second->OrderingId() - reserve_size;
        int size = landmarkVertex.second->LocalDimension();
        Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
    }

    // step2: solve camera poses:
    MatXX tempH = Hpm * Hmm_inv;
    H_pp_schur_ = Hessian_.block(
        0, 0,
        reserve_size, reserve_size
    ) - tempH * Hmp;
    b_pp_schur_ = bpp - tempH * bmm;

    for (ulong i = 0; i < ordering_poses_; ++i) {
        H_pp_schur_(i, i) += currentLambda_;
    }

    // use PCG solver:
    size_t num_iter_camera = H_pp_schur_.rows() << 1;
    VecX delta_x_pp(VecX::Zero(reserve_size));
    delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, num_iter_camera);
    delta_x_.head(reserve_size) = delta_x_pp;

    // step 3: solve landmarks
    H_ll_ = Hmm;
    b_ll_ = bmm - Hmp*delta_x_pp;

    // leverage the structure:
    VecX delta_x_ll(marg_size);
    for (auto landmarkVertex : idx_landmark_vertices_) {
        int idx = landmarkVertex.second->OrderingId() - reserve_size;
        int size = landmarkVertex.second->LocalDimension();

        delta_x_ll.segment(idx,size) = H_ll_.block(idx, idx, size, size).inverse() * b_ll_.segment(idx,size);
    }
    delta_x_.tail(marg_size) = delta_x_ll;
```

#### c. Problem::TestMarginalize()
#### c. Problem::TestMarginalize()

```c++
    // variable to be marginalized:
    int idx = 1;
    // dimensions of to-be-marginalized variable:
    const size_t D = 1;            
    const size_t N = 3;
    const size_t M = N - D;
    double delta1 = 0.1 * 0.1;
    double delta2 = 0.2 * 0.2;
    double delta3 = 0.3 * 0.3;

    MatXX H_marg(MatXX::Zero(N, N));
    H_marg << 
         1./delta1,                         -1./delta1,          0,
        -1./delta1,  1./delta1 + 1./delta2 + 1./delta3, -1./delta3,
               0.0,                         -1./delta3,   1/delta3;
    std::cout << "---------- TEST Marg: before marg------------"<< std::endl;
    std::cout << H_marg << std::endl;

    // step 1: move row i to bottom:
    Eigen::MatrixXd src_row = H_marg.block(
        idx, 0, 
        D, N
    );
    Eigen::MatrixXd dst_row = H_marg.block(
        N - D, 0, 
        D, N
    );
    H_marg.block(
        N - D, 0,
        D, N
    ) = src_row;
    H_marg.block(
        idx, 0,
        D, N
    ) = dst_row;

    // step 2: move col i to right
    Eigen::MatrixXd src_col = H_marg.block(
        0, idx, 
        N, D
    );
    Eigen::MatrixXd dst_col = H_marg.block(
        0, N - D, 
        N, D
    );
    H_marg.block(
        0, N - D, 
        N, D
    ) = src_col;
    H_marg.block(
        0, idx, 
        N, D
    ) = dst_col;

    std::cout << "---------- TEST Marg: target variable has been moved to bottom right------------"<< std::endl;
    std::cout<< H_marg <<std::endl;

    /// start marginalization： compute schur complement
    double eps = 1e-8;
    Eigen::MatrixXd Amm = 0.5 * (H_marg.block(M, M, D, D) + H_marg.block(M, M, D, D).transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd(
            (saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                              saes.eigenvectors().transpose();

    // get sub matrix handlers:
    Eigen::MatrixXd Arm = H_marg.block(
        0, M, 
        M, D
    );
    Eigen::MatrixXd Amr = H_marg.block(
        M, 0,
        D, M
    );
    Eigen::MatrixXd Arr = H_marg.block(
        0, 0,
        M, M
    );

    Eigen::MatrixXd tempB = Arm * Amm_inv;
    Eigen::MatrixXd H_prior = Arr - tempB * Amr;

    std::cout << "---------- TEST Marg: after marg------------"<< std::endl;
    std::cout << H_prior << std::endl;
```

---

### 2. Summarize the Gauge-Free Handling in Optimization for VIO
### 2. 请总结[论文](doc/on-the-comparison-of-gauge-free-handling-of-vio-optimization.pdf)优化过程中处理H自由度的不同操作方式。内容包括: 具体处理方式,实验效果,结论

#### Gauge-Free Handling Strategy
#### 处理方式

Three possible gauge-free handlings are proposed in the paper, namely

* `Gauge Fixation` 
    * Fix the position and yaw angle of the first camera for VIO 
    * Fix the poses of the first and the second camera for Mono Visual Odometry
* `Gauge Prior`
    * Add a prior to the initial pose of the first camera for VIO
    * Add a prior to both the first and the second camera poses for Mono Visual Odometry
* `Gauge Free`
    * Let the full parameter vector evolve freely during the optimization

#### Results
#### 实验效果

##### a. Accuracy
##### a. 准确度

The three strategies have similar estimation error on both `simulated` and `real-world` datasets. 

##### b. Computational Effort
##### b. 计算复杂度

* The `free gauge` approach is slightly faster than the other two because it takes fewer iterations and less time to converge.
* The `gauge fixation` approach has the least time per iteration due to the smaller number of variables in optimization

#### Conclusions
#### 结论

* `Accuracy`
    * The three approaches have almost the same accuracy.
* `Computational Effor`
    * The `free gauge` approach is slightly faster than the others because it takes fewer iterations to converge.
* `Gauge Prior v.s. Gauge Fixation`
    * With a proper weight, the `gauge prior` approach has almost the same performance as `gauge fixation` approach.
    * Proper weight is needed to avoid extra computational cost.

---

### 3. Implement Gauge Prior Strategy and Compare the Effect of Different Weights
### 3. 在代码中给第一帧和第二帧添加prior约束, 并比较为prior设定不同权重时, BA求解收敛精度和速度

#### Results

Below is the result of `accuracy` and `computational effort` under different prior weight of `information matrix`

| Prior Weight | Problem Solve Cost / ms | MakeHessian Cost / ms | RMSE, Landmark | RMSE, Camera Position |
|:------------:|:-----------------------:|:---------------------:|:--------------:|:---------------------:|
|    6.25e-2   |         101.853         |         82.496        |     0.0021     |         0.1030        |
|    1.25e-1   |         109.529         |        88.4697        |     0.0022     |         0.1015        |
|    2.5e-1    |         99.9498         |        81.4395        |     0.0022     |         0.0991        |
|     5e-1     |         95.3707         |        77.9149        |     0.0023     |         0.0967        |
|       1      |         103.553         |        84.4058        |     0.0025     |         0.0966        |
|       5      |         132.2900        |        108.5080       |     0.0027     |         0.1147        |
|      10      |         123.7580        |        101.4130       |     0.0027     |         0.1305        |
|      25      |         115.2990        |        94.9124        |     0.0031     |         0.1629        |
|      50      |         69.8729         |        57.5055        |     0.0048     |         0.2004        |

The experimental results are summarized as follows:

* The prior weight has a significant impact on `computing speed`. The larger the prior weight of information matrix, the shorter the time that is needed for optimization to converge. This agrees very well with the recommanded paper.

* However, the actual optimization time is also sensitive to the solver config. Under current problem setup it attains the best estimation accuracy when `prior weight equals 1.0`

#### Implementation

The full source code can be found [here](app/TestMonoBA.cpp). Below is the code snippet for core computing 

```c++
    // add poses:
    MatXX H(MatXX::Zero(6, 6));
    for (size_t i = 0; i < 6; ++i) {
        H(i, i) = PRIOR_WEIGHT;
    }

    vector<shared_ptr<VertexPose> > vertexCams_vec;
    for (size_t i = 0; i < cameras.size(); ++i) {
        shared_ptr<VertexPose> vertexCam(new VertexPose());
        Eigen::VectorXd pose(7);
        pose << cameras[i].twc, cameras[i].qwc.x(), cameras[i].qwc.y(), cameras[i].qwc.z(), cameras[i].qwc.w();
        vertexCam->SetParameters(pose);

        problem.AddVertex(vertexCam);
        vertexCams_vec.push_back(vertexCam);

        if(i < 2) {
            // strategy 2: gauge prior
            shared_ptr<EdgeSE3Prior> edge(
                new EdgeSE3Prior(
                    cameras_gt[i].twc, cameras_gt[i].qwc
                )
            );

            std::vector<std::shared_ptr<Vertex>> edge_vertex;
            edge_vertex.push_back(vertexCams_vec[i]);
            edge->SetVertex(edge_vertex);
            
            // set information matrix:
            edge->SetInformation(H);

            problem.AddEdge(edge);
        }
    }
```

Here the prior distribution is set using `ground truth camera pose`.