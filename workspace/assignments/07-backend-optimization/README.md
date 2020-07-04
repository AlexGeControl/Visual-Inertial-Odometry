# Visual Odometry: Backend Optimization --  

This is the solution of Assignment 07 of Visual SLAM: Theory and Practice from [深蓝学院](https://www.shenlanxueyuan.com/my/course/240).

深蓝学院视觉SLAM理论与实践第六节课习题答案. 版权归深蓝学院所有. 请勿抄袭.

---

### Up and Running

The solution has been tested inside Docker CPU/GPU.

```bash
# go to workspace:
cd /workspace/assignments/07-backend-optimization
# build:
mkdir build && cd build && cmake .. && make -j8
# execute the solution 
```

---

## Solutions

---

### 1. Bundle Adjustment
### 1. Bundle Adjustment

---

#### 1.a Research Review
#### 1.a 文

---

#### 1.b BAL-dataset
#### 1.b BAL-dataset

The solution is available at (click to follow the link) [here](01-bal-g2o/src/bal_g2o.cpp)

##### Analytic Jacobian Implementation

Below is the C++ implementation of **Analytic Jacobian for BAL PnP**

**First**, implement `linearizeOplus` of the binary edge as follows:

```c++
    // use analytic Jacobian:
    virtual void linearizeOplus() override {
        // get vertex handlers:
        const VertexPoseAndIntrinsics *vertex_camera = static_cast<VertexPoseAndIntrinsics *>(_vertices[0]);
        const VertexPoint *vertex_point = static_cast<VertexPoint *>(_vertices[1]);

        // get camera pose:
        // get camera intrinsics:
        double f = vertex_camera->estimate().focal;
        double k1 = vertex_camera->estimate().k1;
        double k2 = vertex_camera->estimate().k2;
        // get intermediate variables:
        Eigen::Vector3d X = vertex_point->estimate();
        Eigen::Vector3d P = vertex_camera->estimate().T.map(X);
        double P_X = P(0), P_Y = P(1), P_Z = P(2);

        double P_X_2 = P_X*P_X;
        double P_X_4 = P_X_2*P_X_2;

        double P_Y_2 = P_Y*P_Y;
        double P_Y_4 = P_Y_2*P_Y_2;

        double P_Z_2 = P_Z*P_Z;
        double P_Z_3 = P_Z*P_Z_2;
        double P_Z_4 = P_Z_2*P_Z_2;
        double P_Z_5 = P_Z*P_Z_4;
        double P_Z_6 = P_Z_3*P_Z_3;

        double R_2 = P_X_2 + P_Y_2;
        double R_4 = R_2*R_2;

        // Jacobian with respect to camera pose:
        Eigen::Matrix<double, 2, 3> JP;
        JP(0, 0) = -f*(2*P_X_2*(P_Z_2*k1 + 2*k2*R_2) + P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2))/P_Z_5;
        JP(0, 1) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*R_2)/P_Z_5;
        JP(0, 2) = P_X*f*(P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2) + 2*R_2*(P_Z_2*k1 + 2*k2*R_2))/P_Z_6;
        JP(1, 0) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*R_2)/P_Z_5;
        JP(1, 1) = -f*(2*P_Y_2*(P_Z_2*k1 + 2*k2*R_2) + P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2))/P_Z_5;
        JP(1, 2) = P_Y*f*(P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2) + 2*R_2*(P_Z_2*k1 + 2*k2*R_2))/P_Z_6;

        // Jacobian with respect to camera intrinsic:
        Eigen::Matrix<double, 2, 3> JI;
        JI(0, 0) = -P_X*(P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2))/P_Z_5;
        JI(0, 1) = -P_X*f*R_2/P_Z_3;
        JI(0, 2) = -P_X*f*R_2/P_Z_5;
        JI(1, 0) = -P_Y*(P_Z_4 + R_2*(P_Z_2*k1 + k2*R_2))/P_Z_5;
        JI(1, 1) = -P_Y*f*R_2/P_Z_3;
        JI(1, 2) = -P_Y*f*R_2/P_Z_5;

        if (!vertex_camera->fixed()) {
            // a. camera pose:
            _jacobianOplusXi.block<2, 3>(0, 0) = JP;
            _jacobianOplusXi.block<2, 3>(0, 3) = -JP * g2o::skew(P);
            // b. camera intrinsics:
            _jacobianOplusXi.block<2, 3>(0, 6) = JI;
        }

        // Jacobian for landmark:
        if (!vertex_point->fixed()) {
            _jacobianOplusXj = JP * vertex_camera->estimate().T.rotation().toRotationMatrix();
        }
    }
```

Besides, the method `oplusImpl` and `project` of `VertexPoseAndIntrinsics` also have to be changed as follows. Here `g2o::SE3Quat` is used to manipulate the camera pose:

```c++
    virtual void oplusImpl(const double *update) override {
        // update camera pose:
        Eigen::Matrix3d dR = SO3d::exp(
            Vector3d(update[3], update[4], update[5])
        ).matrix();
        Eigen::Vector3d dt(
            update[0], update[1], update[2]
        );

        _estimate.T = g2o::SE3Quat(dR, dt) * _estimate.T;
        // update camera intrinsics:
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    Vector2d project(const Vector3d &point) {
        // transform to camera frame:
        Eigen::Vector3d P = _estimate.T.map(point);
        // perspective division:
        Eigen::Vector2d p(
            -P(0)/P(2), -P(1)/P(2)
        );
        // distortion:
        double r_squared = p.squaredNorm();
        double distortion = 1.0 + r_squared * (_estimate.k1 + _estimate.k2 * r_squared);
        Eigen::Vector2d p_prime(
            _estimate.focal * distortion * p(0),
            _estimate.focal * distortion * p(1)
        );
        return p_prime;
    }
```

##### Up and Running

Compile, and execute the following commands to reproduce the results

```bash
# go to working directory:
cd /workspace/assignments/07-backend-optimization/
# build:
mkdir build && cd build && cmake .. && make -j4
# execute:
./bal_g2o /workspace/assignments/07-backend-optimization/data/01-bal/problem-16-22106-pre.txt
```

##### Results and Review

The two key sub matrices needed for analytic Jacobian are shown below. For further details please refer to [here](01-bal-g2o/README.md)

First comes the matrix needed for `camera pose` and `point position` Jacobian derivation

<img src="01-bal-g2o/doc/Jacobian--point-in-camera-frame.png" alt="Jacobian with respect to Intermediate Variables" width="100%">

Then comes the matrix needed for `camera intrinsic` Jacobian derivation

<img src="01-bal-g2o/doc/Jacobian--camera-intrinsics.png" alt="Jacobian with respect to Camera Intrinsics" width="100%">

---

The `log` of optimization using analytic Jacobian is as follows. We can see that compared with using default numerical Jacobian, **using the optimized analytic Jacobian can greatly accelerate the optimization process**.

```bash
# problem overview:
Header: 16 22106 83718
# optimization log:
iteration= 0	 chi2= 2132524.720570	 time= 7.32429	 cumTime= 7.32429	 edges= 83718	 schur= 1	 lambda= 888.503326	 levenbergIter= 1
iteration= 1	 chi2= 1220355.327504	 time= 7.24393	 cumTime= 14.5682	 edges= 83718	 schur= 1	 lambda= 296.167775	 levenbergIter= 1
iteration= 2	 chi2= 984382.567199	 time= 7.3394	 cumTime= 21.9076	 edges= 83718	 schur= 1	 lambda= 98.722592	 levenbergIter= 1
iteration= 3	 chi2= 681054.506120	 time= 7.35763	 cumTime= 29.2653	 edges= 83718	 schur= 1	 lambda= 32.907531	 levenbergIter= 1
iteration= 4	 chi2= 489824.754203	 time= 7.69444	 cumTime= 36.9597	 edges= 83718	 schur= 1	 lambda= 10.969177	 levenbergIter= 1
iteration= 5	 chi2= 403070.794266	 time= 11.6948	 cumTime= 48.6545	 edges= 83718	 schur= 1	 lambda= 29.251138	 levenbergIter= 3
iteration= 6	 chi2= 363009.118699	 time= 7.53188	 cumTime= 56.1864	 edges= 83718	 schur= 1	 lambda= 19.500759	 levenbergIter= 1
iteration= 7	 chi2= 314099.844975	 time= 11.8666	 cumTime= 68.0529	 edges= 83718	 schur= 1	 lambda= 52.002024	 levenbergIter= 3
iteration= 8	 chi2= 296637.546781	 time= 12.1796	 cumTime= 80.2326	 edges= 83718	 schur= 1	 lambda= 138.672063	 levenbergIter= 3
iteration= 9	 chi2= 290072.599275	 time= 7.65337	 cumTime= 87.8859	 edges= 83718	 schur= 1	 lambda= 92.448042	 levenbergIter= 1
iteration= 10	 chi2= 287766.552552	 time= 9.9005	 cumTime= 97.7864	 edges= 83718	 schur= 1	 lambda= 123.264056	 levenbergIter= 2
...
```

---

The view of `King's Landing, Dubrovnik-16` is shown below

<img src="01-bal-g2o/doc/kings-landing--final.png" alt="King's Landing, Dubrovnik, Analytic Jacobation 50 Iterations" width="100%">

---

### 2. Direct Method Using Bundle Adjustment
### 2. 直接法的 Bundle Adjustment


