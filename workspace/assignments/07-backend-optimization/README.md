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

### 1. Bundle Adjustment
### 1. Bundle Adjustment

#### 1.b BAL-dataset
#### 1.b BAL-dataset

The solution is available at (click to follow the link) [here](01-bal-g2o/src/bal_g2o.cpp)

#### Analytic Jacobian Implementation

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

Besides, the `oplusImpl` of `VertexPoseAndIntrinsics` also has to be changed as:

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
```

Below is the implementation of **Coarse-to-Fine Estimation using Image Pyramid**

```c++
void OpticalFlowMultiLevel(
        const Mat &image_target,
        const Mat &image_source,
        const vector<KeyPoint> &keypoints_target,
        vector<KeyPoint> &keypoints_source,
        vector<bool> &success,
        bool inverse
) {
    // parameters
    int NUM_PYRAMIDS = 4;
    double PYRAMID_SCALE = 0.5;

    // get input config:
    const size_t N = keypoints_target.size();
    bool has_initial = (keypoints_source.size() != 0);

    // create pyramids
    vector<Mat> pyramid_target, pyramid_source;
    vector<cv::KeyPoint> keypoints_target_;

    pyramid_target.push_back(image_target);
    pyramid_source.push_back(image_source);

    for (int i = 0; i < NUM_PYRAMIDS - 1; i++) {
        const cv::Mat &image_target_ = pyramid_target.at(i);
        const cv::Mat &image_source_ = pyramid_source.at(i);

        cv::Mat image_target_downsampled;
        cv::Mat image_source_downsampled;

        cv::pyrDown(image_target_, image_target_downsampled, cv::Size(PYRAMID_SCALE*image_target_.cols, PYRAMID_SCALE*image_target_.rows));
        cv::pyrDown(image_source_, image_source_downsampled, cv::Size(PYRAMID_SCALE*image_source_.cols, PYRAMID_SCALE*image_source_.rows));

        pyramid_target.push_back(image_target_downsampled);
        pyramid_source.push_back(image_source_downsampled);
    }

    // perform coarse-to-fine estimation:
    for (int i = NUM_PYRAMIDS - 1; 0 <= i; --i) {
        // compute current scale:
        double scale = pow(PYRAMID_SCALE, i);
        
        const cv::Mat &image_target_ = pyramid_target.at(i);
        const cv::Mat &image_source_ = pyramid_source.at(i);

        // scale keypoints:
        for (size_t j = 0; j < N; ++j) {
            cv::KeyPoint keypoint_target_ = keypoints_target.at(j);

            keypoint_target_.pt.x *= scale;
            keypoint_target_.pt.y *= scale;

            keypoints_target_.push_back(keypoint_target_);
        }

        // optical flow estimation:
        OpticalFlowSingleLevel(
            image_target_, image_source_,
            keypoints_target_, keypoints_source,
            success, inverse
        );

        // already reach the original scale, terminate:
        if (i == 0) { break; }
        
        // clear keypoint target:
        keypoints_target_.clear();

        // scale back for next iteration:
        for (size_t j = 0; j < N; ++j) {
            keypoints_source.at(j).pt.x /= PYRAMID_SCALE;
            keypoints_source.at(j).pt.y /= PYRAMID_SCALE;
        }

        // clear indicators:
        success.clear();
    }
}
```

#### Results and Review

The results from the implemented `single-level` estimators are as follows:

Single-Level Forward       |Single-Level Inverse
:-------------------------:|:-------------------------:
![Single-Level, Forward](doc/image/01-optical-flow/single-level-forward.png)  |  ![Single-Level, Inverse](doc/image/01-optical-flow/single-level-inverse.png)

The results from the implemented `multi-level` estimators are as follows:

Multi-Level Forward        |Multi-Level Inverse
:-------------------------:|:-------------------------:
![Single-Level, Forward](doc/image/01-optical-flow/multi-level-forward.png)  |  ![Single-Level, Inverse](doc/image/01-optical-flow/multi-level-inverse.png)

The result from OpenCV is shown below:

<img src="doc/image/01-optical-flow/opencv.png" alt="Multi-Level Optical Flow from OpenCV" width="100%">

#### 1.a Overview
#### 1.a 文献综述

1. 按此文的分类,光流法可分为哪几类?

    * Forward Additive
    * Forward Compositional
    * Inverse Compositional
    * Inverse Additive

2. 在 compositional 中,为什么有时候需要做原始图像的 warp?该 warp 有何物理意义?

    Warp是线性变换的参数化表达。Warp可以用来表达各种各样的线性变换， 比如光流法中的Translation， 等。

3. forward 和 inverse 有何差别?

    `forward`是算法的直接实现，简单易懂。`inverse`是对直接实现的优化。 后者利用特征点的梯度生成Approximated Hessian Matrix, 且矩阵在迭代过程中不发生变化，相比直接法计算量更小，估计质量更高。

#### 1. b Forward Additive using Gaussian-Newton
#### 1. b 直接法

1. 从最小二乘角度来看,每个像素的误差怎么定义?
    
    See the code snippet below.

    ```c++
    double error = T.GetPixelValue(x0 + du, y0 + dv) - I.GetPixelValue(x + du, y + dv);
    ```

2. 误差相对于自变量的导数如何定义?

    See the code snippet below.

    ```c++
    // compute Jacobian:
    if (!inverse) {
        J = I.GetGradient(x + du, y + dv);
        H += J * J.transpose();
    } else {
        J = T.GetGradient(x0 + du, y0 + dv);
    }
    ```

#### 1. d Multi-Level through Image Pyramid
#### 1. d 推广至金字塔

1. 所谓 coarse-to-fine 是指怎样的过程?

    首先在下采样的图片上, 进行光流估计; 然后以该估计为初值, 在上一层级的图片上继续进行光流估计.

2. 光流法中的金字塔用途和特征点法中的金字塔有何差别?

    光流法的金字塔, 旨在在系统快速运动时，为光流法估计提供较好的初始值。特征点法中的金字塔，旨在生成Hierarchical Feature Description.

---

### 2. Direct Method
### 2. 直接法

The solution is available at (click to follow the link) [here](02-direct-method/direct_method.cpp)

#### Implementation

Below is the C++ implementation of **Single-Layer Direct Method**

```c++
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const Camera &camera,
    const vector<Eigen::Vector3d> &keypoints,
    Sophus::SE3d &T21,
    int MAX_ITERATIONS 
) {
    // parameters
    int HALF_PATCH_SIZE = 4;

    // initialize image handlers:
    ImageWithGradient T(img1);
    ImageWithGradient I(img2);

    double cost = 0, lastCost = 0;
    // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        goodProjection.clear();

        auto R = T21.rotationMatrix();
        auto t = T21.translation();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < keypoints.size(); i++) {
            const Eigen::Vector3d &P_target = keypoints.at(i);

            // project to target frame:
            Eigen::Vector2d p_target = camera.ToPixelFrame(P_target);

            // project to source frame:
            Eigen::Vector3d P_source = R*P_target + t;
            Eigen::Vector2d p_source = camera.ToPixelFrame(P_source);

            // check validity:
            if (I.IsValidPatch(p_source, HALF_PATCH_SIZE)) {
                goodProjection.push_back(p_source);
            } else {
                continue;
            }

            // Jacobian of pixel coordinates with respect to se3:
            Matrix26d J_pixel_xi = camera.GetJacobian(P_source);

            // and compute error and jacobian
            for (int x = -HALF_PATCH_SIZE; x < HALF_PATCH_SIZE; x++) {
                for (int y = -HALF_PATCH_SIZE; y < HALF_PATCH_SIZE; y++) {
                    Eigen::Vector2d p_target_ = Eigen::Vector2d(p_target.x() + x, p_target.y() + y);
                    Eigen::Vector2d p_source_ = Eigen::Vector2d(p_source.x() + x, p_source.y() + y);

                    double error = T.GetPixelValue(p_target_) - I.GetPixelValue(p_source_);

                    // image Jacobian:
                    Eigen::Vector2d J_img_pixel = I.GetGradient(p_source_);    

                    // total jacobian
                    Vector6d J = J_img_pixel.transpose() * J_pixel_xi;

                    // update H, b and cost:
                    H += J * J.transpose();
                    b += error * J;

                    cost += 0.5 * error*error;
                }
            }
        }

        const size_t N = goodProjection.size();

        // average over all good projections:
        cost /= N;

        // solve update and put it into estimation
        Vector6d update = H.fullPivHouseholderQr().solve(b);
        T21 = Sophus::SE3d::exp(update) * T21;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost >= lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        cout << "cost = " << cost << ", good = " << goodProjection.size() << " at " << iter << endl;
    }
    cout << "good projection: " << goodProjection.size() << endl;
    cout << "T21 = \n" << T21.matrix() << endl;
}
```

Below is the implementation of **Coarse-to-Fine Estimation using Direct Method**

```c++
void DirectPoseEstimationMultiLayer(
    const cv::Mat &image_target,
    const cv::Mat &image_source,
    Camera &camera,
    std::vector<Eigen::Vector3d> &keypoints,
    Sophus::SE3d &T21,
    int MAX_ITERATIONS 
) {
    // parameters
    int NUM_PYRAMIDS = 4;
    double PYRAMID_SCALE = 0.5;

    // create image pyramids
    vector<cv::Mat> pyramid_target, pyramid_source;

    pyramid_target.push_back(image_target);
    pyramid_source.push_back(image_source);

    for (int level = 0; level < NUM_PYRAMIDS; ++level) {
        const cv::Mat &image_target_ = pyramid_target.at(level);
        const cv::Mat &image_source_ = pyramid_source.at(level);

        cv::Mat image_target_downsampled;
        cv::Mat image_source_downsampled;

        cv::pyrDown(
            image_target_, image_target_downsampled, 
            cv::Size(PYRAMID_SCALE*image_target_.cols, PYRAMID_SCALE*image_target_.rows)
        );
        cv::pyrDown(
            image_source_, image_source_downsampled, 
            cv::Size(PYRAMID_SCALE*image_source_.cols, PYRAMID_SCALE*image_source_.rows)
        );

        pyramid_target.push_back(image_target_downsampled);
        pyramid_source.push_back(image_source_downsampled);
    }

    for (int level = NUM_PYRAMIDS - 1; level >= 0; --level) {
        double scale = pow(PYRAMID_SCALE, level);

        cv::Mat &image_target_ = pyramid_target.at(level);
        cv::Mat &image_source_ = pyramid_source.at(level);

        Camera camera_(
            scale*camera.GetFx(), scale*camera.GetFy(), scale*camera.GetCx(), scale*camera.GetCy()
        );

        DirectPoseEstimationSingleLayer(image_target_, image_source_, camera_, keypoints, T21, MAX_ITERATIONS);
    }
}
```

#### Results and Review

First comes the estimated trajectory from `Single-Layer` method:

```bash
# observation 01:
Image ../data/02-direct-method/000001.png, Pose = 
   0.999991  0.00240945  0.00337804 -0.00204298
-0.00241685    0.999995  0.00218773  0.00274849
-0.00337275 -0.00219587    0.999992   -0.724754
          0           0           0           1
# observation 02:
Image ../data/02-direct-method/000002.png, Pose = 
   0.999973  0.00136864  0.00728795  0.00754102
-0.00139701    0.999991  0.00388917 -0.00128753
-0.00728257 -0.00389925    0.999966    -1.47003
          0           0           0           1
# observation 03:
Image ../data/02-direct-method/000003.png, Pose = 
    0.999909  0.000449684    0.0134511    -0.235722
-0.000522703     0.999985   0.00542543 -0.000589808
  -0.0134484  -0.00543197     0.999895     -1.86832
           0            0            0            1
# observation 04:
Image ../data/02-direct-method/000004.png, Pose = 
   0.999857  0.00262317   0.0167018   -0.294455
-0.00271008    0.999983    0.005183   0.0214327
 -0.0166879 -0.00522753    0.999847    -2.02179
          0           0           0           1
# observation 05:
Image ../data/02-direct-method/000005.png, Pose = 
   0.999734  0.00152337   0.0230244    -0.40915
-0.00162931    0.999988  0.00458283   0.0625754
 -0.0230172 -0.00461912    0.999724    -2.97014
          0           0           0           1
```

Below is the estimated trajectory from `Multi-Layer` method:

```bash
# observation 01:
Image ../data/02-direct-method/000001.png, Pose = 
   0.999991  0.00240945  0.00337804 -0.00204298
-0.00241685    0.999995  0.00218773  0.00274849
-0.00337275 -0.00219587    0.999992   -0.724754
          0           0           0           1
# observation 02:
Image ../data/02-direct-method/000002.png, Pose = 
   0.999973  0.00136864  0.00728795  0.00754094
-0.00139701    0.999991  0.00388917 -0.00128751
-0.00728257 -0.00389925    0.999966    -1.47003
          0           0           0           1
# observation 03:
Image ../data/02-direct-method/000003.png, Pose = 
   0.999937  0.00163339   0.0110926  0.00840143
-0.00168998    0.999986  0.00509423  0.00347674
 -0.0110841 -0.00511266    0.999925    -2.20921
          0           0           0           1
# observation 04:
Image ../data/02-direct-method/000004.png, Pose = 
    0.999874  0.000355519    0.0158756   0.00882546
-0.000446735     0.999983   0.00574246   0.00279928
  -0.0158733  -0.00574883     0.999857     -2.99584
           0            0            0            1
# observation 05:
Image ../data/02-direct-method/000005.png, Pose = 
   0.999803  0.00120434   0.0198218   0.0189633
-0.00133428    0.999978  0.00654327  -0.0105501
 -0.0198134 -0.00656842    0.999782    -3.79388
          0           0           0           1
```

The results of keypoints tracking from the two methods are shown below:

Single-Level               |Multi-Level 
:-------------------------:|:-------------------------:
![Single-Level, Obs. 01](doc/image/02-direct-method/single-layer/000001.png)  |  ![Multi-Level, Obs. 01](doc/image/02-direct-method/multi-layer/000001.png)
![Single-Level, Obs. 02](doc/image/02-direct-method/single-layer/000002.png)  |  ![Multi-Level, Obs. 02](doc/image/02-direct-method/multi-layer/000002.png)
![Single-Level, Obs. 03](doc/image/02-direct-method/single-layer/000003.png)  |  ![Multi-Level, Obs. 03](doc/image/02-direct-method/multi-layer/000003.png)
![Single-Level, Obs. 04](doc/image/02-direct-method/single-layer/000004.png)  |  ![Multi-Level, Obs. 04](doc/image/02-direct-method/multi-layer/000004.png)
![Single-Level, Obs. 05](doc/image/02-direct-method/single-layer/000005.png)  |  ![Multi-Level, Obs. 05](doc/image/02-direct-method/multi-layer/000005.png)

#### 2.a 直接法是否可以类似光流,提出 inverse, compositional 的概念?它们有意义吗?

    I think the concept of inverse is meaningful. Based on the algorithm assumption, the image gradient from target image patch can be used to approximate that from source image. This can accelerate the computing since the approximated Hessian will not change during iteration.

#### 2.b 请思考上面算法哪些地方可以缓存或加速?

    For the bi-linear interpolated pixel value and image gradient, they can be cached to accelerate the computing speed.

#### 2.c 在上述过程中,我们实际假设了哪两个 patch 不变?

    The algorithm assumes the patches around the original keypoint in target image and the projected keypoint in source image are the same.

#### 2.d 为何可以随机取点?而不用取角点或线上的点?那些不是角点的地方,投影算对了吗?

    Because the algorithm assumes the two patches are the same so it doesn't need any information from feature point. However, non-corner points can not guide the optimization since its gradient will be zero.

#### 2.e 请总结直接法相对于特征点法的异同与优缺点。

    Pros
        * No need for feature point detection.
        * Can work on region with no strong feature points.
        * Can be extended to incorporate the merits of feature-matching based methods
    
    Cons
        * Algorithm assumption on patch intensity is too strong.
---

### 3. Disparity Map through Optical Flow
### 3. 用光流法计算视差

The solution is available at (click to follow the link) [here](03-disparity-map/disparity_map.cpp)

#### Implementation

The core idea is: **for each good estimation, extract its estimated horizontal disparity and corresponding value from disparity map then evaluate Pearson correlation coefficients on it**.

```c++
    // evaluate correlation 
    CorrCoeff evaluator;
    ImageWithGradient D(disparity);

    // plot the differences of those functions
    Mat img2_multi_forward;
    cv::cvtColor(img2, img2_multi_forward, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi_forward.size(); i++) {
        if (success_multi_forward[i]) {
            cv::circle(img2_multi_forward, kp2_multi_forward[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi_forward, kp1[i].pt, kp2_multi_forward[i].pt, cv::Scalar(0, 250, 0));
            evaluator.Add(
                // estimated disparity:
                kp1.at(i).pt.x - kp2_multi_forward.at(i).pt.x,
                // measurement from disparity map:
                D.GetPixelValue(kp1.at(i).pt.x, kp1.at(i).pt.y)
            );
        }
    }
    cout << "[Correlation Coefficient, Multi-Level Forward]: " << evaluator.GetCoeff() << endl;
```

#### Results

The Pearson Correlation Coefficients for the three methods (Multi-Level Forward, Multi-Level Inverse and OpenCV) are as follows:

```bash
[Correlation Coefficient, Multi-Level Forward]: 0.741482
[Correlation Coefficient, Multi-Level Inverse]: 0.730189
[Correlation Coefficient, OpenCV]: 0.79446
```

The estimated disparities are shown below. **From the image it can be seen that the farther a keypoit is away from the ego vehicle the smaller the estimated disparity, which agrees very well with the direct disparity estimation**.

Multi-Level Forward        |Multi-Level Inverse        |OpenCV
:-------------------------:|:-------------------------:|:-------------------------:
![Multi-Level, Forward](doc/image/03-disparity-map/multi-level-forward.png)  |  ![Multi-Level, Inverse](doc/image/03-disparity-map/multi-level-inverse.png)  |  ![OpenCV](doc/image/03-disparity-map/opencv.png)


