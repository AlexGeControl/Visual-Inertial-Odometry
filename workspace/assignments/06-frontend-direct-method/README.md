# Visual Odometry: Direct Method -- 视觉里程计: 直接法

This is the solution of Assignment 06 of Visual SLAM: Theory and Practice from [深蓝学院](https://www.shenlanxueyuan.com/my/course/240).

深蓝学院视觉SLAM理论与实践第六节课习题答案. 版权归深蓝学院所有. 请勿抄袭.

---

### Up and Running

The solution has been tested inside Docker CPU/GPU.

```bash
# go to workspace:
cd /workspace/assignments/06-frontend-direct-method
# build:
mkdir build && cd build && cmake .. && make -j8
# execute the solution 
```

---

## Solutions

### 1. Lucas-Kanade Optical Flow
### 1. LK 光流

The solution is available at (click to follow the link) [here](01-optical-flow/optical_flow.cpp)

#### Implementation

Below is the C++ implementation of **Lucas-Kanade Optical Flow**

```c++
void OpticalFlowSingleLevel(
        const Mat &image_target,
        const Mat &image_source,
        const vector<KeyPoint> &keypoints_target,
        vector<KeyPoint> &keypoints_source,
        vector<bool> &success,
        bool inverse
) {
    // parameters
    int HALF_PATCH_SIZE = 4;
    int MAX_ITERATIONS = 10;
    bool have_initial = !keypoints_source.empty();

    // initialize images:
    ImageWithGradient T(image_target);
    ImageWithGradient I(image_source);

    for (size_t i = 0; i < keypoints_target.size(); ++i) {
        const auto &keypoint = keypoints_target[i];

        // initialize x0 and y0:
        double x0{keypoint.pt.x}, y0{keypoint.pt.y};

        // initialize dx and dy:
        double dx{0.0}, dy{0.0};
        if (have_initial) {
            dx = keypoints_source.at(i).pt.x - x0;
            dy = keypoints_source.at(i).pt.y - y0;
        }

        // initialize Hessian:
        Eigen::Matrix2d H;

        State state(dx, dy);

        // Gauss-Newton iterations
        if (T.IsValidPatch(x0, y0, HALF_PATCH_SIZE)) {
            for (int j = 0; j < MAX_ITERATIONS; j++) {
                dx = state.p(0), dy = state.p(1);
                double x = x0 + dx;
                double y = y0 + dy;

                // whether the current patch is inside the image:
                if (!I.IsValidPatch(x, y, HALF_PATCH_SIZE)) {
                    state.success = false;
                    break;
                }

                Eigen::Vector2d b = Eigen::Vector2d::Zero();

                if (!inverse) {
                    H = Eigen::Matrix2d::Zero();
                } else {
                    if (!state.is_inverse_initialized) {
                        // for inverse method the approximated Hessian will not change for each patch
                        H = Eigen::Matrix2d::Zero();
                        for (int du = -HALF_PATCH_SIZE; du < HALF_PATCH_SIZE; ++du) {
                            for (int dv = -HALF_PATCH_SIZE; dv < HALF_PATCH_SIZE; ++dv) {
                                Eigen::Vector2d J = T.GetGradient(x0 + du, y0 + dv);
                                H += J * J.transpose(); 
                            }
                        }
                        state.is_inverse_initialized = true;
                    }
                }   
                
                double cost = 0.0;

                // compute cost and jacobian:
                Eigen::Vector2d J;
                for (int du = -HALF_PATCH_SIZE; du < HALF_PATCH_SIZE; ++du) {
                    for (int dv = -HALF_PATCH_SIZE; dv < HALF_PATCH_SIZE; ++dv) {
                        // compute error:
                        double error = T.GetPixelValue(x0 + du, y0 + dv) - I.GetPixelValue(x + du, y + dv);

                        // compute Jacobian:
                        if (!inverse) {
                            J = I.GetGradient(x + du, y + dv);
                            H += J * J.transpose();
                        } else {
                            J = T.GetGradient(x0 + du, y0 + dv);
                        }

                        // update b and cost;
                        b += error * J;

                        cost += 0.5 * error*error;
                    }
                }

                // compute update
                Eigen::Vector2d dp = H.fullPivHouseholderQr().solve(b);

                if (!state.Update(dp, cost)) {
                    break;
                }
            }
        }

        success.push_back(state.success);

        // set source keypoint:
        dx = state.p(0), dy = state.p(1);
        if (have_initial) {
            keypoints_source.at(i).pt = keypoint.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = keypoint;
            tracked.pt += cv::Point2f(dx, dy);
            keypoints_source.push_back(tracked);
        }
    }
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

<img src="doc/image/01-optical-flow/opencv.png" alt="Multi-Level Optical Flow from OpenCV">

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