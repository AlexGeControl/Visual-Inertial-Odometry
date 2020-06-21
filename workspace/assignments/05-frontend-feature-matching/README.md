# Visual Odometry: Feature Matching -- 视觉里程计: 特征点法

This is the solution of Assignment 05 of Visual SLAM: Theory and Practice from [深蓝学院](https://www.shenlanxueyuan.com/my/course/240).

深蓝学院视觉SLAM理论与实践第五节课习题答案. 版权归深蓝学院所有. 请勿抄袭.

---

## Solutions

### 1. ORB Feature Matching
### 1. ORB 特征点

The solution is available at (click to follow the link) [here](01-orb-feature-matching/compute_orb.cpp)

#### Oriented Fast

```c++
void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints) {
    static const int HALF_PATCH_SIZE = 8;
    static const float RADIUS_TO_DEGREE = 180.0 / M_PI;

    for (auto &keypoint : keypoints) {
        int row = static_cast<int>(keypoint.pt.y);
        int col = static_cast<int>(keypoint.pt.x);
        
        keypoint.angle = 0.0f;

        // whether the keypoint is inside ROI:
        if (
            (HALF_PATCH_SIZE <= row && row < image.rows - HALF_PATCH_SIZE) && 
            (HALF_PATCH_SIZE <= col && col < image.cols - HALF_PATCH_SIZE)
        ) {
            // if so, calculate angle for the keypoint:
            int m10{0}, m01{0};
            for (int y = -HALF_PATCH_SIZE; y < +HALF_PATCH_SIZE; ++y) {
                for (int x = -HALF_PATCH_SIZE; x < +HALF_PATCH_SIZE; ++x) {
                    int I = static_cast<int>(
                        image.at<unsigned char>(row+y, col+x)
                    );

                    m10 += x * I;
                    m01 += y * I;
                }
            }
            // calculate orientation:
            keypoint.angle = static_cast<float>(
                atan2(
                    static_cast<double>(m01), 
                    static_cast<double>(m10)
                )
            );

            // convert to OpenCV angle:
            keypoint.angle *= RADIUS_TO_DEGREE;
        }
    }

    return;
}
```

#### ORB Description

```c++
void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, vector<DescType> &desc) {
    static const size_t N = 256;
    static const float DEGREE_TO_RADIAN = M_PI / 180.0;

    for (auto &keypoint: keypoints) {
        DescType d;
        bool out_of_bound = false;

        // calculate descriptor:
        for (int i = 0; i < N; i++) {
            // get current ORB pattern:
            int u_p{ORB_pattern[(i << 2) + 0]}, v_p{ORB_pattern[(i << 2) + 1]};
            int u_q{ORB_pattern[(i << 2) + 2]}, v_q{ORB_pattern[(i << 2) + 3]};

            // get feature point orientation:
            double theta = keypoint.angle * DEGREE_TO_RADIAN;
            double cos_theta{cos(theta)}, sin_theta{sin(theta)};

            // get pixel coordinates in oriented keypoint frame:
            int x_p = static_cast<int>(keypoint.pt.x + cos_theta*u_p - sin_theta*v_p);
            int y_p = static_cast<int>(keypoint.pt.y + sin_theta*u_p + cos_theta*v_p);
            int x_q = static_cast<int>(keypoint.pt.x + cos_theta*u_q - sin_theta*v_q);
            int y_q = static_cast<int>(keypoint.pt.y + sin_theta*u_q + cos_theta*v_q);

            // whether the sample pixel is still inside the image:
            if (
                (0 <= x_p && x_p < image.cols) && (0 <= x_q && x_q < image.cols) && 
                (0 <= y_p && y_p < image.rows) && (0 <= y_q && y_q < image.rows)
            ) {
                // std::cout << "\t Pair" << i << std::endl;
                d[i] = (
                    (image.at<unsigned char>(y_p, x_p) > image.at<unsigned char>(y_q, x_q)) ? true : false
                );
            } else {
                out_of_bound = true;
                break;
            }
        }

        if (!out_of_bound) {
            desc.push_back(d);
        }
    }

    int total = keypoints.size();
    int bad = keypoints.size() - desc.size();
    cout << "bad/total: " << bad << "/" << total << endl;
    return;
}
```

#### Brute-Force Matching

```c++
void bfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches) {
    static const size_t N = 256;
    static const int D_MAX = 50;

    for (int i = 0; i < desc1.size(); ++i) {
        cv::DMatch match{i, 0, N};

        for (int j = 0; j < desc2.size(); ++j) { 
            int distance = (desc1.at(i) ^ desc2.at(j)).count();

            if (distance >= D_MAX) continue;

            if (distance < match.distance) {
                match.trainIdx = j;
                match.distance = distance;
            }
        }

        if (match.distance < D_MAX) {
            matches.push_back(match);
        }
    }

    for (auto &m: matches) {
        cout << m.queryIdx << ", " << m.trainIdx << ", " << m.distance << endl;
    }

    return;
}
```

#### Results and Review

The matching result is shown below.

<img src="doc/01-orb-feature-matching/matches.png" alt="ORB Feature Matching Result">

##### 1. Why ORB is a binary feature?
##### 1. 为什么说 ORB 是一种二进制特征?

Because each ORB description is a vector on boolean field.

##### 2. Why 50 is used as distance threhold? How would its value influence matching?
##### 2. 为什么在匹配时使用 50 作为阈值,取更大或更小值会怎么样?

The value is chosen to strike a balance between **quality of matching** and **the number of matchings available for later constraint optimization**.

- Smaller value would keep only high quality matches but the number of constraints available for optimization would be smaller.
- Larger value would create more constraints but the quality of matching would be reduced.

##### 3. How could the implementation be further optimized?
##### 3. 暴力匹配在你的机器上表现如何?你能想到什么减少计算量的匹配方法吗?

The execution time is shown below. Here C++ `bitset` is used to replace `std::vector<bool>` as `DescType`.

```bash
# run ORB feature matching:
time ./orb_feature_matching 
# time it:
real	0m0.167s
user	0m0.322s
sys	0m0.786s
```

Further optimization could be achieved by **Using KDTree for feature matching**. But the search tree should be adapted to work on bitset.

---

### 2. Reconstruct Relative Pose from Essential Matrix
### 2. 从 E 恢复 R, t

The solution is available at (click to follow the link) [here](02-essential-matrix-to-pose/E2Rt.cpp)

#### Implementation

```c++
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
```

#### Result

```bash
[Essential Matrix to Relative Pose]:
	R1 = 
  -0.365887  -0.0584576    0.928822
-0.00287462    0.998092   0.0616848
   0.930655  -0.0198996    0.365356
	t1 = 
 -0.581301
-0.0231206
  0.401938
	R2 = 
  -0.365887  -0.0584576    0.928822
-0.00287462    0.998092   0.0616848
   0.930655  -0.0198996    0.365356
	t2 = 
 0.581301
0.0231206
-0.401938
	R3 = 
 -0.998596  0.0516992 -0.0115267
-0.0513961   -0.99836 -0.0252005
 0.0128107  0.0245727  -0.999616
	t3 = 
 0.581301
0.0231206
-0.401938
	R4 = 
 -0.998596  0.0516992 -0.0115267
-0.0513961   -0.99836 -0.0252005
 0.0128107  0.0245727  -0.999616
	t4 = 
 -0.581301
-0.0231206
  0.401938
	 Reconstructed, from t^R =
 -0.0203619   -0.400711  -0.0332407
   0.393927   -0.035064    0.585711
-0.00678849   -0.581543  -0.0143826
	 Relative error, F-norm, %: 5.74042e-14
```