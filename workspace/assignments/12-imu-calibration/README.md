# Visual Inertial Odometry: IMU -- 从零开始手写VIO: IMU

This is the solution of Assignment 02 of Hands on VIO from [深蓝学院](https://www.shenlanxueyuan.com/course/247).

深蓝学院从零开始手写VIO第2节IMU传感器答案. 版权归深蓝学院所有. 请勿抄袭.

当前ROS Melodic Workspace包含两个C++ Package, 两Package为对Dr HeYiJia与GaoWenLiang实现的重构, 方便之后同生产解决方案的集成:

* `simulator`
    * Generate `Simulated IMU Measurement`
    * Publish `Groud Truth Odometry` and `Integrated Odometry from IMU`
* `calibration`
    * Calibrate IMU using `Allan Variance Analysis`
    * Write `Allan Variance Parameters` as `JSON` file
    * Write `Smoothed Allan Variance Curve` as `CSV` file

---

## Set Up Environment

The solution is organized as a ROS Melodic workspace. First init the workspace as:

```bash
# make sure you have properly configured ROS melodic:
catkin_init_workspace
```

Then build release as:

```bash
# build for release then setup session for solution:
catkin config --install && catkin build
```

Finally, setup the session for solution stack with the command below. Then you are ready to reproduce the results.

```bash
# set up session for solution
source install/setup.bash
```

---

## Up & Running

First, **generate the simulated IMU measurement for Allan Variance calibration** by:

```bash
# generate simulated static IMU measurement for Allan Variance calibration:
roslaunch simulator simulator.launch
```

Then get the `calibration results`(in JSON) and `Allan Variance curve`(in CSV) using the command below:

```bash
# calibrate using Allan Variance analysis:
roslaunch calibration calibration.launch
```

--

## Solutions

### 1. Calibrate IMU with Allan Variance Analysis
### 1. 生成 Allen 方差标定曲线

原始标定结果请参见:

* [IMU Calibration Results](imu-calibration-results.json)
* [Allan Variance Curve](imu-calibration-results.csv)

绘制曲线所用的脚本参见[Here](src/calibration/calibration/visualize.py):

#### Gyroscope

`Gyroscope`的标定结果如下. 测量噪声能够得到较精准的估计, Bias的随机游走能精准到数量级

```json
{
    "gyro": {
        "measurement_noise_stddev": 0.01468, 
        "bias_random_walk_stddev": 3.436e-05
    }
}
```

其`Allan Variance Curve`为:

<img src="doc/imu-calibration-results--gyro.png" alt="Allan Variance Curve, Gyroscope" width="100%">

#### accelerometer

`Accelerometer`的标定结果如下. 测量噪声能够得到较精准的估计, Bias的随机游走能精准到数量级

```json
{
    "acc": {
        "measurement_noise_stddev": 0.01880, 
        "bias_random_walk_stddev": 3.559e-04
    }
}
```

其`Allan Variance Curve`为:

<img src="doc/imu-calibration-results--acc.png" alt="Allan Variance Curve, Accelerometer" width="100%">

---

### 2. IMU Integrated Odometry with Euler & Mid-Value Integration
### 2. 将IMU仿真代码中的欧拉积分替换成中值积分

TBD

---

### 3. Generate Trajectory using B-Spline
### 3. 使用B-Spline从已有轨迹生成IMU数据

TBD



