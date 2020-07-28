# Jacobian Derivation for Optimization

There are two types of error in VIO bundle adjustment, namely 

* `feature point reprojection error` and 
* `key frame pose constraint under IMU pre-integration`

---

## Prerequisites

`IMU multiplication` can also be expressed as `matrix multiplication`

`left multiplication by quaternion q1`, **q1*q2**, can be expressed as:

```python
LHS * Matrix([w_2, x_2, y_2, z_2])
```

In which LHS is defined as

<img src="quaternion/LHS-matrix.png" alt="Left Multiplication Matrix" width="100%">

`right multiplication by quaternion q1`, **q2*q1**, can be expressed as:

```python
RHS * Matrix([w_2, x_2, y_2, z_2])
```

In which RHS is defined as

<img src="quaternion/RHS-matrix.png" alt="Right Multiplication Matrix" width="100%">

---

## Reprojection Error

### Overview

A new reprojection error is introduced when `Key Frame i` and `Key Frame j` have common observations

<img src="reprojection/overview--problem-statement.png" alt="Reprojection Error, Definition" width="100%">

In order to ease later derivation, define intermediate variables for `feature point position representation` as 

<img src="reprojection/intermediate-variables--feature-point-position.png" alt="Reprojection Error, Feature Point Position Representation" width="100%">

And choose intermediate variable for `Jacobian derivation` as

<img src="reprojection/intermediate-variables--jacobian-derivation.png" alt="Reprojection Error, Jacobian Derivation" width="100%">

### Derivation

#### Key Frame i

<img src="reprojection/key-frame-i.png" alt="Reprojection Error Jacobian, Key Frame i" width="100%">

#### Key Frame j

<img src="reprojection/key-frame-j.png" alt="Reprojection Error Jacobian, Key Frame j" width="100%">

#### IMU-Camera Extrinsic

First comes the Jacobian with respect to `extrinsic position`:

<img src="reprojection/extrinsic--position.png" alt="Reprojection Error Jacobian, IMU-Camera Extrinsic, Position" width="100%">

First comes the Jacobian with respect to `extrinsic orientation`, which consists of 2 terms:

<img src="reprojection/extrinsic--orientation.png" alt="Reprojection Error Jacobian, IMU-Camera Extrinsic, Orientation" width="100%">

<img src="reprojection/extrinsic--orientation.png" alt="Reprojection Error Jacobian, IMU-Camera Extrinsic, Orientation" width="100%">

<img src="reprojection/extrinsic--orientation--part-01.png" alt="Reprojection Error Jacobian, IMU-Camera Extrinsic, Orientation, Part 01" width="100%">

<img src="reprojection/extrinsic--orientation--part-02.png" alt="Reprojection Error Jacobian, IMU-Camera Extrinsic, Orientation, Part 02" width="100%">

#### Inverse Depth

<img src="reprojection/inverse-depth.png" alt="Reprojection Error Jacobian, Inverse Depth" width="100%">

---

## IMU Pre-Integration Constraint

### Overview

The IMU pre-integration constraint is introduced by IMU measurements between two adjacent key frames

<img src="pre-integration/overview.png" alt="Key Frame Pose Constraint Overview" width="100%">

### Derivation

#### Position

<img src="pre-integration/position.png" alt="Key Frame Pose Constraint Jacobian, Position" width="100%">

#### Orientation

The derivation of orientation terms need to use `matrix representation of quaternion product` in section `prerequisite`. It consists of `three terms`, Jacobian with respect to `Key Frame i`, `Key Frame j` and `gyro bias of Key Frame i`

<img src="pre-integration/position.png" alt="Key Frame Pose Constraint Jacobian, Position" width="100%">

The Jacobian with respect to `Key Frame i`:

<img src="pre-integration/orientation--key-frame-i.png" alt="Key Frame Pose Constraint Jacobian, Orientation, Key Frame i" width="100%">

The Jacobian with respect to `Key Frame j`:

<img src="pre-integration/orientation--key-frame-j.png" alt="Key Frame Pose Constraint Jacobian, Orientation, Key Frame j" width="100%">

The Jacobian with respect to `bias of Key Frame i`:

<img src="pre-integration/orientation--gyro-bias.png" alt="Key Frame Pose Constraint Jacobian, Orientation, Gyro Bias of Key Frame j" width="100%">