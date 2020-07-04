# Bundle Adjustment in the Large, Custom G2O Node with Analytic Jacobian

---

## Derivation of Analytic Jacobian

In this section the analytic expression of Jacobians needed for custom G2O node be derived. 

The derivation is based on `SymPy` inside `Jupyter Lab`.

### Point-to-Pixel Projection

For BAL dataset a pinhole camera model is used. The parameters we estimate for each camera are: 

* Rotation `R`
* Translation `t`

Here we treat the following parameters as fixed:

* Focal length `f`
* Two radial distortion parameters `k1` and `k2`. 

The formula for projecting a 3D point X into a camera using R,t,f,k1,k2 is:

* P  =  R * X + t       (conversion from world to camera coordinates)
* p  = -P / P.z         (perspective division)
* p' =  f * r(p) * p    (conversion to pixel coordinates)

where P.z is the third (z) coordinate of P. 

In the last equation, r(p) is a function that computes a scaling factor to undo the radial distortion:
* r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.

This gives a projection in pixels, where 

* The `origin of the image` is `the center of the image`
* The `positive x-axis` points `right`
* The `positive y-axis` points `up` 

(in addition, in the camera coordinate system, the positive z-axis points backwards, so the camera is looking down the negative z-axis, as in OpenGL).

### Intermediate Variables

For derivation `landmark coordinates in camera frame, X`, is used as `intermediate variable`.

### Jacobian for Camera Pose

Using the [Jupyter notebook](bal-jacobian.ipynb), the C++ Eigen expression of `camera pose Jacobian` can be shown as:

```c++
// this is the analytic Jacobian expression generated automatically by SymPy
double P_X_2 = pow(X, 2);
double P_X_4 = pow(X, 4);
double P_Y_2 = pow(Y, 2);
double P_Y_4 = pow(Y, 4);
double P_Z_2 = pow(Z, 2);
double P_Z_4 = pow(Z, 4);
double P_Z_5 = pow(Z, 5);
double P_Z_6 = pow(Z, 6);

Eigen::Matrix<double, 2, 6> J;

J(0, 0) = -f*(5*P_X_4*k2 + 6*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + P_Y_4*k2 + P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_5;
J(0, 1) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))/P_Z_5;
J(0, 2) = P_X*f*(5*P_X_4*k2 + 10*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_6;
J(0, 3) = P_X*P_Y*f*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + 2*P_Z_2*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)) + k2*(P_X_2 + P_Y_2)**2 + 2*(P_X_2 + P_Y_2)*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)))/P_Z_6;
J(0, 4) = -f*(P_X_2*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2 + 2*(P_X_2 + P_Y_2)*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))) + P_Z_2*(2*P_X_2*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)) + P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2))/P_Z_6;
J(0, 5) = P_Y*f*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2)/P_Z_5;
J(1, 0) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))/P_Z_5;
J(1, 1) = -f*(P_X_4*k2 + 6*P_X_2*P_Y_2*k2 + P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_5;
J(1, 2) = P_Y*f*(5*P_X_4*k2 + 10*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_6;
J(1, 3) = f*(P_Y_2*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2 + 2*(P_X_2 + P_Y_2)*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))) + P_Z_2*(2*P_Y_2*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)) + P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2))/P_Z_6;
J(1, 4) = -P_X*P_Y*f*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + 2*P_Z_2*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)) + k2*(P_X_2 + P_Y_2)**2 + 2*(P_X_2 + P_Y_2)*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2)))/P_Z_6;
J(1, 5) = -P_X*f*(P_Z_4 + P_Z_2*k1*(P_X_2 + P_Y_2) + k2*(P_X_2 + P_Y_2)**2)/P_Z_5;
```

### Jacobian for Landmark Position

Using the [Jupyter notebook](bal-jacobian.ipynb), the C++ Eigen expression of `lamdmark position Jacobian` can be shown as:

```c++
// this is the analytic Jacobian expression generated automatically by SymPy
double P_X_2 = pow(X, 2);
double P_X_4 = pow(X, 4);
double P_Y_2 = pow(Y, 2);
double P_Y_4 = pow(Y, 4);
double P_Z_2 = pow(Z, 2);
double P_Z_4 = pow(Z, 4);
double P_Z_5 = pow(Z, 5);
double P_Z_6 = pow(Z, 6);

Eigen::Matrix<double, 2, 3> J;

J(0, 0) = -f*(5*P_X_4*k2 + 6*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + P_Y_4*k2 + P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_5;
J(0, 1) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))/P_Z_5;
J(0, 2) = P_X*f*(5*P_X_4*k2 + 10*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_6;
J(1, 0) = -2*P_X*P_Y*f*(P_Z_2*k1 + 2*k2*(P_X_2 + P_Y_2))/P_Z_5;
J(1, 1) = -f*(P_X_4*k2 + 6*P_X_2*P_Y_2*k2 + P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_5;
J(1, 2) = P_Y*f*(5*P_X_4*k2 + 10*P_X_2*P_Y_2*k2 + 3*P_X_2*P_Z_2*k1 + 5*P_Y_4*k2 + 3*P_Y_2*P_Z_2*k1 + P_Z_4)/P_Z_6;
```

