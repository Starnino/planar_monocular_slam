# Planar Monocular SLAM
Project repository for the course of Probabilistic Robotics 2020, Sapienza University of Rome.

## Abstract
Robotics applications which require understanding and interaction within the environment, rely on sensors data which provide noisy and uncertain informations. However thanks to probabilistic techniques it is possible to combine different kind of sensors data and minimize the error globally. This is the case of the Gauss-Newton Algorithm which comes useful for a simulation of a vacuum cleaner-ish robot that navigates in an environment collecting data from a monocular camera, capturing landmarks projections around.

## The project
A Octave implementation of the Planar Monocular Slam vacuum cleaner robot simulation. The dataset comprises by the following files:
- ```world.dat``` It contains information about the map. Every row contains: 
  - LANDMARK_ID POSITION

- ```camera.dat``` It contains information about the camera used to gather data:
  - camera matrix
  - cam_transform: pose of the camera w.r.t. robot
  - z_near/z_far: how close/far the camera can perceive stuff
  - width/height of images

- ```trajectory.dat``` Every row contains:
  - pose: POSE_ID ODOMETRY_POSE GROUND_TRUTH_POSE
  
  the ODOMETRY_POSE is obtained by adding Gaussian Noise (0; 0.001) to the actual robot commands

- ```meas-XXXX.dat``` Every measurement contains a sequence number, ground truth (of the robot) and odometry pose and measurement information:
  - point POINT_ID_CURRENT_MESUREMENT ACTUAL_POINT_ID IMAGE_POINT 

  The Image_Point represents the pair [col;row] where the landmark is observed in the image.

The output of the project is summarized in some plots, showing the landmarks and the poses of the robot before and after the bundle adjustment exploiting the Gauss-Newton algorithm, errors and outliers at each iteration.

### Implementation Steps
- **State Identification**: identify state varibles involved in the problem and set the proper representations. These are the following

  | Variable  | Representation | Note |
  | ------------- | ------------- | ------------- |
  | Xr in SE(2) | Xr = (x_r y_r θ) | The pose of the robot belongs to the speciale orthonormal group SE(2) which is a 3x3 transformation matrix X = (R(θ) t) with the composition of a rotation matrix about z axis of an angle θ and a translation vector (x y 1)^T. The robot moves in the plane.|
  | Xl in R^3  | Xl = (x_l y_l z_l) | The 3D landmark positions in the world observed by the robot camera.|
  | ΔXr in R^3 | Xr = (Δx Δy Δθ) | The perturbation vector needed for the Gauss-Newton algorithm procedure. | 

- **Reading Dataset**: organize poses and measurements in propers data structures.
- **Landmark Initialization**: given a set of directions seen at different robot poses, one may compute the intersection for estimating the 3D landmark position in the world. The computation is perfomed by the function *LinesIntersection* in geometry_helpers.m.
- **Factor Graph Constraints**
  - Pose-Pose: the constraints is given by subsequent poses and it represents the pose j (next) expressed in the reference frame o pose i (previous). These constraints are of the form
    ```
    Measurement Z  =  Xr_j^i 
    Prediction  Z^ =  (Xr_i)^-1 Xr_j
    Error       e  =  flatten(Z) - flatten(Z^) 
    ```
    where Xr^i_j is the ideal measured robot transformation matrix of pose j seen by pose i, Xr_i the robot transformation matrix at pose i, Xr_j the robot transformation matrix at pose j, and the function *flatten* considers a vectorized version of the transformation matrix (from 4x4 to 12x1) to reduce computational costs. 
  - Pose-Landmark projection: the constraint is given by relating the estimated landmark 3D positions with the robot poses which have measured those landmarks with the camera. These constraints are of the form
    ```
    Measurement Z  =  (u v)^T 
    Prediction  Z^ =  π[K(X_cam^-1 Xr^-1 Xl)]
    Error       e  =  Z - Z^ 
    ```
    where (u v)^T is the measured image point from the camera, π is the homogeneous division function, K the camera matrix, X_cam the transformation matrix from robot to camera, Xr the robot transformation matrix of the current pose, and Xl is 3D position of the landmark seen by the current robot pose. 
- **Total Least Square Algorithm** (ith iteration)
  - Build pose linear system: build the pose-pose system given by the pose-pose associations, compute the error and the jacobian and sum the contribution to the matrix H and the vector b. 
  - Build projection linear system: build the projection system given by the pose-landmaerk projection associations, compute the error and the jacobian and sum the contribution to the matrix H and the vector b. 
  - Solve the system: solve the system `-b = H ΔX` in order to find the perturbation solution ΔX.
  - Box plus: sum the perturbation vector ΔX to the current state estimation.
- **Plots**: sctipt showing the poses and landmarks in the world, before and after the bundle adjustment process, together with error structure of the H matrix. The latter is a square matrix with dimension (num_poses x pose_dim + num_landmark x landmark_dim)x(num_poses x pose_dim + num_landmark x landmark_dim), which represents the involvements of state variables with the factor graph constraints. 

### Testing
In order to run the application in Octave one has to execute the script in the project directory
```
planar_monocular_slam
```
In the `examples` folder it possible to try the *lineIntersection* algorithm and an example of least squares with dummy data. One should run within the folder
```
least_squares   or    intersection3d
```

## References
> Johannes Traa - UIUC 2013
[Least-Squares Intersection of Lines](https://docplayer.net/21072949-Least-squares-intersection-of-lines.html)

> Giorgio Grisetti 2019/2020
[Probabilistic Robotics Course Site](https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2019-20)
