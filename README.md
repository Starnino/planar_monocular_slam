# Planar Monocular SLAM
Project repository for the course of Probabilistic Robotics 2020, Sapienza University of Rome.

## Abstract
Robotics applications which require understanding and interaction within the environment, rely on sensors data which provide noisy and uncertain informations. However thanks to probabilistic techniques it is possible to combine different kind of sensors data and minimize the error globally. This is the case of the Gauss-Newton Algorithm which comes useful for a simulation of a vacuum cleaner-ish robot that navigates in an environment collecting data from a monocular camera, capturing landmarks projections around.

## The project
A Octave implementation of the Planar Monocular Slam vacuum cleaner robot simulation. It is composed by the following files:
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
- **Reading Dataset**.
- **Landmark Initialization**.
- **Factor Graph Constraints**:
  - Pose-Pose.
  - Pose-Landmark projection.
- **Total Least Square Algorithm** (ith iteration):
  - Build pose linear system:
  - Build projection linear system:
  - Solve the system:
  - Box plus:
- **Plots**.

### Testing
smxoas

## References
> nnsnsnsn.2020
[](https://)
