close all
clear -all
clc

%---------------------------------------- SETUP ----------------------------------------

% import tools
cd('..')
source './tools/geometry_helpers.m'
source './tools/projection_helpers.m'
source './tls/total_least_squares.m'

% global variables
global test; test = true;
global num_poses = 200;
global num_landmarks = 1000;
global K; K = [150,0,320; 0, 150,240; 0, 0, 1];
global z_near = 0; global z_far = 100; global width = 540; global height = 480;
global pose_dim = 6; global landmark_dim = 3; 
global system_size = pose_dim*num_poses + landmark_dim*num_landmarks;

%---------------------------------- INITIALIZATION ---------------------------------------
world_size = 100;

# landmarks in a matrix, one per column
P_world = (rand(landmark_dim, num_landmarks)-0.5)*world_size;

# poses in an array of 4x4 homogeneous transform matrices
XR_true = zeros(4,4,num_poses);
XL_true = P_world;

# initialize 1st pose
XR_true(:,:,1) = eye(4);

# scaling coefficient for uniform random pose generation
# adjusts the translation to cover world_size
# adjusts the rotation to span the three angles;
rand_scale=eye(6);
rand_scale(1:3,1:3)*=(0.5*world_size);
rand_scale(4:6,4:6)*=pi;

for pose_num = 2:num_poses
    xr = rand(6,1)-0.5;
    Xr = v2t(rand_scale*xr);
    XR_true(:,:,pose_num) = Xr;
endfor

# apply a perturbation to each ideal pose (construct the estimation problem)
pert_deviation=1;
pert_scale=eye(6)*pert_deviation;
XR_guess=XR_true;
XL_guess=XL_true;

for (pose_num=2:num_poses)
    xr=rand(6,1)-0.5;
    dXr=v2t(pert_scale*xr);
    XR_guess(:,:,pose_num)=dXr*XR_guess(:,:,pose_num);
endfor;

#apply a perturbation to each landmark
dXl=(rand(landmark_dim, num_landmarks)-0.5)*pert_deviation;
XL_guess+=dXl;

%---------------------------------- BUNDLE ADJUSTMENT -------------------------------------

% pose-pose constraints
Zr = zeros(4,4,num_poses-1);
pose_associations = zeros(2,num_poses-1);

for r = 1:num_poses-1
  Xr_i = XR_true(:,:,r);
  Xr_j = XR_true(:,:,r+1);
  Zr(:,:,r) = Xr_i^-1*Xr_j;
  pose_associations(:,r) = [r;r+1];
endfor

% pose-landmark projection constraints
num_projections = num_poses*num_landmarks;
Zp = zeros(2,num_projections);
projection_associations = zeros(2,num_projections);

projection_id = 1;
for p = 1:num_poses
  inv_Xr = XR_true(:,:,p)^-1;
  for m = 1:num_landmarks
    Xl = [XL_true(:,m);1];
    img_point = projectPoint((inv_Xr*Xl)(1:3));
    if img_point
      Zp(:,projection_id) = img_point;
      projection_associations(:,projection_id) = [p;m];
      projection_id ++;
    endif
  endfor
endfor
# crop the projection associations to something meaningful
projection_associations = projection_associations(:,1:projection_id-1);
Zp = Zp(:,1:projection_id-1);


% total least square algorithm
damping = 1;
kernel_threshold = 1e3;
num_iterations = 5;
[XR,XL,chi_stats,num_inliers,H,b] = total_ls(XR_guess, XL_guess, 
								                             Zp, projection_associations, 
                                             Zr, pose_associations, 
                                             num_iterations, damping, kernel_threshold);

                              
%----------------------------------- VISUALIZATION ----------------------------------------

figure(1)
hold on;
grid;

subplot(2,2,1);
plot3(XL_true(1,:),XL_true(2,:),XL_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XL_guess(1,:),XL_guess(2,:),XL_guess(3,:),'ro',"linewidth",2);
title("Landmark Initial Guess");
legend("True", "Guess");grid;

subplot(2,2,2);
plot3(XL_true(1,:),XL_true(2,:),XL_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XL(1,:),XL(2,:),XL(3,:),'ro',"linewidth",2);
title("Landmark After Optimization");
legend("True", "Guess");grid;

subplot(2,2,3);
plot3(XR_true(1,:),XR_true(2,:),XR_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XR_guess(1,:),XR_guess(2,:),XR_guess(3,:),'ro',"linewidth",2);
title("Poses Initial Guess");
legend("True", "Guess");grid;

subplot(2,2,4);
plot3(XR_true(1,:),XR_true(2,:),XR_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XR(1,:),XR(2,:),XR(3,:),'ro',"linewidth",2);
title("Poses After Optimization");
legend("True", "Guess"); grid;

figure(2);
hold on;
grid;
title("chi evolution");

subplot(3,2,1);
plot(chi_stats(1,:), 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");
subplot(3,2,2);
plot(num_inliers(1,:), 'b-', "linewidth", 2);
legend("#inliers"); grid; xlabel("iterations");

subplot(3,2,5);
plot(chi_stats(2,:), 'r-', "linewidth", 2);
legend("Chi Proj"); grid; xlabel("iterations");
subplot(3,2,6);
plot(num_inliers(2,:), 'b-', "linewidth", 2);
legend("#inliers");grid; xlabel("iterations");

figure(3);
title("H matrix");
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_)) = 0;               # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;