source "./tools/geometry_helpers.m"
source "./tls/tls_indices.m"

# error and jacobian of a measured landmark
# input:
%   Xr: the robot pose in world frame (4x4 homogeneous matrix)
%   Xl: the landmark pose (3x1 vector, 3d pose in world frame)
%   z:  projection of the landmark on the image plane
% output:
%   e: 2x1 is the difference between prediction and measurement
%   Jr: 2x6 derivative w.r.t a the error and a perturbation on the pose
%   Jl: 2x3 derivative w.r.t a the error and a perturbation on the landmark
%   is_valid: true if projection ok

function [is_valid,e,Jr,Jl]=projectionErrorAndJacobian(Xr,Xl,z)
  global width; global height;
  global z_near; global z_far;
  global K; global X_cam;
  global test;
  is_valid = false; % error condition
  e = [0;0]; 
  Jr = zeros(2,6);
  Jl = zeros(2,3); 
  % prediction
  if !test
    [z_hat, p_cam] = projectPoint((X_cam^-1*Xr^-1*[Xl;1])(1:3));  
  else % examples/least_squares.m  
    [z_hat, p_cam] = projectPoint((Xr^-1*[Xl;1])(1:3)); 
  endif
  
  if !z_hat
    return
  endif
  % error
  e = z_hat - z;
  % jacobian
  inv_R = Xr(1:3,1:3)';
  Jwr = zeros(3,6);
  Jwr(1:3,1:3) = -inv_R;
  Jwr(1:3,4:6) = inv_R*skew(Xl);
  Jwl = inv_R;
   
  Jp=[p_cam(3) 0 -p_cam(1)/p_cam(3)^2;
      0 p_cam(3) -p_cam(2)/p_cam(3)^2];
  
  Jr = Jp*K*Jwr;
  Jl = Jp*K*Jwl;
  is_valid = true;
endfunction;


%linearizes the robot-landmark measurements
% input:
%   XR: the initial robot poses (4x4xnum_poses: array of homogeneous matrices)
%   XL: the initial landmark estimates (3xnum_landmarks matrix of landmarks)
%   Z:  the measurements (2xnum_measurements)
%   associations: 2xnum_measurements. k = [p_idx,l_idx] the kth observation made from pose p_idx to landmark l_idx
%   num_poses: number of poses in XR (added for consistency)
%   num_landmarks: number of landmarks in XL (added for consistency)
%   kernel_threshod: robust kernel threshold
% output:
%   XR: the robot poses after optimization
%   XL: the landmarks after optimization
%   chi_stats: array 1:num_iterations, containing evolution of chi2
%   num_inliers: array 1:num_iterations, containing evolution of inliers

function [H, b, chi_tot, num_inliers] = buildLinearSystemProjections(XR, XL, Zl, associations, kernel_threshold)
  % retrive problem variables
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  global system_size; global test;
  
  % initialize
  H = zeros(system_size, system_size);
  b = zeros(system_size,1);
  chi_tot = 0;
  num_inliers = 0;

  % for each measurement
  for m = 1:size(Zl,2)
    pose_index = associations(1,m);
    landmark_index = associations(2,m);
    z = Zl(:,m);
    if !test
      Xr = v2t(XR(pose_index,:));
    else % examples/least_squares.m 
      Xr = XR(:,:,pose_index); 
    endif
    
    Xl = XL(:,landmark_index);
    [is_valid,e,Jr,Jl] = projectionErrorAndJacobian(Xr, Xl, z);
    
    % chi2 
    if !is_valid
       continue;
    endif
    chi = e'*e;
    if chi > kernel_threshold
      e *= sqrt(kernel_threshold/chi);
      chi = kernel_threshold;
    else
      num_inliers++;
    endif
    chi_tot += chi;
     
    % update H and b
    pose_matrix_index = poseMatrixIndex(pose_index);
    landmark_matrix_index = landmarkMatrixIndex(landmark_index);
    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1) += Jr'*Jr;
    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jr'*Jl;
    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jl'*Jl;
    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1) += Jl'*Jr;
    b(pose_matrix_index:pose_matrix_index+pose_dim-1) += Jr'*e;
    b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jl'*e;
  endfor
endfunction
