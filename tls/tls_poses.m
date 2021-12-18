source "./tools/geometry_helpers.m"
source "./tls/tls_indices.m"

% (chordal) error and jacobian of a measured pose
% input:
%   Xi: the observing robot pose (4x4 homogeneous matrix)
%   Xj: the observed robot pose (4x4 homogeneous matrix)
%   Z: the relative transform measured between Xi and Xj
% output:
%   e: 12x1 is the difference between prediction, and measurement, vectorized
%   Ji: 12x6 derivative w.r.t a the error and a perturbation of the first pose
%   Jj: 12x6 derivative w.r.t a the error and a perturbation of the second pose

function [e, J_i, J_j] = poseErrorAndJacobian(X_i, X_j, z)
  global Rx0;
  global Ry0;
  global Rz0;
  % prediction
  z_hat = X_i^-1*X_j;
  % error
  e = flatten(z_hat) - flatten(z);
  % jacobian
  J_i = zeros(12,6); J_j = zeros(12,6); 
  R_i_transpose = X_i(1:3,1:3)';
  R_j = X_j(1:3,1:3); 
  t_j = X_j(1:3,4); 
  r_dax = reshape(R_i_transpose*Rx0*R_j, 9, 1);
  r_day = reshape(R_i_transpose*Ry0*R_j, 9, 1);
  r_daz = reshape(R_i_transpose*Rz0*R_j, 9, 1);
  J_j(1:9,4:6) = [r_dax r_day r_daz];
  J_j(10:12,1:3) = R_i_transpose;
  J_j(10:12,4:6) = -R_i_transpose*skew(t_j);
  J_i = -J_j;
 endfunction;

#linearizes the pose-pose measurements
# input:
#   XR: the initial robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the initial landmark estimates (3xnum_landmarks matrix of landmarks)
#   ZR: the robot_robot measuremenrs (4x4xnum_measurements: array of homogeneous matrices)
#   associations: 2xnum_measurements k = [i_idx, j_idx]. the kth observation made from pose i_idx, to j_idx
#   kernel_threshod: robust kernel threshold
# output:
#   H: the H matrix, filled
#   b: the b vector, filled
#   chi_tot: the total chi2 of the current round
#   num_inliers: number of measurements whose error is below kernel_threshold

function [H, b, chi_tot, num_inliers] = buildLinearSystemPoses(XR, XL, Zr, associations, kernel_threshold)
  % retrieve problem variables
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  global system_size; global test;
  
  % initialize
  H = zeros(system_size,system_size);
  b = zeros(system_size,1);
  chi_tot = 0;
  num_inliers = 0;
  
  % for each measurement
  for m = 1:size(Zr,3)
    pose_index_ij = associations(1:2,m);

    if !test
      X_i = v2t(XR(pose_index_ij(1),:));
      X_j = v2t(XR(pose_index_ij(2),:));       
    else % examples/least_squares.m 
      X_i = XR(:,:,pose_index_ij(1)); 
      X_j = XR(:,:,pose_index_ij(2));
    endif

    z = Zr(:,:,m);
    [e,J_i,J_j] = poseErrorAndJacobian(X_i, X_j, z);

    % chi2
    Omega = eye(12);
    Omega(1:9,1:9) *= 1e3;
    chi = e'*Omega*e;
    if chi > kernel_threshold
      Omega *= sqrt(kernel_threshold/chi);
      chi = kernel_threshold;
    else
      num_inliers ++;
    endif;
    chi_tot += chi;

    % update H and b
    pose_matrix_index_i = poseMatrixIndex(pose_index_ij(1));
    pose_matrix_index_j = poseMatrixIndex(pose_index_ij(2));
    H(pose_matrix_index_i:pose_matrix_index_i+pose_dim-1,
      pose_matrix_index_i:pose_matrix_index_i+pose_dim-1) += J_i'*Omega*J_i;
    H(pose_matrix_index_i:pose_matrix_index_i+pose_dim-1,
      pose_matrix_index_j:pose_matrix_index_j+pose_dim-1) += J_i'*Omega*J_j;
    H(pose_matrix_index_j:pose_matrix_index_j+pose_dim-1,
      pose_matrix_index_i:pose_matrix_index_i+pose_dim-1) += J_j'*Omega*J_i;
    H(pose_matrix_index_j:pose_matrix_index_j+pose_dim-1,
      pose_matrix_index_j:pose_matrix_index_j+pose_dim-1) += J_j'*Omega*J_j;
    b(pose_matrix_index_i:pose_matrix_index_i+pose_dim-1) += J_i'*Omega*e;
    b(pose_matrix_index_j:pose_matrix_index_j+pose_dim-1) += J_j'*Omega*e;
  endfor
endfunction