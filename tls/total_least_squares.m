source "./tools/geometry_helpers.m"
source "./tls/tls_indices.m"
source "./tls/tls_poses.m"
source "./tls/tls_projections.m"

% sums the perturbation vector to the state
function [XR,XL] = boxPlus(XR, XL, dx)
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  global test;
  % pose box plus
  for pose_index = 1:num_poses
    pose_matrix_index = poseMatrixIndex(pose_index);
    dxr = dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    if !test
      XR(pose_index,:) = t2v(v2t(dxr)*v2t(XR(pose_index,:)));
    else  % examples/least_squares.m 
      XR(:,:,pose_index) = v2t(dxr)*XR(:,:,pose_index);
    endif

  endfor
  % landmark box plus  
  for landmark_index = 1:num_landmarks
    landmark_matrix_index = landmarkMatrixIndex(landmark_index);
    dxl = dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(:,landmark_index) += dxl;
  endfor
endfunction

function [XR,XL,chi_stats,num_inliers,H,b] = total_ls(XR, XL,
                                                      Zp, projection_associations,
                                                      Zr, pose_associations,
                                                      num_iterations, damping, kernel_threshold)
  % retrieve problem variables
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  global system_size;
  % statistics
  chi_stats = zeros(2,num_iterations);
  num_inliers = zeros(2,num_iterations);
  fprintf('iteration ')
  for iteration = 1:num_iterations
    % disp iteration
    fprintf('%d ', iteration);
    % initialize H, b, dx
    H = zeros(system_size,system_size);
    b = zeros(system_size,1);
    dx = zeros(system_size,1);
    % build pose system  
    [H_r,b_r,chi_,num_inliers_] = buildLinearSystemPoses(XR, XL, Zr, pose_associations, kernel_threshold);
    chi_stats(1,iteration) += chi_;
    num_inliers(1,iteration) = num_inliers_;
    % build projection system
    [H_p,b_p,chi_,num_inliers_] = buildLinearSystemProjections(XR, XL, Zp, projection_associations, kernel_threshold);
    chi_stats(2,iteration) += chi_;
    num_inliers(2,iteration) = num_inliers_;
    % build H and b
    H += H_r + H_p;
    b += b_r + b_p;
    % damping
    H += eye(system_size)*damping;
    % solve the linear system (with 1st robot pose fixed) 
    dx(pose_dim+1:end) = -(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    % boxplus
    [XR,XL] = boxPlus(XR, XL, dx);
  endfor
  fprintf('\n')
endfunction