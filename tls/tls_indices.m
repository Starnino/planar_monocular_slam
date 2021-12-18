% retrieves the index in the perturbation vector, that corresponds to a certain pose
% input:
%   pose_index: the index of the pose for which we want to compute the index
% output:
%   v_idx: the index of the sub-vector corrsponding to pose_index, in the array of perturbations

function v_idx = poseMatrixIndex(pose_index)
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  if pose_index > num_poses
    v_idx = -1;
  else
    v_idx = 1 + (pose_index-1)*pose_dim;
  endif
endfunction

% retrieves the index in the perturbation vector, that corresponds to a certain landmark
% input:
%   landmark_index: the index of the landmark for which we want to compute the index
% output: 
%   v_idx: the index of the sub-vector corresponding to the landmark_index, in the array of perturbations

function v_idx = landmarkMatrixIndex(landmark_index)
  global pose_dim; global landmark_dim;
  global num_poses; global num_landmarks;
  if landmark_index > num_landmarks
    v_idx = -1;
  else 
    v_idx = 1 + (num_poses)*pose_dim + (landmark_index-1)*landmark_dim;
  endif
endfunction;