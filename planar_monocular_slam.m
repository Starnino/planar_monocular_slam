close all
clear -all
clc

%---------------------------------------- SETUP ----------------------------------------

% import tools
source './tools/geometry_helpers.m'
source './tools/projection_helpers.m'
source './tls/total_least_squares.m'

% global variables
global test = false;
global num_poses; global num_landmarks;
global K; global X_cam; global z_near, global z_far; global width; global height;
global pose_dim = 6; global landmark_dim = 3; global system_size;

% pose data 
fid = fopen('dataset/trajectory.dat');
data = textscan(fid, 'pose: %d %f %f %f %f %f %f', 'CollectOutput', 1)(2){:};
XR_guess = data(:,1:3);
XR_true = data(:,4:6);
num_poses = length(XR_true);
fclose(fid);

% landmark data
fid = fopen('dataset/world.dat');
XL_true = textscan(fid, '%d %f %f %f', 'CollectOutput', 1)(2){:}';
num_landmarks = length(XL_true);
fclose(fid);

% system size
system_size = pose_dim*num_poses + landmark_dim*num_landmarks;

% measurements
files = dir('dataset');
files = regexpi({files.name},'meas-[0-9]{5}.dat','match');
files = [files{:}];
Z = cell(1, length(files));
num_projections = 0;

for k = 1:length(files);
	fid = fopen(strcat('dataset/', files{k}));
  pose_id = textscan(fid, 'seq: %f'){1} + 1;
  xr = textscan(fid, 'odom_pose: %f %f %f', 'Headerlines', 1, 'CollectOutput', 1){1};
  m = textscan(fid, 'point %f %f %f %f', 'CollectOutput', 1){1};
  Z{k} = struct('pose_id', pose_id, 'pose', xr, 'landmark_ids', m(:,2) + 1, 'points', m(:,3:4));
  num_projections = num_projections + length(Z{k}.points);
	fclose(fid);
endfor

% camera parameters
fid = fopen('dataset/camera.dat');
K = cell2mat(textscan(fid, '%f %f %f', 'Headerlines', 1));
X_cam = cell2mat(textscan(fid, '%f %f %f %f', 'Headerlines', 1));
z_near = textscan(fid, '%d', 'Headerlines', 1){1};
z_far = textscan(fid, '%d', 'Headerlines', 1){1};
width = textscan(fid, '%d', 'Headerlines', 1){1};
height = textscan(fid, '%d', 'Headerlines', 1){1};
fclose(fid);

% disp problem settings
disp('---PROBELM SETTINGS---')
fprintf('Num poses = %d\n', num_poses)
fprintf('Num landmarks = %d\n', num_landmarks)
fprintf('Num projections = %d\n\n', num_projections)

%---------------------------------- INITIALIZATION ---------------------------------------

fprintf('---LANDMARK TRIANGULATION ')

% landmarks 
XL_guess = zeros(3,num_landmarks);
landmarks_missed = 0;

% for each landmark l-th
for l = 1:num_landmarks
  
  % sequence of poses where the directions toward 
  % l-th landmark have been observed by the camera 
  poses = [];
  dirs = [];

  for p = 1:num_poses
    % find landmark id position if it has been observed
    id = find(Z{p}.landmark_ids == l);
    if id
      % retrieve pose
      Xr = v2t(Z{p}.pose);
      % take 3d point of the current pose
      poses(:,end+1) = [Xr(1:3,4)];
      % take direction in camera frame
      p_cam = unproj(Z{p}.points(id,:));
      % compute direction toward l in world coordinates
      dirs(:,end+1) = (Xr*X_cam*[(K^-1*p_cam);1])(1:3);
    endif
  endfor
  
  % landmark not observed
  if length(dirs) < 2
    XL_guess(:,l) = [NaN; NaN; NaN];
    landmarks_missed++;
  elseif
    % compute least square intersection
    XL_guess(:,l) = linesIntersection(poses, dirs);
  endif
endfor

fprintf('ENDED---\n')
fprintf('Missed landmarks = %d\n\n', landmarks_missed);

%---------------------------------- BUNDLE ADJUSTMENT -------------------------------------
disp('---TOTAL LEAST SQUARES---')

% pose-pose constraints
Zr = zeros(4,4,num_poses-1);
pose_associations = zeros(2,num_poses-1);

for r = 1:num_poses-1
  Xr_i = v2t(XR_guess(r,:));
  Xr_j = v2t(XR_guess(r+1,:));
  Zr(:,:,r) = Xr_i^-1*Xr_j;
  pose_associations(:,r) = [r;r+1];
endfor

% pose-landmark projection constraints
Zp = zeros(2,num_projections);
projection_associations = zeros(2,num_projections);

projection_id = 1;
for p = 1:num_poses
  pose_id = Z{p}.pose_id;
  for m = 1:length(Z{p}.points)
    landmark_id = Z{p}.landmark_ids(m);
    Zp(:,projection_id) = Z{p}.points(m,:);
    projection_associations(:,projection_id) = [pose_id;landmark_id];
    projection_id ++;
  endfor
endfor

% total least square algorithm
damping = 1;
kernel_threshold = 1e3;
num_iterations = 20;
[XR,XL,chi_stats,num_inliers,H,b] = total_ls(XR_guess, XL_guess, 
								                             Zp, projection_associations, 
                                             Zr, pose_associations, 
                                             num_iterations, damping, kernel_threshold);

                              
%----------------------------------- VISUALIZATION ----------------------------------------

plots;