close all
clear
clc

Z = {};
fid = fopen('../dataset/meas-00000.dat');
pose_id = textscan(fid, 'seq: %d'){1};
xr = textscan(fid, 'odom_pose: %f %f %f', 'Headerlines', 1, 'CollectOutput', 1){1};
m = textscan(fid, 'point %f %f %f %f', 'CollectOutput', 1){1};
for p = 1:length(m)
  Z{end+1} = struct('pose_id', pose_id, 'pose', xr, 'landmark_id', m(p,2), 'point', m(p,3:4));
endfor
fclose(fid);
disp(Z{1});