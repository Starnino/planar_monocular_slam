function p_cam = unproj(p_img)
  p_cam = [p_img(1);p_img(2);1];
endfunction

function [p_img, p_cam] = projectPoint(p_w)
  global width; global height;
  global z_near; global z_far;
  global K;
  
  p_img = [0;0]; % error condition
  p_cam = [0;0;0];
  
  % check whether point falls behind or too far
  if p_w(3) < z_near || p_w(3) > z_far
    return
  endif
  % projection and homogeneous division
  p_cam = K*p_w;
  x = p_cam(1)/p_cam(3);
  y = p_cam(2)/p_cam(3);
  % check whether the point is inside image plane
  if (x < 0 || 
     x > width ||
     y < 0 ||
     y > height)
     return
  endif
  
  p_img = [x; y];
endfunction