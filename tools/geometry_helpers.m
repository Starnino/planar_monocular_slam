% 3D rotation matrix around x axis
function R=Rx(theta)
 c=cos(theta);
 s=sin(theta);
 R= [1  0  0;
     0  c  -s;
     0  s  c];
endfunction

% 3D rotation matrix around y axis
function R=Ry(theta)
 c=cos(theta);
 s=sin(theta);
 R= [c  0  s;
     0  1  0;
     -s  0 c];
endfunction

% 3D rotation matrix around z axis
function R=Rz(theta)
 c=cos(theta);
 s=sin(theta);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
endfunction

#derivative of rotation matrix w.r.t rotation around x, in 0
global  Rx0=[0 0 0;
	     0 0 -1;
	     0 1 0];

#derivative of rotation matrix w.r.t rotation around y, in 0
global  Ry0=[0 0 1;
	     0 0 0;
	     -1 0 0];

#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];
       
function S=skew(v)
  S=[0 -v(3) v(2);
     v(3) 0 -v(1);
     -v(2) v(1) 0];
endfunction

% computes the homogeneous transform matrix A of the pose vector v
% T:[ R t ] 4x4 homogeneous transformation matrix, r translation vector
% v: [x,y,z,alpha,beta,gamma] 6D pose vector
function T = v2t(v)
  % make 2D pose 3D pose
  if length(v) == 3
    v = [v(1) v(2) 0 0 0 v(3)];
  endif
  T=eye(4);
  T(1:3,1:3)=Rx(v(4))*Ry(v(5))*Rz(v(6));
  T(1:3,4)=v(1:3);
endfunction

% compute the pose vector v of the homogeneous 4x4 transform matrix
function v = t2v(T)
	v(1:2) = T(1:2,4);
	v(3) = atan2(T(2,1),T(1,1));
end

% vectorization of transform
function v = flatten(T)
  v = zeros(12,1);
  v(1:9) = reshape(T(1:3,1:3),9,1);
  v(10:12) = T(1:3,4);
endfunction

% normalize vector
function p = normalize(p)
  p = p/norm(p);
endfunction

% compute a least square solution of the closest point w.r.t. the input lines
function lsp = linesIntersection(p,l)
	n = size(p)(1);
	A = zeros(n,n);
	b = zeros(n,1);
  
	for i = 1:size(p)(2)
    v = normalize(l(:,i));
		s = eye(n,n) - v*v';
		A = A + s;
		b = b + s*p(:,i);
	endfor
	
	lsp = pinv(A)*b;
endfunction