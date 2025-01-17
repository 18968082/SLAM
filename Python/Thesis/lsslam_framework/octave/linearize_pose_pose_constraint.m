% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error

  x1_homgen = v2t(x1);
  x2_homgen = v2t(x2);
  z_homgen = v2t(z);
  
  e_homogen = invt(z_homgen)*(invt(x1_homgen)*x2_homgen);
  e = t2v(e_homogen);
  
  
  
  disp(e_homogen);
  disp('');
  
  A = 0;
  B = 0;

end;
