function se3mat = ECE569_VecTose3(V)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.
% Example Input:
% 
% clear; clc;
% V = [1; 2; 3; 4; 5; 6];
% se3mat = VecTose3(V)
% 
% Output:
% se3mat =
%     0    -3     2     4
%     3     0    -1     5
%    -2     1     0     6
%     0     0     0     0 

omega = V(1:3);                 % first 3 entries = angular part
v     = V(4:6);                 % last 3 entries  = linear part

so3mat = ECE569_VecToso3(omega);

se3mat = [so3mat, v;           % top-right is v
          0 0 0   0];          % last row is zeros
end