% Function which calculates Ai which is the homogeneous transformation
% matrix that expresses the position and orientation of frame i w.r.t frame
% i - 1

function [A] = DH(a, alpha, d, theta)
A = [cos(theta) -sin(theta)*round(cos(alpha)) sin(theta)*round(sin(alpha))  a*cos(theta)
     sin(theta)  cos(theta)*round(cos(alpha)) -cos(theta)*round(sin(alpha)) a*sin(theta)
      0          round(sin(alpha))             round(cos(alpha))            d
      0             0                           0                           1];
end
