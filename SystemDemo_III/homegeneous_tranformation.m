% Define four 4x4 matrices
ten_deg = 0.1745;
syms theta;

A = [1 0 0 0; 0 1 0 0; 0 0 1 -0.0125; 0 0 0 1];
B = [1 0 0 0; 0 1 0 0; 0 0 1 0.9125; 0 0 0 1];
% C = [cos(theta) -sin(theta) 0 -0.1; sin(theta) cos(theta) 0 0; 0 0 1 -0.1; 0 0 0 1];
D = [1 0 0 -0.225; 0 1 0 0; 0 0 1 0; 0 0 0 1];

M1 = A * B;

for i = 1:10
    theta = i * ten_deg;
    C = [cos(theta) -sin(theta) 0 -0.1; sin(theta) cos(theta) 0 0; 0 0 1 -0.1; 0 0 0 1];
    M2 = C * D;
    M = M1 * M2;
    disp('Matrix for angle');
    disp(rad2deg(theta));
    disp(M);
end

