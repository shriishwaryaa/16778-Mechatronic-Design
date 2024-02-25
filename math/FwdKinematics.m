clc
clearvars

% Link lengths in meters
l1 = 0.05;
l2 = 0.5334;
l3 = 0.2286;

syms theta1 theta2 theta3

% DH Parameters
% Link 1
a1 = l1;
alpha1 = 0;
d1 = 0;
theta1 = theta1;

%Link 2
a2 = l2;
alpha2 = 0;
d2 = 0;
theta2 = theta2;

%Link 3
a3 = l3;
alpha3 = 0;
d3 = 0;
theta3 = theta3;

H0_1 = DH(a1, alpha1, d1, theta1)
H1_2 = DH(a2, alpha2, d2, theta2)
H2_3 = DH(a3, alpha3, d3, theta3)

H0_3 = H0_1 * H1_2 * H2_3

