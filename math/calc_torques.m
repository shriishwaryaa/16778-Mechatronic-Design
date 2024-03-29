% W = weights, L = lengths, N = normals
function[T] = calc_torques(W, L, th)

totalW = 150; %total system weight

% T_leftFoot
n(2) = -1*(- W(1)*(L(1)*cos(th(1))) - W(2)*(L(1)*cos(th(1)) + L(2)*cos(th(2)))...
             - W(3)*(L(1)*cos(th(1)) + L(2)*cos(th(2))) - W(4)*(L(1)*cos(th(1)) + L(2)*cos(th(2)) + L(3))...
             - 2*W(3)*(2*L(3) + L(1)*cos(th(1)) + L(2)*cos(th(2)))...
             - 2*W(2)*(2*L(3) + L(1)*cos(th(1)) + L(2)*cos(th(2)))...
             - 2*W(1)*(2*L(3) + L(1)*cos(th(1)) + 2*L(2)*cos(th(2))))/...
              (2*(2*L(3) + 2*L(1)*cos(th(1)) + 2*L(2)*cos(th(2))));

n(1) = totalW - 2*n(2);

%T_knee = 0
T(1) =  - n(1)*(L(1)*cos(th(1))) - W(2)*(L(2)*cos(th(2))) - W(3)*(L(2)*cos(th(2)))...
         - W(4)*(L(2)*cos(th(2)) + L(3)) - 2*W(3)*(L(2)*cos(th(2)) + 2*L(3))...
         - 2*W(2)*(L(2)*cos(th(2)) + 2*L(3)) - 2*W(1)*(2*L(2)*cos(th(2)) + 2*L(3))...
         + 2*n(2)*(2*L(2)*cos(th(2)) + 2*L(3) + L(1)*cos(th(1)));

% T_hip = 0 at equilibrium
T(2) =  - n(1)*(L(1)*cos(th(1))) + L(2)*(cos(th(2))) + W(1)*(L(2)*cos(th(2)))...
        - W(4)*L(3) - 2*W(2)*L(3) - 2*W(3)*L(3) - 2*W(1)*(2*L(3) + L(2)*cos(th(2)))...
        + 2*n(2)*(2*L(3) + L(2)*cos(th(2)) + L(1)*cos(th(1)));
         
end
