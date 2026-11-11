function [T, x, y, phi] = fun_4R(theta, L)
% Calcola della cinematica diretta per un manipolatore planare 4R

th1 = theta(1);
th2 = theta(2);
th3 = theta(3);
th4 = theta(4);

% Somme cumulative degli angoli
phi_1   = th1;
phi_12  = th1 + th2;
phi_123 = th1 + th2 + th3;
phi     = th1 + th2 + th3 + th4;

% Coordinate dell'End-Effector
x = L(1)*cos(phi_1) + L(2)*cos(phi_12) + L(3)*cos(phi_123) + L(4)*cos(phi);
y = L(1)*sin(phi_1) + L(2)*sin(phi_12) + L(3)*sin(phi_123) + L(4)*sin(phi);

% Matrice di trasformazione omogenea complessiva T
T = [cos(phi), -sin(phi), 0, x;
    sin(phi),  cos(phi), 0, y;
    0,         0,        1, 0;
    0,         0,        0, 1];
end
