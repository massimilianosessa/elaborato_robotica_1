function [M, C, G] = dyn_4R(theta, dtheta, params)
q = theta(:); dq = dtheta(:);
M = calc_M(q, params);
G = calc_G(q, params);

% Matrice C tramite simboli di Christoffel numerici
C = zeros(4, 4);
eps_val = 1e-6;
dM = zeros(4, 4, 4);
for k = 1:4
    qp = q; qp(k) = qp(k) + eps_val;
    qm = q; qm(k) = qm(k) - eps_val;
    dM(:,:,k) = (calc_M(qp, params) - calc_M(qm, params)) / (2*eps_val);
end
for k = 1:4
    for j = 1:4
        for i = 1:4
            C(k,j) = C(k,j) + 0.5*(dM(k,j,i) + dM(k,i,j) - dM(i,j,k))*dq(i);
        end
    end
end
end

function M = calc_M(q, p)
M = zeros(4, 4);
for i = 1:4
    Jv = zeros(2, 4); Jw = zeros(1, 4);
    for j = 1:i
        Jw(1, j) = 1;
        dx = 0; dy = 0;
        for k = j:i
            len = (k < i)*p.L(k) + (k == i)*p.lc(i);
            ang = sum(q(1:k));
            dx = dx - len*sin(ang);
            dy = dy + len*cos(ang);
        end
        Jv(1, j) = dx; Jv(2, j) = dy;
    end
    M = M + p.m(i)*(Jv'*Jv) + p.I(i)*(Jw'*Jw);
end
end

function G = calc_G(q, p)
G = zeros(4, 1);
for j = 1:4
    for i = j:4
        dy = 0;
        for k = j:i
            len = (k < i)*p.L(k) + (k == i)*p.lc(i);
            dy = dy + len*cos(sum(q(1:k)));
        end
        G(j) = G(j) + p.m(i)*p.g*dy;
    end
end
end
