% Terne e Parametri di Denavit - Hartenberg
clear; clc;

% qua definisco numericamente la matrice dei link 
L = [0.8, 0.7, 0.5, 0.4];          %Li (i=1-4)
theta = [pi/6, pi/4, -pi/6, pi/8]; % Angoli di giunto thetha i (i=1-4)

% Matrice DH standard: ogni riga è [theta_i, d_i, a_i, alpha_i]
DH_num = [
    theta(1),  0,  L(1),  0;
    theta(2),  0,  L(2),  0;
    theta(3),  0,  L(3),  0;
    theta(4),  0,  L(4),  0
    ];

disp('Tabella DH numerica [theta, d, a, alpha]:');
disp(DH_num);

%  Definizione Simbolica
syms th1 th2 th3 th4 L1 L2 L3 L4 real

DH_sym = [
    th1,  0,  L1,  0;
    th2,  0,  L2,  0;
    th3,  0,  L3,  0;
    th4,  0,  L4,  0
    ];

disp('Tabella DH simbolica [theta, d, a, alpha]:');
disp(DH_sym);




% costruisco e calcolo le matrici di trasformazione A_i
% Funzione anonima per la matrice di trasformazione DH standard
dh_mat = @(th, d, a, al) [
    cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
    sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
    0,        sin(al),          cos(al),         d;
    0,        0,                0,               1
    ];

% Lavoro le matrici A_i
% Calcolo Numerico con un unico ciclo e lo stampo a display
A_num = cell(1, 4);
for i = 1:4
    A_num{i} = dh_mat(DH_num(i,1), DH_num(i,2), DH_num(i,3), DH_num(i,4));
    fprintf('Matrice numerica A_%d:\n', i);
    disp(A_num{i});
end

% Calcolo Simbolico con un unico ciclo e lo stampo a display
A_sym = cell(1, 4);
for i = 1:4
    A_sym{i} = dh_mat(DH_sym(i,1), DH_sym(i,2), DH_sym(i,3), DH_sym(i,4));
    fprintf('Matrice simbolica A_%d:\n', i);
    disp(A_sym{i});
end

% TRASFORMAZIONE COMPLESSIVA T ED ESTRAZIONE (x, y, phi)

% --- A. Calcolo Simbolico ---
T_sym = simplify(A_sym{1} * A_sym{2} * A_sym{3} * A_sym{4});

disp('Matrice di trasformazione complessiva simbolica T:');
disp(T_sym);

% Estrazione simbolica esplicita
x_sym   = T_sym(1, 4);
y_sym   = T_sym(2, 4);
phi_sym = th1 + th2 + th3 + th4; % Equivale a atan2(T_sym(2,1), T_sym(1,1))

fprintf('x(theta) = %s\n', char(x_sym));
fprintf('y(theta) = %s\n', char(y_sym));
fprintf('phi(theta) = %s\n\n', char(phi_sym));

% Calcolo Numerico 
T_num = A_num{1} * A_num{2} * A_num{3} * A_num{4};

disp('Matrice di trasformazione complessiva numerica T:');
disp(T_num);

% Estrazione numerica esplicita
x_num   = T_num(1, 4);
y_num   = T_num(2, 4);
phi_num = atan2(T_num(2, 1), T_num(1, 1)); % Orientamento in radianti [-pi, pi]

fprintf('Posizione End-Effector: x = %.4f m, y = %.4f m\n', x_num, y_num);
fprintf('Orientamento End-Effector: phi = %.4f rad (%.2f deg)\n', phi_num, rad2deg(phi_num));

% FUNZIONE fk_4R E GRAFICO DELLA CONFIGURAZIONE

% Richiamo della funzione con i parametri definiti sopra
[T_fun, x_fun, y_fun, phi_fun] = fun_4R(theta, L);

fprintf('Verifico la funzione fun_4R:\n');
fprintf('x = %.4f m, y = %.4f m, phi = %.4f rad (%.2f deg)\n', ...
    x_fun, y_fun, phi_fun, rad2deg(phi_fun));

% Calcolo delle posizioni dei giunti del grafico

x_joints = zeros(1, 5);
y_joints = zeros(1, 5);
ang_comp = 0;

for i = 1:4
    ang_comp = ang_comp + theta(i);
    x_joints(i+1) = x_joints(i) + L(i)*cos(ang_comp);
    y_joints(i+1) = y_joints(i) + L(i)*sin(ang_comp);
end

% Generazione del grafico
figure('Name', 'Cinematica Diretta - del braccio planare4R');
hold on; grid on; axis equal;

% Disegno dei link e dei giunti
plot(x_joints, y_joints, '-o', 'Color', [0.2 0.2 0.2], 'LineWidth', 3.5, ...
    'MarkerFaceColor', [0.9 0.4 0.1], 'MarkerSize', 8, 'DisplayName', 'Link e Giunti');

% Evidenzio la base (Giunto 1) e l'End-Effector
plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Base (0,0)');
plot(x_fun, y_fun, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'End-Effector');

% Etichette e limiti degli assi
L_tot = sum(L);
xlim([-L_tot*0.3, L_tot*1.1]);
ylim([-L_tot*0.3, L_tot*1.1]);
xlabel('X [m]');
ylabel('Y [m]');
title(sprintf('Configurazione Robot 4R (x=%.2fm, y=%.2fm, \\phi=%.1f°)', ...
    x_fun, y_fun, rad2deg(phi_fun)));
legend('Location', 'best');


% CINEMATICA INVERSA

% Punto target desiderato p = (x_des, y_des, phi_des)
x_des   = 1.2;
y_des   = 0.8;
phi_des = pi/4;

% Calcolo delle due soluzioni multiple
th_elbow_down = ifun_4R(x_des, y_des, phi_des, L,  1); % Gomito basso (+1)
th_elbow_up   = ifun_4R(x_des, y_des, phi_des, L, -1); % Gomito alto (-1)


[~, x_ver_d, y_ver_d, phi_ver_d] = fun_4R(th_elbow_down, L);
[~, x_ver_u, y_ver_u, phi_ver_u] = fun_4R(th_elbow_up, L);

fprintf('--- VERIFICA CINEMATICA INVERSA ---\n');
fprintf('Target:       x = %.4f, y = %.4f, phi = %.4f rad\n', x_des, y_des, phi_des);
fprintf('Gomito Basso: x = %.4f, y = %.4f, phi = %.4f rad (Errore pos: %.2e)\n', ...
    x_ver_d, y_ver_d, phi_ver_d, norm([x_des - x_ver_d, y_des - y_ver_d]));
fprintf('Gomito Alto:  x = %.4f, y = %.4f, phi = %.4f rad (Errore pos: %.2e)\n', ...
    x_ver_u, y_ver_u, phi_ver_u, norm([x_des - x_ver_u, y_des - y_ver_u]));

% Grafico comparativo delle due soluzioni
figure('Name', 'Cinematica Inversa - Soluzioni Multiple');
hold on; grid on; axis equal;

% Funzione helper inline per estrarre i punti dei giunti
get_pts = @(th) [0, cumsum(L .* cos(cumsum(th))); 0, cumsum(L .* sin(cumsum(th)))];

pts_d = get_pts(th_elbow_down);
pts_u = get_pts(th_elbow_up);

plot(pts_d(1,:), pts_d(2,:), '-o', 'LineWidth', 3, 'Color', [0 0.45 0.74], 'DisplayName', 'Gomito Basso');
plot(pts_u(1,:), pts_u(2,:), '--s', 'LineWidth', 3, 'Color', [0.85 0.33 0.1], 'DisplayName', 'Gomito Alto');
plot(x_des, y_des, 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Target');

xlim([-sum(L)*0.3, sum(L)*1.1]); ylim([-sum(L)*0.5, sum(L)*1.1]);
xlabel('X [m]'); ylabel('Y [m]');
title('Verifica IK: Soluzioni Multiple (Gomito Alto vs Gomito Basso)');
legend('Location', 'best');







% SOTTO LA DINAMICA SIMBOLICA CON EULERO-LAGRANGE
syms th1 th2 th3 th4 dth1 dth2 dth3 dth4 real
syms m1 m2 m3 m4 L1 L2 L3 L4 lc1 lc2 lc3 lc4 I1 I2 I3 I4 g real

q_sym  = [th1; th2; th3; th4];
dq_sym = [dth1; dth2; dth3; dth4];

% i baricentri
xc1 = lc1*cos(th1); yc1 = lc1*sin(th1);
xc2 = L1*cos(th1) + lc2*cos(th1+th2); yc2 = L1*sin(th1) + lc2*sin(th1+th2);
xc3 = L1*cos(th1) + L2*cos(th1+th2) + lc3*cos(th1+th2+th3);
yc3 = L1*sin(th1) + L2*sin(th1+th2) + lc3*sin(th1+th2+th3);
xc4 = L1*cos(th1) + L2*cos(th1+th2) + L3*cos(th1+th2+th3) + lc4*cos(th1+th2+th3+th4);
yc4 = L1*sin(th1) + L2*sin(th1+th2) + L3*sin(th1+th2+th3) + lc4*sin(th1+th2+th3+th4);

% le velocità lineari e angolari
v1_sq = diff(xc1,th1)^2*dth1^2 + diff(yc1,th1)^2*dth1^2; % Semplificato via Jacobiano
J1 = jacobian([xc1; yc1], q_sym); J2 = jacobian([xc2; yc2], q_sym);
J3 = jacobian([xc3; yc3], q_sym); J4 = jacobian([xc4; yc4], q_sym);

T_kin = 0.5*(m1*(dq_sym'*(J1'*J1)*dq_sym) + I1*dth1^2 + ...
    m2*(dq_sym'*(J2'*J2)*dq_sym) + I2*(dth1+dth2)^2 + ...
    m3*(dq_sym'*(J3'*J3)*dq_sym) + I3*(dth1+dth2+dth3)^2 + ...
    m4*(dq_sym'*(J4'*J4)*dq_sym) + I4*(dth1+dth2+dth3+dth4)^2);

V_pot = g * (m1*yc1 + m2*yc2 + m3*yc3 + m4*yc4);

% Estrazione matrici M(q), G(q), C(q, dq)
M_sym = simplify(hessian(T_kin, dq_sym));
G_sym = simplify(jacobian(V_pot, q_sym)');

C_sym = sym(zeros(4,4));
for k = 1:4
    for j = 1:4
        for i = 1:4
            c_ijk = 0.5 * (diff(M_sym(k,j), q_sym(i)) + diff(M_sym(k,i), q_sym(j)) - diff(M_sym(i,j), q_sym(k)));
            C_sym(k,j) = C_sym(k,j) + c_ijk * dq_sym(i);
        end
    end
end
C_sym = simplify(C_sym);
disp('Matrici simboliche M_sym, C_sym, G_sym calcolate.');





% CONTROLLORE, SIMULAZIONE, GRAFICI E ANIMAZIONE

% Parametri fisici del manipolatore
params.L  = L;
params.lc = L / 2;
params.m  = [2.0, 1.5, 1.0, 0.5];
params.I  = (1/12) * params.m .* (L.^2);
params.g  = 9.81;

% Parametri traiettoria circolare desiderata
t_span = [0 6];
xc = 1.2; yc = 0.5; R_traj = 0.35;      
omega = 2*pi / 6;                       % 1 giro completo in 6 s
phi_d = 0;                              % Orientamento costante

% Condizioni iniziali (partenza sulla traiettoria a t = 0)
q0  = ifun_4R(xc + R_traj, yc, phi_d, params.L, 1)';
dq0 = [0; 0; 0; 0];

% Integrazione dinamica con ode45
opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t_sim, state] = ode45(@(t, s) robot_ode(t, s, params, xc, yc, R_traj, omega, phi_d), ...
                       t_span, [q0; dq0], opts);

q_sim  = state(:, 1:4);
dq_sim = state(:, 5:8);

% Ricostruzione di errore e(t), coppie tau(t) e traiettoria
N = length(t_sim);
e_sim   = zeros(N, 4);
tau_sim = zeros(N, 4);
xy_sim  = zeros(N, 2);
xy_des  = zeros(N, 2);

for k = 1:N
    [qd, dqd, ddqd, xd, yd] = get_traj(t_sim(k), params.L, xc, yc, R_traj, omega, phi_d);
    xy_des(k, :) = [xd, yd];
    
    [~, x_c, y_c, ~] = fun_4R(q_sim(k, :)', params.L);
    xy_sim(k, :) = [x_c, y_c];
    
    e_sim(k, :)   = (qd - q_sim(k, :)')';
    tau_sim(k, :) = compute_tau(q_sim(k, :)', dq_sim(k, :)', qd, dqd, ddqd, params)';
end

% Grafici: q(t), e(t), tau(t)
figure('Name', 'Risultati Controllo 4R', 'Position', [100, 100, 900, 650]);

subplot(3,1,1);
plot(t_sim, rad2deg(q_sim), 'LineWidth', 1.5); grid on;
ylabel('q(t) [deg]'); title('Andamento delle variabili di giunto q(t)');
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', 'Location', 'best');

subplot(3,1,2);
plot(t_sim, rad2deg(e_sim), 'LineWidth', 1.5); grid on;
ylabel('e(t) [deg]'); title('Errore di inseguimento e(t) = q_d - q');
legend('e_1', 'e_2', 'e_3', 'e_4', 'Location', 'best');

subplot(3,1,3);
plot(t_sim, tau_sim, 'LineWidth', 1.5); grid on;
xlabel('Tempo [s]'); ylabel('\tau(t) [Nm]'); title('Coppie di controllo \tau(t)');
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', 'Location', 'best');

% Animazione con terne DH, target e drawnow
figure('Name', 'Animazione Manipolatore 4R', 'Position', [150, 150, 700, 700]);
step = max(1, floor(N / 100));

for k = 1:step:N
    clf; hold on; grid on; axis equal;
    xlim([-0.5, 2.5]); ylim([-1.0, 2.0]);
    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('Animazione 4R - t = %.2f s', t_sim(k)));
    
    % Target e scia End-Effector
    plot(xy_des(:,1), xy_des(:,2), 'g--', 'LineWidth', 1.5, 'DisplayName', 'Target');
    plot(xy_sim(1:k,1), xy_sim(1:k,2), 'b:', 'LineWidth', 1.5, 'DisplayName', 'Traiettoria EE');
    
    % Posizioni giunti
    q_k = q_sim(k, :);
    px = zeros(1, 5); py = zeros(1, 5);
    ang = 0;
    for i = 1:4
        ang = ang + q_k(i);
        px(i+1) = px(i) + params.L(i)*cos(ang);
        py(i+1) = py(i) + params.L(i)*sin(ang);
    end
    
    % Disegno robot
    plot(px, py, '-o', 'Color', [0.2 0.2 0.2], 'LineWidth', 3.5, ...
         'MarkerFaceColor', [0.9 0.4 0.1], 'MarkerSize', 7, 'DisplayName', 'Robot 4R');
    
    % Terne DH (X_i rosso, Y_i blu)
    sc = 0.2; ang_dh = 0;
    for i = 1:4
        ang_dh = ang_dh + q_k(i);
        quiver(px(i+1), py(i+1),  sc*cos(ang_dh), sc*sin(ang_dh), 0, 'r', 'LineWidth', 1.2);
        quiver(px(i+1), py(i+1), -sc*sin(ang_dh), sc*cos(ang_dh), 0, 'b', 'LineWidth', 1.2);
    end
    
    drawnow;
end

%  FUNZIONI LOCALI PER SIMULAZIONE
function ds = robot_ode(t, s, params, xc, yc, R, omega, phi_d)
    q  = s(1:4);
    dq = s(5:8);
    [qd, dqd, ddqd, ~, ~] = get_traj(t, params.L, xc, yc, R, omega, phi_d);
    tau = compute_tau(q, dq, qd, dqd, ddqd, params);
    [M, C, G] = dyn_4R(q, dq, params);
    ddq = M \ (tau - C*dq - G);
    ds = [dq; ddq];
end

function tau = compute_tau(q, dq, qd, dqd, ddqd, params)
    [M, C, G] = dyn_4R(q, dq, params);
    Kp = diag([150, 150, 100, 80]);
    Kd = diag([25, 25, 20, 15]);
    tau = M * (ddqd + Kd*(dqd - dq) + Kp*(qd - q)) + C*dq + G;
end

function [qd, dqd, ddqd, xd, yd] = get_traj(t, L, xc, yc, R, omega, phi_d)
    dt = 1e-4;
    xd = xc + R*cos(omega*t);
    yd = yc + R*sin(omega*t);
    qd   = ifun_4R(xd, yd, phi_d, L, 1)';
    qd_p = ifun_4R(xc + R*cos(omega*(t+dt)), yc + R*sin(omega*(t+dt)), phi_d, L, 1)';
    qd_m = ifun_4R(xc + R*cos(omega*(t-dt)), yc + R*sin(omega*(t-dt)), phi_d, L, 1)';
    dqd  = (qd_p - qd_m) / (2*dt);
    ddqd = (qd_p - 2*qd + qd_m) / (dt^2);
end
