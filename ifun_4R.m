function theta = ifun_4R(x, y, phi, L, elbow)
% Calcola della cinematica inversa 
%   elbow : +1 per gomito basso (default), -1 per gomito alto
% Output:

if nargin < 5
    elbow = 1; % posiziono il gomito basso
end

% Equazione del polso 
xw = x - L(4)*cos(phi);
yw = y - L(4)*sin(phi);

% ricalcolo la posizione dei precedenti 3 link rendendo i link 2 e 3 collineari in quanto un link risulta ridondante
L1  = L(1);
L23 = L(2) + L(3); % <--eccolo 
r2  = xw^2 + yw^2;

D = (r2 - L1^2 - L23^2) / (2 * L1 * L23);

% Controllo raggiungibilità nello spazio di lavoro
if abs(D) > 1
    error('Punto (%.2f, %.2f) fuori dallo spazio di lavoro raggiungibile!', x, y);
end

% Calcolo theta2 (gomito alto / gomito basso)
th2 = atan2(elbow * sqrt(1 - D^2), D);

% Calcolo theta1
th1 = atan2(yw, xw) - atan2(L23 * sin(th2), L1 + L23 * cos(th2));

% Scelta di ridondanza per theta3
th3 = 0;

% 3. Calcolo del quarto giunto
th4 = phi - (th1 + th2 + th3);

theta = [th1, th2, th3, th4];
end
