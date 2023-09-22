%% Definizione dei parametri
% del sistema in forma simbolica 
syms m1 m2 k c l;

symA = [
        zeros(2, 2),        eye(2);
        -k/m1, k/m1,    -c/m1, c/m1;
        k/m2, -k/m2,    c/m2, -c/m2
];
symB = [
   0;
   0;
   1/m1;
   0
];
symM = [
    0;
    0;
    0;
    1/m2;
];
sysC = [
    1,  0,  0,  0
];
sysD = 0;

disp("A = ");
disp(symA);

disp("B = ");
disp(symB);

disp("M = ");
disp(symM);

disp("C = ");
disp(sysC);

disp("D = ");
disp(sysD);
%% Verifica della OBS
% al variare dei parametri del sistema
Q = sym(zeros(length(symA), length(sysC)));
Q(1, :) = sysC;
for i = (2 : 1 : length(symA))
    Q(i, :) =  Q(i - 1, :) * symA;
end
disp("Q = ");
disp(Q);

disp("Sistema OBS se k != 0 (rank Q = " + rank(Q) + ")");

disp("Sistema parzialmente non OBS se k = 0 (rank Q = " + rank(subs(Q, k, 0)) + ")");
disp("ker Q = ");
disp(null(subs(Q, k, 0)));

disp("Sistema parzialmente non OBS se k = c = 0 (rank Q = " + rank(subs(Q, [k, c], [0, 0])) + ")");
disp("ker Q = ");
disp(null(subs(Q, [k, c], [0, 0])));
%% Scelta delle tuple di parametri
% per gli autovalori del sistema
pA = det(l * eye(length(symA)) - symA);
sA = solve(pA == 0, l);

paramArray = [m1, m2, k, c];
paramValues = {
    [3, 7, 2.1, 0];
    [3, 7, 8.421, .42];
    [3, 7, 63 / 40, 4.2]
};
paramChoise = 2;

sysA = double(subs(symA, paramArray, paramValues{paramChoise}));
sysB = double(subs(symB, paramArray, paramValues{paramChoise}));
sysM = double(subs(symM, paramArray, paramValues{paramChoise}));

sAChoise = double(subs(sA, paramArray, paramValues{paramChoise}));

disp("σ(A) = ");
disp(sAChoise);
%% Progettiamo gli OBS asintotici
% iniziando da quello più lento
eps = .1;
sAFeedback = (min(real(sAChoise)) - eps) *  [1; 1; 1; 1];

disp("σ(A) in retroazione (lento) = ");
disp(sAFeedback);

pAFeedback = prod(l * ones(4, 1) - sAFeedback);
    
Pi = double(subs(transpose(Q), paramArray, paramValues{paramChoise}));
Pi = Pi ^ (-1);
Pi = Pi(length(Pi), :);

sysF = - Pi * polyvalm(double(sym2poly(pAFeedback)), transpose(sysA));

sysVslow = - transpose(sysF);
disp("Matrice di trasferimento uscita-stima V (lento) =");
disp(sysVslow);

disp("Matrice di trasferimento ingresso-stima R (lento) =")
disp(sysB - sysVslow * sysD);

disp("Matrice di aggiornamento della stima H (lento) =");
disp(sysA - sysVslow * sysC);

disp("Autovalori della dinamica d'errore a ciclo chiuso (lento) =");
disp(solve(vpa(det(l * eye(4) - (sysA - sysVslow * sysC)), 20) == 0, l));
%% Proseguiamo con gli OBS veloci
% con autovalori multipli di quello più veloce
eps = 10;
sAFeedback = eps * min((min(real(sAChoise))), - .1) * [1; 1; 1; 1];

disp("σ(A) in retroazione (veloce) = ");
disp(sAFeedback);

pAFeedback = prod(l * ones(4, 1) - sAFeedback);

sysF = - Pi * polyvalm(double(sym2poly(pAFeedback)), transpose(sysA));

sysVfast = - transpose(sysF);
disp("Matrice di trasferimento uscita-stima V (veloce) =");
disp(sysVfast);

disp("Matrice di trasferimento ingresso-stima R (veloce) =")
disp(sysB - sysVfast * sysD);

disp("Matrice di aggiornamento della stima H (veloce) =");
disp(sysA - sysVfast* sysC);

disp("Autovalori della dinamica d'errore a ciclo chiuso (veloce) =");
disp(solve(vpa(det(l * eye(4) - (sysA - sysVfast * sysC)), 20) == 0, l));
%% Progettiamo ora l'OBS di Kalmann
% con nella forma base e con spostamento del semipiano
r = 1;
[sysVkmnn, ~, polesVkmnn] = lqr(transpose(sysA) + eye(4) * eps * (min(real(sAChoise))), transpose(sysC), eye(4), r);
sysVkmnn = transpose(sysVkmnn);

disp("Autovalori della dinamica d'errore a ciclo chiuso (Kalmann) =");
disp(polesVkmnn);

disp("Matrice di trasferimento uscita-stima V (Kalmann) =")
disp(sysVkmnn);
    
disp("Matrice di trasferimento ingresso-stima R (Kalmann) =")
disp(sysB - sysVkmnn * sysD);

disp("Matrice di aggiornamento della stima H (Kalmann) =");
disp(sysA - sysVkmnn * sysC);

disp("Autovalori della dinamica d'errore a ciclo chiuso (Kalmann) =");
% disp(solve(vpa(det(l * eye(4) - (sysA + eye(4) * eps * (min(real(sAChoise))) - sysVkmnn * sysC)), 20) == 0, l));
disp(polesVkmnn);
%% Avviamo la simulazione del sistema
% modificando di volta in volta le caratteristiche del sistema
paramNoisePower = 0;
paramValuesNoise = paramValues{paramChoise} .* (1 +  paramNoisePower * abs(randn(size(paramValues{paramChoise}))));
sysAnoise = double(subs(symA, paramArray, paramValuesNoise));
sysBnoise = double(subs(symB, paramArray, paramValuesNoise));

sysTime = 100;

q1Init = 12;
dq1Init = 0;
q2Init = 3;
dq2Init = 0;
sysInit = [q1Init; q2Init; dq1Init; dq2Init];

noisePower = 0;
noiseTime = 1;

disturbPower = 0;
disturbTime= 1;

open('SysTheoryNo3Model.slx');
sim('SysTheoryNo3Model.slx');