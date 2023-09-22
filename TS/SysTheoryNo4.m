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

disp("C = ");
disp(sysC);

disp("D = ");
disp(sysD);
%% Scelta delle tuple di parametri
% per gli autovalori del sistema
pA = det(l * eye(length(symA)) - symA);
sA = solve(pA == 0, l);

paramArray = [m1, m2, k, c];
paramValues = {
    [3, 7, 2.1, 0];
    [3, 7, 8.421, .42]
};
paramChoise = 1;

sysA = double(subs(symA, paramArray, paramValues{paramChoise}));
sysB = double(subs(symB, paramArray, paramValues{paramChoise}));

sAChoise = double(subs(sA, paramArray, paramValues{paramChoise}));

disp("σ(A) = ");
disp(sAChoise);
%% Progettiamo l'osservatore asintotico in forma di Kalmann
% come primo passo per la realizzazione del compensatore
rV = 1;
eps = 2;
[sysV, ~, polesV] = lqr(transpose(sysA) + eye(4) * eps * max(min(real(sAChoise)), .1), transpose(sysC), eye(4), rV);
sysV = transpose(sysV);

disp("Matrice di trasferimento uscita-stima V dell'OBS =")
disp(sysV);
    
disp("Matrice di trasferimento ingresso-stima R dell'OBS =")
disp(sysB - sysV * sysD);

disp("Matrice di aggiornamento della stima H dell'OBS =");
disp(sysA - sysV * sysC);

disp("Autovalori della dinamica d'errore dell'OBS =");
% disp(solve(vpa(det(l * eye(4) - (sysA + eye(4) * eps * (min(real(sAChoise))) - sysV * sysC)), 20) == 0, l));
disp(polesV);
%% Progettiamo ora  la matrice in retroazione
% dello stato stimato
rF = 1;
[sysF, ~, polesF] = lqr(sysA + eye(4) * eps * max(min(real(sAChoise)), .1), - sysB, eye(4), rF);

disp("Matrice di guadagno in retroazione dalla stima F =")
disp(sysF);

disp("Autovalori della dinamica d'errore del sistema retroazionato =");
% disp(solve(vpa(det(l * eye(4) - (sysA + eye(4) * eps * max(min(real(sAChoise)), .1) + sysB * sysF)), 20) == 0, l));
disp(polesF);
%% Effettuiamo l'analisi del compensatore
% nel caso volessimo far inseguire il sistema ingressi sinusoidali
M0 = .5;
sysAm = [
    0, - M0 ^ 2;
    1, 0
];
sysAwfl = [
    sysAm, zeros(2, 4)
    sysB * [0, 1],  sysA
];
sysBwfl = [
    1;
    0;
    zeros(size(sysB))   
];
sysCwfl = [
   sysD * [0, 1], sysC
];
sysDwfl = 0;
%% Progettiamo quindi il compensatore
% in modo che il sistema insegui anche sinusoidi
rVsin = 1;
[sysVsin, ~, polesVsin] = lqr(transpose(sysAwfl) + eye(6) * eps * max(min(real(sAChoise)), .1), transpose(sysCwfl), eye(6), rVsin);
sysVsin = transpose(sysVsin);

disp("Matrice di trasferimento uscita-stima V dell'OBS nell'inseguimento di sinusoidi =")
disp(sysVsin);
    
disp("Matrice di trasferimento ingresso-stima R dell'OBS nell'inseguimento di sinusoidi =")
disp(sysBwfl - sysVsin * sysDwfl);

disp("Matrice di aggiornamento della stima H dell'OBS nell'inseguimento di sinusoidi =");
disp(sysAwfl - sysVsin * sysCwfl);

disp("Autovalori della dinamica d'errore dell'OBS nell'inseguimento di sinusoidi =");
% disp(solve(vpa(det(l * eye(6) - (sysAsin + eye(6) * eps * (min(real(sAChoise))) - sysVsin * sysCsin)), 20) == 0, l));
disp(polesVsin);

rFsin = 1;
[sysFsin, ~, polesFsin] = lqr(sysAwfl + eye(6) * eps * max(min(real(sAChoise)), .1), - sysBwfl, eye(6), rFsin);

disp("Matrice di guadagno in retroazione dalla stima F nell'inseguimento di sinusoidi =")
disp(sysFsin);

disp("Autovalori della dinamica d'errore del sistema retroazionato nell'inseguimento di sinusoidi =");
% disp(solve(vpa(det(l * eye(6) - (sysAwfl + eye(6) * eps * max(min(real(sAChoise)), .1) + sysBwfl * sysFsin)), 20) == 0, l));
disp(polesFsin);
%% Ricaviamo quindi le matrici dello spazio di stato
% del controllore per questo sistema
sysAsin = sysAwfl - sysVsin * sysCwfl + sysBwfl * sysFsin - sysVsin * sysDwfl * sysFsin;
sysBsin = - sysVsin;
sysCsin = sysFsin;
sysDsin = 0;

sysAsinfinal = [
    sysAsin, zeros(length(sysAsin), length(sysAm));
    [1; 0] * sysCsin, sysAm
];
sysBsinfinal = [
    sysBsin;
    0;
    0
];
sysCsinfinal = [zeros(size(sysCsin)), 0, 1];
sysDsinfinal = zeros(size(sysDsin));
%% Avviamo la simulazione del sistema
% modificando di volta in volta le caratteristiche del sistema
paramNoisePower = 0;
paramValuesNoise = paramValues{paramChoise} .* (1 +  paramNoisePower * abs(randn(size(paramValues{paramChoise}))));

sysAnoise = double(subs(symA, paramArray, paramValuesNoise));
sysBnoise = double(subs(symB, paramArray, paramValuesNoise));

M0 = 3;
M1 = 10;
M2 = pi / 4;

sysTime = 400;

q1Init = 0;
dq1Init = 0;
q2Init = 0;
dq2Init = 0;
sysInit = [q1Init; q2Init; dq1Init; dq2Init];

open('SysTheoryNo4Model.slx');
sim('SysTheoryNo4Model.slx');