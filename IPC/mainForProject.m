%% Script per l'analisi e controllo di un pendolo inverso su carrello mobile
%
% Pulizia dello spazio di lavoro
%
clear
close all
clc
%%
%
% Creazione delle variabili simboliche per 
    % le caratteristiche del sistema (variabili nel tempo e costanti)
%
syms x0(t) t1(t) m0 m1 J1 L1 g
% Posizione del CoM del pendolo
x1 = x0 + L1 / 2 * sin(t1);
y1 = - L1 / 2 * cos(t1);
% Posizione terminale del DPI
xt = x1 + L1 / 2 * sin(t1);
yt = y1 - L1 / 2 * cos(t1);
% Calcolo delle velocità lineari
v0 = diff(x0);
vx1 = diff(x1);
vy1 = diff(y1);
% Calcolo delle velocità angolari
w1 = diff(t1);
%%
%
% Calcolo del lagrangiano LGR = KE - PE e derivate parziali rispetto alle posizioni
% $x_0, \vartheta_1$ e alle velocità $v_0, \omega_1$
%   
KE = .5 * (m0 * v0 ^ 2 ...
    + m1 * vx1 ^ 2 + m1 * vy1 ^ 2 ...
     + J1 * w1 ^ 2 ...
);
PE = - m1 * g * y1;
LGR = KE - PE
%%
%
% Rimpiazzo le accelerazioni degli stati con variabili simboliche (utile
% per semplificare l'espressione di queste ultime in termini delle altre
% variabili di stato)
%
syms a0 u1 u(t);
ddQ = [a0, u1];
%%
%
% Calcolo delle forze agenti sul sistema
%%
% $$F_{ext} = \partial_t\partial_{\dot q} L_{GR} -
% \partial_q L_{GR}$$
%
F = subs([ ...
    diff(diff(LGR, v0)) - diff(LGR, x0), ...
    diff(diff(LGR, w1)) - diff(LGR, t1), ...
], [diff(v0), diff(w1)], ddQ);
F = simplify(F)
%%
%
% Scelgo come forze esterne agenti sul sistema
% la forza (attuabile) sul carrello $u(t)$ e le forze di disspazione che
% sono lineari nelle velocità $d_0v_0, d_1\omega_1, d_2\omega_2$
%
sys = F == [u, 0];
sym(transpose(sys))
%%
%
% Nota: il sistema è non lineare nelle variabili di stato
% $q=[x_0, \vartheta_1]$, tantomeno nelle
% variabili estese $Q = [x_0, \vartheta_1, v_0,
% \omega_1]$
syms cartvel pendvel cartpos pendpos inforce;

newStateVars = [cartvel, pendvel, cartpos, pendpos, inforce];
oldStateVars = [diff(x0(t), t), diff(t1(t), t), x0(t), t1(t), u(t)];

sysWR2acc = subs(solve(sys, ddQ), oldStateVars, newStateVars);
sysWR2a0 = subs(sysWR2acc.a0, oldStateVars, newStateVars);
sysWR2u1 = sysWR2acc.u1;

matlabFunction(sysWR2a0, 'File', 'a0Funct');
matlabFunction(sysWR2u1, 'File', 'u1Funct');
%%
%
% Per linearizzare il sistema e sviluppare quindi uno schema di controllo
% per l'impianto, usiamo un'approssimazione di Taylor del primo ordine
% attorno ad un punto di lavoro $X_o,U_o$ per cui $\ddot x_0=\ddot
% \vartheta_1=\dot x_0=\dot\vartheta_1=0$ ($X,U$ sono variabili di stato e
% ingresso del sistema del primo ordine: nel nostro caso $X=[x_0,v_0,
% \vartheta_1,\omega_1]$, quindi al punto di lavoro $v_0=\omega_1=0$ mentre
% $u=0$, $x_0$ generica e $\vartheta_1\in\{0,\pi\}$)
%
oprState = zeros(1, 4);
latex(subs(sysWR2acc, newStateVars(1:4), oprState).a0)
latex(subs(sysWR2acc, newStateVars(1:4), oprState).u1)
oprState(5) = 0;
subs(sysWR2acc, newStateVars, oprState)
%%
%
% $$
%   \dot f(X, U)\approx \partial_X f(X_o,U_o)(X-X_o)
%   + \partial_U f(X_o,U_o)(U-U_o)
% $$
%
linearA = [ ...
    zeros(2, 2), eye(2); ...
    subs(jacobian([sysWR2a0, sysWR2u1], [cartpos, pendpos, cartvel, pendvel]), ...
    newStateVars, oprState) ...
]
linearB = [ ...
    zeros(2, 1);
     subs(jacobian([sysWR2a0, sysWR2u1], [inforce]), ...
    newStateVars, oprState)
]
%%
%
% Per le matrici $C$ e $D$ del sistema linearizzato scelgo $C= [0,1, 0,
% 0]$, $D=0$ (osservo l'attuale angolo del pendolo che, in ultima battuta,
% sarà la misura da stabilizzare)
%
linearC = [0, 1, 0, 0];
linearD = 0;
%%
%
% Per le costanti del sistema scelgo le stesse presenti nel modello
% Simulink, quindi $g=9.81, L_1=.61,m_1=0.21,m_0=0.455,J_1=\frac13m_1L_1^2$
%
linearA = subs(linearA, [J1, m0, g], [1 / 3 * m1 * L1^2, .455, 9.8]);
linearA = double(subs(linearA, [m1, L1], [.21, .61]))
linearB = subs(linearB, [J1, m0, g], [1 / 3 * m1 * L1^2, .455, 9.8]);
linearB = double(subs(linearB, [m1, L1], [.21, .61]))
%%
%
% Converto le matrici nella funzione di trasferimento dell'impianto
% linearizzato
%
[linearPnum, linearPden] = ss2tf(linearA, linearB, linearC, linearD)
linearP = minreal(tf(linearPnum, linearPden))
%%
%
% Passando ora allo studio della stabilità e, specificando di volere
% asservimento nullo per riferimenti costanti $r(t)= R\delta_{-1}(t)$
% dovremmo soddisfarre la condizione
%
%%
%
% $$
%   \lim_{s\rightarrow+0}\left|s\frac{R}{s}\frac{s^\rho D_P}{\mu
%   N_P}\right|=0 
% $$
%
%%
%
% che complessivamente porta a scegliere $\rho = 1$, quindi un controllore
% con un polo all'origine
%
%%
%
% Scelgo quindi un controllore PID che renda il sistema a ciclo chiuso
% asintoticamente stabile con guadagno dell'integratore $K_I\neq 0$
%
syms s KP KI KD;
N = 100;
Ctrl0num = KD * s ^ 2 + KP * s + KI;

Ctrl0den = s * (s / N + 1);

W0 = coeffs(expand(poly2sym(linearPnum, s) * Ctrl0num ...
+ poly2sym(linearPden, s) * Ctrl0den), s);
tableW0 = myRouth(W0);
vpa(tableW0, 3)
%% 
%
% Dal criterio di Routh sappiamo che $K_I<0$:
% scelgo $K_P = -10, K_I = -15, K_D = -2$ che soddisfano le condizioni di
% stabilità e danno origine ad un luogo delle radici favorevole alla
% realizzazione del controllore
%
candKP = 10;
candKI = 15;
candKD = 2;

vpa(subs(tableW0, [KP, KI, KD], - 1 * [candKP, candKI, candKD]), 3)

s = tf('s');
Ctrl0 =  - (candKI / s + candKP + candKD * s / (s / N + 1));
%%
% applico un guadagno di $5.35$ che porti i poli più veloci quando più
% possibile verso il semipiano sinistro
Loop0 = Ctrl0 * linearP;
Wcls0 = drawEverythingButSignalResponse(Loop0);

[numCtrl0, denCtrl0] = tfdata(Ctrl0);
numCtrl0 = cell2mat(numCtrl0);denCtrl0 = cell2mat(denCtrl0);
%%
%
% Per aumentare ancora di più il margine di fase ntroduco una rete
% anticipatrice con polo in $-15$ e zero in $3.64$
%
Ctrl1 = 5.36 * Ctrl0;

Loop1 = Ctrl1 * linearP;
Wcls1 = drawEverythingButSignalResponse(Loop1);

[numCtrl1, denCtrl1] = tfdata(Ctrl1);
numCtrl1 = cell2mat(numCtrl1);denCtrl1 = cell2mat(denCtrl1);

CtrlFinal = Ctrl1;
numCtrlFinal = numCtrl1;
denCtrlFinal = denCtrl1;
%%
%
% Ora il sistema a ciclo chiuso è asintoticamente stabile (vedi tabella di
% Routh), con funzione di trasferimento propria, sovraelongazione del $16\%$ e
% tempo d'assestamento al $5\%$ pari a $.86$ secondi: per migliorare
% ulteriormente le prestazioni inserisco un filtro al riferimento
%
tau = 3 / .893;
CtrlRef = 1 / (s / tau + 1);

WclsRef = CtrlRef * Wcls1;
step(WclsRef)

[numCtrlRef, denCtrlRef] = tfdata(CtrlRef);
numCtrlRef = cell2mat(numCtrlRef);
denCtrlRef = cell2mat(denCtrlRef);
%%
%
% Tento, tramite un controllore in feed forward, di rimpiazzare poli
% stabili del sistema a ciclo chiuso con altri poli più veloci:
% innanzitutto calcolo i poli effettivi del sistema a ciclo chiuso
%
[temp, rootslinearP] = tfdata(linearP);
rootslinearP = roots(cell2mat(rootslinearP));

Ffctrl = 1;
for i = (1 : 1 : length(rootslinearP))
    r = real(rootslinearP(i));
    if(r > -100 && r < 0)
        Ffctrl = Ffctrl * (s / (rootslinearP(i))  - 1) / (s / 75 + 1);
    end
end

Ffctrl = minreal(Ffctrl);
[numFfctrl, denFfctrl] = tfdata(Ffctrl);
numFfctrl = cell2mat(numFfctrl);
denFfctrl = cell2mat(denFfctrl);
%%
%
% Stampo a schermo la versione finale della funzione di trasferimento del
% sistema a ciclo chiuso, verifico stabilità di quest ultimo e 
% grafico la risposta a scalino del sistema
%
WclsEverything = minreal((linearP* Ffctrl + linearP * CtrlFinal * CtrlRef) ...
    / (1 + linearP * CtrlFinal))
[temp, WclsEverythingden] = tfdata(WclsEverything);
vpa(myRouth(cell2mat(WclsEverythingden)), 3)

figure(1);
step(WclsEverything);
%%
%
% Per i parametri nel transitorio eseguo |margin| sull'ultima versione della
% funzione d'anello (|Loop1|)
%

[gainMarg, phaseMarg, gainPuls, phasePuls] = margin(Loop1)
maxDelay = deg2rad(phaseMarg) / phasePuls
%%
%
% Infine avvio la simulazione dal modello Simulink
%
initAngle = pi;
finalAngle = 2 * pi;
delayVal = 0.02;
noisePower = 10;
noiseFrequency = 100;
d0 = 1;
d1 = 1; 
forceSaturation = 100;
load_system("myPendulum.slx");
set_param('myPendulum', 'StopTime', '20');
open_system("myPendulum/Measurements' scoping/The scope")
sim('myPendulum');