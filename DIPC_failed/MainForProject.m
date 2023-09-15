%% Script per l'analisi e controllo di un doppio pendolo inverso su carrello mobile (sistema DPIC)
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
syms x0(t) t1(t) t2(t) m0 m1 m2 J1 J2 L1 L2 g
% Posizione del CoM del primo braccio
x1 = x0 + L1 / 2 * sin(t1);
y1 = - L1 / 2 * cos(t1);
% Posizione del CoM del secondo braccio
x2 = x1 + L1 / 2 * sin(t1) + L2 / 2 * sin(t2);
y2 = y1 - L1 / 2 * cos(t1) - L2 / 2 * cos(t2);
% Posizione terminale del DPI
xt = x2 + L2 / 2 * sin(t2);
yt = y2 - L2 / 2 * cos(t2);
% Calcolo delle velocità lineari
v0 = diff(x0);
vx1 = diff(x1);
vy1 = diff(y1);
vx2 = diff(x2);
vy2 = diff(y2);
% Calcolo delle velocità angolari
w1 = diff(t1);
w2 = diff(t2);
%%
%
% Calcolo del lagrangiano LGR = KE - PE e derivate parziali rispetto alle posizioni
% $x_0, \vartheta_1, \vartheta_2$ e alle velocità $v_0, \omega_1, \omega_2$
%   
KE = .5 * (m0 * v0 ^ 2 ...
    + m1 * vx1 ^ 2 + m1 * vy1 ^ 2 ...
    + m2  * vx2 ^ 2 + m2 * vy2 ^ 2 ... % aggiungi + J1 * w1 ^ 2 + J2 * w2 ^ 2 per includere anche i momenti
);
PE = - m1 * g * y1 - m2 * g * y2;
LGR = KE - PE
%%
%
% Rimpiazzo le accelerazioni degli stati con variabili simboliche (utile
% per semplificare l'espressione di queste ultime in termini delle altre
% variabili di stato)
%
syms a0 u1 u2 u(t);
ddQ = [a0, u1, u2];
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
    diff(diff(LGR, w2)) - diff(LGR, t2) ...
], [diff(v0), diff(w1), diff(w2)], ddQ);
F = simplify(F)
%%
%
% Scelgo come forze esterne agenti sul sistema
% la forza (attuabile) sul carrello $u(t)$ e le forze di disspazione che
% sono lineari nelle velocità $d_0v_0, d_1\omega_1, d_2\omega_2$
%
sys = F == [u, 0, 0]; % includere altre variabili simboliche per la dissipazione d'energia
% del carrello e delle astesyms d0 d1 d2;
%%
%
% Nota: il sistema è non lineare nelle variabili di stato
% $q=[x_0, \vartheta_1, \vartheta_2]$, tantomeno nelle
% variabili estese $Q = [x_0, \vartheta_1, \vartheta_2, v_0,
% \omega_1, \omega_2]$, ma può comunque essere espresso nella forma
%%
%
% $$ D(q) \cdot \ddot q + C(q) \cdot \dot q + G(q) = U $$
%
% dove $D, C, G$ sono matrici mentre $U = [u(t), 0, 0]$
%
%%
% 
% Per ottenere le matrici $D, C, G$ sostituisco i termini non lineari in
% $q,\dot q$ con nuove variabili simboliche |w1, w2, W1, W2, c1, s1, c2, s2|
% le ricavabili dalla seguente |subs|
%
syms w1(t) W1 w2(t) W2 c1 s1 c2 s2 cd sd;
sys = subs(sys, [ ...
    diff(t1, t) ^ 2, diff(t2, t) ^ 2, cos(t1), sin(t1), cos(t2), sin(t2), cos(t1 - t2), sin(t1 - t2)], ...
    [w1 * W1, w2 * W2, c1, s1, c2, s2, cd, sd] ...
    );
disp(transpose(sys));
%%
%
% Avendo fatto questa sostituzione possiamo usare la funzione
% |equationsToMatrix| per ottenere le matrici $D, C, G$
%%
%
% Per ottenere $D$ scegliamo variabili da linearizzare come
% prodotto di matrici $a_0, u_1, u_2$; per $C$
% scegliamo $x_0,\omega_1,\omega_2$ (non scegliamo $v_0$ in quanto comunque non
% compare nell'equazione e a quanto sembra MATLAB non riconosce
% |diff(x0(t), t)| come una variabile simbolica); mentre per $G$ scegliamo
% $g$ (per ispezione visiva i termini lineari in questo sistema modificato
% hanno in comune l'accelerazione gravitazionale $g$)
%
D = equationsToMatrix(sys, ddQ);
C = equationsToMatrix(sys, [x0(t), w1(t), w2(t)]);
G = g * equationsToMatrix(sys, g);
%%
%
% Volendo esprimere il sistema del secondo ordine dato da $D\ddot q+C\dot q+G=U$
% come un sistema del primo ordine possiamo svolgere innanzitutto risolvere
% l'equazione in termini di $\ddot q$:
%
%% 
% 
% $$ \ddot q = D^{-1} U -D^{-1} C \dot q - C^{-1} $$
% Definendo nuova variabile di stato $Q = [q, \dot q]$ il sistema può
% essere scritto come
%
%%
% 
% $$
% \dot Q = \left[\matrix{0& 1 \cr 0&-D^{-1} C}\right]Q +
% \left[\matrix{0&0\cr D^{-1}& -D^{-1} G}\right]\left[\matrix{U\cr1}\right] 
% $$
%
%%
% Per rendere matrici $D, G, C$ utilizzabili in Simulink possiamo
% sostituire loro variabili simboliche di partenza
%
D = subs(D, [ ...
    w1, W1, w2, W2, c1, s1, c2, s2, cd, sd...
    ], [ ...
    diff(t1, t), diff(t1, t), diff(t2, t), diff(t2, t), cos(t1), sin(t1), cos(t2), sin(t2), cos(t1 - t2), sin(t1 - t2)...
    ]);
C = subs(C, [ ...
    w1, W1, w2, W2, c1, s1, c2, s2, cd, sd...
    ], [ ...
    diff(t1, t), diff(t1, t), diff(t2, t), diff(t2, t), cos(t1), sin(t1), cos(t2), sin(t2), cos(t1 - t2), sin(t1 - t2)...
    ]);
G = subs(G, [ ...
    w1, W1, w2, W2, c1, s1, c2, s2, cd, sd...
    ], [ ...
    diff(t1, t), diff(t1, t), diff(t2, t), diff(t2, t), cos(t1), sin(t1), cos(t2), sin(t2), cos(t1 - t2), sin(t1 - t2)...
    ]);
%%
%
% Calcolo le matrici del sistema di primo grado nelle variabili $Q$
%
I = inv(D);
H = - I *  C;
J = - I * G;
A = [zeros(3, 3), eye(3); zeros(3, 3), H];
B = [zeros(3, 4); I, J];
%%
%
% Definisco i parametri per la simulazione de modello Simulink
%
states = [diff(x0(t), t), diff(t1(t), t), diff(t2(t), t), x0(t), t1(t), t2(t)];
constsSym = [m0, m1, m2, L1, L2, g]; %, J1, J2];
constsReal = [1.5, .5, .75, .5, .75, 9.81]; %1 / 3 * .5 * .5 ^ 2 , 1 / 3 * .75 * .75 ^ 2];
syms X0 T1 T2 V0 W1 W2;
newstates = [V0 W1 W2 X0 T1 T2];
functA = subs(A, [constsSym, states], [constsReal, newstates])
functB = subs(B, [constsSym, states], [constsReal, newstates])
%%
%
% Dalle espressioni così ottenute possiamo notare come $B$ dipenda solo da
% $\vartheta_1,\vartheta_2$, mentre $A$ solo da 
% $\vartheta_1,\vartheta_2, \omega_1,\omega_2$
% (nessuna delle due dipende dall'effettiva posizione
% o velocità del carrello)
%
%%
%
% Per la simulazione converto le matrici $A$ e $B$ in funzioni che
% memorizzo rispettivamente in |functA.m| e |functB.m|
% 
Q0 = zeros(6, 1);
Q0(2) = .01;
%%
% 
% Definisco quindi le condizioni
% iniziali del sistema (tutti sei i parametri di stato, anche se ai nostri interessi servono 
% solo gli angoli $\vartheta_1, \vartheta_2$ delle aste del pendolo)
%
matlabFunction(functA, 'File', 'functA.m');
matlabFunction(functB, 'File', 'functB.m');
%%
%
% Per linearizzare il sistema scelgo come primo punto di equilibrio la
% configurazione up-up delle aste, corrispondente quindi a
% $\vartheta_1=\vartheta_2=0$, posizione generica $x_0$ e velocità
% nulle $\omega_1=\omega_2=v_0=0$
%
%%
%
% Scelte le condizioni inziali posso usare la formula per l'approssimazione
% al primo ordine data da 
%
%%
%
% $$ f(x,u)\approx [\partial_x f(x,u)]_{x=x_0,u=u_0}\cdot
% (x-x_0)+[\partial_u f(x,u)]_{x=x_0,u=u_0}\cdot (u-u_0) $$
%
%%
%
% In realtà, dalla struttura di $A, B$ posso osservare che le uniche parti
% non lineari del sistema sono date dalle sottomatrici $A_{[4:6, 5:6]}$ di
% $A$ e $B_{[4:6,1:4]}$ di $B$ (usando una notazione simile a quella
% dell'indicizzazione in |MATLAB|)
%
%%
%
% Da questa osservazione, usando la formula sopra riportata per le sole
% dinamiche non lineari del sistema e svolgendo tutti i calcoli otteniamo 
% l'approssimazione lineare
%
%%
%
% $$
%   \dot Q_i\approx\left[ -(\partial_{Q_i} D^{-1}) C Q - D^{-1}(\partial_{Q_i} C)
%   Q -D^{-1} C (\partial_{Q_i} Q) + (\partial_{Q_i}
%   D^{-1})U-(\partial_{Q_i} D^{-1})G - D^{-1}(\partial_{Q_i}
%   G)\right]_{Q=Q_0,U=U_0}(Q-Q_{0}) + \ast (U-U_0)
% $$
%
Qeq = Q0'
subs(diff(functA, V0), newstates, Qeq)
subs(diff(functA, W1), newstates, Qeq)
subs(diff(functA, W2), newstates, Qeq)
%%
%
% Notiamo che $\partial_{Q_i} D^{-1}$, all'equilibrio, è zero per ogni $i$
%
%%
% 
% Stesso discorso vale per $Q$ che, all'equilibrio, è il vettore di soli
% zeri e per $C$ (verificabile con una |subs| su |C|)
%
%%
%
% $$
%   \dot Q_i\approx\left[- D^{-1}(\partial_{Q_i} G)\right]_{Q=Q_0,U=U_0}Q
%   +\ast (U - U_0)
% $$
%
%%
% 
% Definisco quindi la matrice $A$ per il sistema linearizzato
%
%m = subs(I, [states, constsSym], [Qeq, constsReal])
m = subs(I, [states], [Qeq])

%l = subs(jacobian(G, [x0(t), t1(t), t2(t)]), [states, constsSym], [Qeq, constsReal])
l = subs(jacobian(G, [x0(t), t1(t), t2(t)]), [states], [Qeq])
linA = [ ...
    zeros(3, 3), eye(3, 3); ...
    - m * l, zeros(3, 3)
]
%%
%
% Per quanto riguarda $B$ effettuo gli stessi calcoli prendendo derivate
% parziali rispetto ad $U$ invece di $Q_i$
%
%%
%
% Siccome l'unico termine a moltiplicare $U$ è $D^{-1}$, la matrice $B$
% sarà proprio $D^{-1}$ valutata in $Q_0$
%
linB = [zeros(3, 1); m * [1; 0; 0]]
%%
%
% Le matrici $C,D$ le scelgo come la matrice della sola componente $\vartheta_2$
% e 0 (non misuro l'ingresso)
%
linC = [0, 1, zeros(1, 4)]
linD = 0
%%
%
% Converto le matrici del sistema linearizzato nella funzione di
% trasferimento dell'impianto, usando innanzitutto |ss| seguito da |tf|
%
%linPtf = tf(ss(linA, linB, linC, linD))
%[linPnum, linPden] = tfdata(linPtf);

%drawEverythingButSignalResponse(linPtf)
%%
%
% Voglio che il sistema risponda ad errore nullo a riferimenti costanti
% ed overshoot e settling time più piccoli possibili
%
%%
%
% $$
%   0=\lim_{t\rightarrow+\infty} |e(t)|=\lim_{s\rightarrow+0}\left|s\frac{s^\rho D_CD_P}{s^\rho D_CD_P+\mu
%   N_CN_P}\frac{R_0}{s}\right|=\lim_{s\rightarrow+0}\left|\frac{R_0(s^\rho \mu_{D_P})}{\mu\mu_{N_P}}\right|
% $$
%
%%
% %
% % Affinché il limite sopra riportato sia zero scelgo di inserire un polo
% % all'origine nel controllore e, a questo punto, opto per un controllore
% % PID con un polo all'origine ($K_I\neq 0$)
% %
% S = tf('s');
% syms s KP KI KD;
% 
% numC0 = KD * s^2 + KI + KP * s;
% denC0 = s;
% 
% [numP, denP] = tfdata(linPtf);
% numP = poly2sym(cell2sym(numP), s);
% denP = poly2sym(cell2sym(denP), s);
% 
% denW0 = expand(numC0 * numP + denC0 * denP);
% denW0 = coeffs(denW0, s);
% resExpr = myRouth(denW0);
% vpa(resExpr, 3)
% %%
% %
% % Dalla tabella di Routh possiamo osservare che affinché la prima colonna
% % sia positiva dovremmo avere $K_I>0$, $K_P> 468 / 17.4\approx 27$. Per ora
% % scelgo $K_P = 30, K_I=1$
% %
% resExpr = subs(resExpr, [KP, KI], [30, 1]);
% vpa(resExpr, 3)
% %%
% %
% % Per il terzo vincolo basta scegliere $K_D > - 0.0250$: scegliendo $K_D>0$
% % andremmo in contraddizione con il quarto vincolo quindi dobbiamo
% % scegliere un valore di $K_D<0$. Scelgo $K_D = -0.016$ che, pur non
% % stabilizzando il sistema a ciclo chiuso rende un solo elemento negativo
% %
% vpa(subs(resExpr, [KD], [-.016]), 3)
% KP = 30; KI = 1; KD = - .16;
% C0 = KP + KI / S + KD * S;
% L0 = - C0 * linPtf;
% 
% W0 = drawEverythingButSignalResponse(L0);
% %%
% %
% % Per rendere il sistema a ciclo chiuso asintoticamente stabile provo a
% % spostare
% %
%% 
%
% Inserisco un polo nell'origine per assicurare asservimento a regime di
% segnali a scalino
%
%s = tf('s');
%C0 = - 1 / s;

%L0 = C0 * linPtf;
%W0 = drawEverythingButSignalResponse(L0)
%%
%C1 = C0 * 1 / (s / 50 + 1)^2 * ((s + 1)^2 + 1)^2;

%L1 = C1 * linPtf;
%W1 = drawEverythingButSignalResponse(L1);