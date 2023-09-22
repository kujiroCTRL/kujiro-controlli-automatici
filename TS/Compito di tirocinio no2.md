Dato il sistema a tempo continuo:
$$
\dot x(t)=Ax(t)+Bu(t), x(t)\in\mathbb R^2,u(t)\in\mathbb R
$$
descritto dalle matrici
$$
A=\begin{bmatrix}0&1\\\gamma_1&\gamma_2\end{bmatrix},B=\begin{bmatrix}0\\1\end{bmatrix}
$$
Si scelgano alcune coppie di valori per $\gamma_1$ e $\gamma_2$; ottenendo casi diversi per lo spettro di $A$:
- due autovalori reali e distinti, entrambi negativi
- due autovalori reali e distinti, uno negativo e uno non negativo,
- due autovalori complessi coniugati...
1) Calcolare l'espressione della matrice Gramiana di raggiungibilità nei casi prescelti, eventualmente aiutandosi con programmi di calcolo simbolco quali Maxima, Mathematica o altro; occorre esprimere la Gramiana sotto forma reale, senza usare coefficienti complessi
2) Pe ciascuno dei casi individuati, scegliere un ingresso $u_0(\cdot)$, e trovare la risposta forzata $\bar x$ al tempo $\bar t$ a tale ingresso ($\bar x=\phi(\bar t, 0, u_0(\cdot)$). Eseguire questo passo direttamente tramite simulazione in Matlab, ottenendo così un valore approssimato per $\bar x$
3) Per ciascuno dei casi individuati, utilizzando l'espressione della matrice Gramiana calcolata al punto 1), per un insieme di tempi $\{T_1,T_2,\dots,T_m\}$ (che includa $\bar t$) calcolare il controllo $u_{T_i}^\ast(\cdot)$ che, partendo da $x(0)=0$, ottiene $x(T_i)=\bar x$, minimizzando l'indice di costo: $$J_{T_i}(u)=\int_0^{T_i}||u(\tau)||_2^2\text d\tau$$ È possibile eseguire questo passo numericamente in Matlab
4) Attraverso simulazione in Matlab, valutare l'effetto degli ingressi di controllo ottimi calcolati al punto 3), calcolando numericamente i corrispondenti valori dell'indice di costo. Visualizzare l'andamento di  $J_{T_i}(u^\ast)$, al variare di $T_i$ in $\{T_1,T_2,\dots,T_m\}$, confrontandolo con $J_{\bar t}(u_0)$. Visualizzare anche, per alcuni casi scelti, gli andamenti di $u^\ast(\cdot)$ e della corrispondente risposta nello stato $x^\ast(\cdot)$, nel tempo, ma anche nel piano degli stati, nell'intervallo $[0,T_i]$.
Nota: indipendentemente dai programmi software utilizzati, si richiede un risultato esatto solo per il punto 1).

### Osservazioni sul compito
Lo spettro di $A$ è costituito dalle radici complesse del suo polinomio caratteristico:
$$
	\begin{align*}
		p_A(s)&=s^2-\gamma_2s-\gamma_1\stackrel?=0\\
		\leadsto \sigma(A)&=\left\{\frac{\gamma_2\pm\sqrt{\gamma_2^2+4\gamma_1}}2\right\}
	\end{align*}
$$

![[desmos-graph.png]]

Nota: dal calcolo (simbolico, senza effettuare controlli) della matrice gramiana si vede che posti $\gamma=(\gamma_1,\gamma_2)$, gli elementi della matrice saranno ben definiti solo se
- $\gamma_1\ne 0$
- $\gamma_2\ne 0$
- $\gamma_2^2+4\gamma_1\ne0$
(vedi [[temp]])

Inoltre la matrice di raggiungibilità $P(A,B)$ è pari a$$P(A,B)=\left[\begin{array}{c:c}B&AB\end{array}\right]=\begin{bmatrix}0&1\\1&\gamma_2\end{bmatrix}$$ ed ha sempre rango pieno $\leadsto$ il sistema è raggiungibile