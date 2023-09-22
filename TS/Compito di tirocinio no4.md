### Spiegazioni del design del compensatore
$$
	\binom{\dot x}{\dot e}=\begin{pmatrix}A+BF&-BF\\&A-VC\end{pmatrix}\binom xe+\binom B\ r
$$
Dove $V$ l'abbiamo progettata come filtro di Kalmann (`lqr(transpose(A), transpose(C), ...)`) nel duale, $A,B,C$ son note $\leadsto$ rimarrebbe da progettare $F$
Affinché il sistema sia AS dobbiam imporrere gli autovalori di
$$
	\mathbf A=\begin{pmatrix}A+BF&-BF\\&A-VC\end{pmatrix}
$$
nel semipiano complesso sx
$$
	\sigma\{\mathbf A\}=\sigma\{A+BF\}\cup\sigma\{A-VC\}
$$
ove $\sigma\{A-VC\}$ effettivamente contiene soli autovalori AS (da design di $V$)
Per esempio potremmo progettare anche $F$ come filtro di Kalmann (`lqr(A, - B, ...)`))
Per poter usare queste scelte di $V, F$ in uno schema di controllo classico (retroazione dall'uscita dell'impianto con controllore dell'errore $\mathbf C$) scegliamo matrici $A,B,C,D$ del controllore come 
$$
	\mathbf C:\begin{cases}
		A=A-VC+BF-VDF\\
		B=-V\\
		C=F\\
		D=0
	\end{cases}
$$
## Inseguimento di segnali a rampa e sinusoidali
Osserviamo che il nostro impianto possiede:
- 2 poli nell'origine
- una coppia di zeri / poli stabili o asintoticamente stabili (due dei quali in $\pm\jmath\omega$)
Pertanto la specifica a regime di asservimento di segnali costanti o a rampa è già garantita
Affinché il sistema possa anche inseguire senza errore segnali sinusoidali di pulsazione $\Omega$, il controllore $\mathbf C$ dovrà avere dei poli in $\pm\jmath\Omega$
Sia quindi $\mathbf C_M=\frac1{s^2+\Omega^2}$ il controllore in catena diretta a $\mathbf C$ responsabile di rendere la dinamica d'inseguimento di segnali sinusoidali AS: una possibile realizzazione nello spazio di stato sarà data da
$$
	\mathbf C_M:\begin{cases}
	A=\begin{bmatrix} 0&\Omega^2\\1&0\end{bmatrix}\\
	B=[1,0,\dots,0]^\top \\
	C=[0,\dots,0,1] \\
	D=0\end{cases}
$$