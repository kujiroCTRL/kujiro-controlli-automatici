## OBS del sistema
Affinché il sistema sia OBS dovremmo avere $\operatorname{rank} Q=n$
- $Q(A,C)=[\mathbf r_1,\mathbf r_2, \mathbf r_3, \mathbf r_4]^\top$
	$$
	\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ -\frac{k}{m_{1}} & \frac{k}{m_{1}} & -\frac{c}{m_{1}} & \frac{c}{m_{1}}\\ \frac{c\,k}{{m_{1}}^2}+\frac{c\,k}{m_{1}\,m_{2}} & -\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}} & \frac{c^2}{{m_{1}}^2}-\frac{k}{m_{1}}+\frac{c^2}{m_{1}\,m_{2}} & \frac{k}{m_{1}}-\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}} \end{array}\right)
	$$
- ${\mathbf r}_3\leftarrow-\frac{m_1}k{\mathbf r}_3$
	$$
	\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ 1 & -1 & \frac{c}{k} & -\frac{c}{k}\\ \frac{c\,k}{{m_{1}}^2}+\frac{c\,k}{m_{1}\,m_{2}} & -\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}} & \frac{c^2}{{m_{1}}^2}-\frac{k}{m_{1}}+\frac{c^2}{m_{1}\,m_{2}} & \frac{k}{m_{1}}-\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}} \end{array}\right)
	$$
- ${\mathbf r}_3\leftarrow-(\mathbf r_3-\mathbf r_1-\frac ck\mathbf r_2)$ 
	$$
	\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0&1&0&\frac ck\\ \frac{c\,k}{{m_{1}}^2}+\frac{c\,k}{m_{1}\,m_{2}} & -\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}} & \frac{c^2}{{m_{1}}^2}-\frac{k}{m_{1}}+\frac{c^2}{m_{1}\,m_{2}} & \frac{k}{m_{1}}-\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}} \end{array}\right)
	$$
- $\mathbf r_4\leftarrow\mathbf r_4-\alpha_1\mathbf r_1-\alpha_2\mathbf r_2$ 
	$$
	\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0&1&0&\frac ck\\ 0& -\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}} & 0 & \frac{k}{m_{1}}-\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}} \end{array}\right)
	$$dove $\alpha_1$ e $\alpha_2$ sono gli elementi della riga 4 sulla colonna 1, 3 rispettivamente
- $\mathbf r_3\leftarrow\left(-\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}}\right)\mathbf r_3$
	$$\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0&-\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}}&0& -\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}}\\ 0& -\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}} & 0 & \frac{k}{m_{1}}-\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}} \end{array}\right)
	$$
- $\mathbf r_4\leftarrow\mathbf r_4-\mathbf r_3$
	$$\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0&-\frac{c\,k}{{m_{1}}^2}-\frac{c\,k}{m_{1}\,m_{2}}&0& -\frac{c^2}{{m_{1}}^2}-\frac{c^2}{m_{1}\,m_{2}}\\ 0& 0 & 0 & \frac{k}{m_{1}}\end{array}\right)
	$$
Invertendo la penultima operazione svolta risulta chiaro che $\operatorname{rank} Q=4$, quindi il sistema è OBS (il procedimento è valido solo se $ck\neq 0$ quindi se la molla e l'attrito effettivamente agissero nel sistema)
Nello specifico $k=0,c\neq 0\implies\operatorname{rank}Q=3$ mentre $k=0=c\implies\operatorname{rank} Q=2$

## Sottospazio osservabile
Vediamo ora, nei casi degeneri $k=0$ e $k=c=0$, quale sia lo spazio osservabile (equivalentemente non osservabile) del sistema
$$
	\operatorname{ker}\{Q\}=\left\{x=\begin{bmatrix}q_1\\q_2\\\dot{q_1}\\\dot{q_2}\end{bmatrix}\in\mathbb R^n:Qx=0\right\}
$$
Per ispezione visiva le componenti 1 e 3 di tali $x$ saranno 0
Nel caso $k=0$ la componente 2 è libera mentre la 4 sarà uguale alla 3 (quindi 0)
Nel caso $k=c=0$ le componenti 2 e 4 saranno entrambe libere
Complessivamente
$$
\begin{align*}
	k=0&\implies\mathcal X_{!\operatorname{OBS}}=\left\langle\begin{bmatrix}0\\1\\0\\0\end{bmatrix}\right\rangle\\
	k=c=0&\implies\mathcal X_{!\operatorname{OBS}}=\left\langle\begin{bmatrix}0\\1\\0\\0\end{bmatrix},\begin{bmatrix}0\\0\\0\\1\end{bmatrix}\right\rangle
\end{align*}
$$

## Autovalori del sistema
$$
	p_A(\lambda)=\lambda^2(\lambda^2+cM\lambda+kM)
$$dove $M=\frac{m_1+m_2}{m_1m_2}$, $c,k\geq 0$, $M>0$ (in queste condizioni il sistema è stabile, degenera nella stabilità semplice)
$$
	\sigma\{A\}=\left\{0,0,-\frac{cM}2\pm\frac12\sqrt{c^2M^2-4kM}\right\}
$$
1) Autovalori immaginari se $cM=0\implies c=0$
	Per esempio $m_1=3,m_2=7,k=2.1$ porta gli autovalori a $\pm\jmath$  
2) Autovalori complessi coniugati se $c^2M<4k$
	Per esempio volendo le parti reali a $-0.1$, $cM=0.2$ e parti immaginarie a $\pm 2$, $0.04-4kM=-16\leadsto kM=4.01$
	Prendendo i valori $m_1=3,m_2=7$ di prima dovremmo imporre $c=0.42,k=8.421$ 
3) Autovalori reali distinti se $c^2M>4k$
	Per esempio potremmo scegliere $cM=2\leadsto m_1=3,m_2=7,c=4.2$ e $k$ in modo che $c^2M^2-4kM$ sia un quadrato perfetto, per esempio $4kM=3\leadsto k=\frac3{4M}=\frac{63}{40}$
	In tal caso avremmo autovalori in $-\frac32,-\frac12$
In tutti questi casi (e in realtà ogni caso in cui $k\neq 0$) il sistema è OBS e DTB