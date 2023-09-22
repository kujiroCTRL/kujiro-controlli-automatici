function [x, t, X] = simResp(A, B, u, T)
	n = length(A);
    
    % Piccola nota sull'implementazione di simResp:
    % per come l'ho implementata io, la simulazione richiede l'uso della ode45 che, a
    % sua volta, necessita che il controllo sul sistema sia una funzione
    % Di fatto, se u fosse nota "in forma chiusa", MATLAB non si farebbe
    % problemi a calcolare la soluzione del sistema dinamico
    % Se, d'altro canto, il controllo fosse definito a punti (come se lo
    % avessimo campionato a intervalli costanti) MATLAB non riuscirebbe
    % a far corrispondere gli elementi del parametro "tspan" con i
    % corrispondenti valori del vettore di controllo nonostante
    % 1. "tspan" ed "u" combacino nel numero di elementi
    % 2. la comparazione tra i due gli viene esplicitamnete data
    % dall'utente definendo la funzione "U = @(t) u(tspan == t)" che, al
    % variare di "t" in "tspan", restituisce il valore del controllo al
    % tempo "t"
    % ("u(t)" non potrebbe mai funzionare in questo senso siccome
    % l'indicizzazione può avvenire solo con indici interi e maggiori di 0:
    % usando l'argomento "tspan == t" otteniamo quell'indice "i" per cui
    % "u(i) = tspan")
    if(isa(u, 'function_handle'))
        [t, x] = ... 
		ode45(...
	    @(t, x) A * x + B * double(u(t)),...
		T, zeros(n, 1));
        
        x = transpose(x);

        syms tau;
        n = expm(A * (T(end) - tau)) * B;
        X = double(int(n * u(tau), tau, T(1), T(end)));
    else
        x = zeros(n, length(T));
        t = T;
        dT = t(2) - t(1);
        for i = (2 : 1 : length(T))
            x(:, i) = x(:, i - 1) + dT * A * x(:, i - 1) + dT * B * u(i - 1);
        end
        X = x(:, end);
    end
end
