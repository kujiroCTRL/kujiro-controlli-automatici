%% Impostazione delle matrici A, B e la gramiana G
clear
close
syms g1 g2 t;

gamma = [g1, g2];
Amat = [0, 1; g1, g2];
Bmat = [0; 1];

Grm = GrmMat(Amat, Bmat, t);
%% Matrice gramiana nel caso generale
temp = fopen("gramian.md", "w+");

oldString = ["g_{1}", "g_{2}", "\,", "\"];
newString = ["\gamma_1", "\gamma_2", " ", "\\"];
outString = latex(Grm);

for i = (1 : length(oldString))
    outString = strrep(outString, oldString(i), newString(i));
end

outString = "$$" + outString + "$$\n";
fprintf(temp, outString);
fclose(temp);
%% Scelta dei valori di g1, g2
gammaVals = ...
{ ...   
    [- 2 / 100, 3 / 10], ...
    [- 1, - 2], ...
     [- 4, 0], ...
};

AmatCurr = {zeros(length(gammaVals))};
GrmCurr = {zeros(length(gammaVals))};
eigenCurr = {zeros(length(gammaVals))};
%% Stampa a file le matrici G
for i = (1 : length(gammaVals))
    AmatCurr{i} = subs(Amat, gamma, gammaVals{i});
    GrmCurr{i} = GrmMat(AmatCurr{i}, Bmat, t);

    temp = fopen("gramian" + i + ".md", "w");
    
    fprintf(temp, "$$\\gamma_1 = " ...
        + (gammaVals{i}(1)) + ", \\gamma_2 = " ...
        + (gammaVals{i}(2))...
        + "$$");
    
    eigenCurr{i} = transpose(eig(AmatCurr{i}));
    
    outString = "$$\sigma_1 = \left(" ...
        + latex(real(eigenCurr{i}(1))) + "\right)+\left(" + latex(imag(eigenCurr{i}(1))) + "\right)\jmath" ...
        + ", \sigma_2 = \left(" ...
        + latex(real(eigenCurr{i}(2))) + "\right)+\left(" + latex(imag(eigenCurr{i}(2))) + "\right)\jmath" ...
        + "$$";
    outString = strrep(outString, "\", "\\");
    
    fprintf(temp, outString);
    
    outString = latex(GrmCurr{i});
    outString = strrep(outString, "\", "\\");
    outString = "$$G_{\\mathcal S}(t) = " + outString + "$$";
    
    fprintf(temp, outString);
    fclose(temp);
end
%% Risposta forzata nota la funzione di controllo
timeFinal = 5;
eigCurr = 1;

AmatChose = double(AmatCurr{eigCurr});
GrmChose = GrmCurr{eigCurr};
uChose = @(t) sin(t);

[xChose1, timeSimulChose1, XChose1] = simResp( ...
    AmatChose, ...
    Bmat, ...
    uChose, ...
    [0, timeFinal]);

plotSlave(xChose1, timeSimulChose1, XChose1, timeFinal);
%% Controllo a energia minima
timeInteg = .01;
timeSample = 2;
timeLast = 16;

intervalInteg = @(Ta) (0 : timeInteg : Ta - timeInteg); 
intervalSample = (timeSample : timeSample : timeLast); 

temp = length(intervalSample);
GrmSimul = zeros(2, 2 * temp);
vSimul = zeros(2, temp);
uSimul = zeros(temp, length(intervalInteg(timeLast)));
JSimul = zeros(1, temp);

for i = (1 : 1 : temp)
    GrmSimul(:, i : i + 1) = subs(GrmChose, t, intervalSample(i));
    vSimul(:, i) = GrmSimul(:, i : i + 1) \ XChose1;
    JSimul(i) = 0;

    intervalCurr = intervalInteg(intervalSample(i));
    for j = (1 : 1 : length(intervalCurr))
        uSimul(i, j) = transpose(expm(AmatChose * (intervalSample(i) - intervalCurr(j))) * Bmat) * vSimul(:, i);
        JSimul(i) = JSimul(i) + uSimul(i, j) ^ 2 * timeInteg;
    end
end
%% Grafici dell'indice di costo minimo
figure();

JAxes = axes(); 
hold(JAxes, "on");
grid(JAxes, "on");
legend(JAxes, "on");

JChose = double(int(abs(uChose(t)), t, 0, timeFinal));
plot(JAxes, ...
    intervalSample, ...
    ones(1, length(intervalSample)) .* JChose, ...
    "DisplayName", "J(u_0)", "LineStyle", "--");

plot(JAxes, intervalSample, JSimul, "DisplayName", "J^\ast @T");

xlabel("T");
ylabel("J(u)");
%% Grafici del controllo a energia minima
figure();
uAxes1 = axes();
xlabel("\tau");
ylabel("u(\tau)");

hold(uAxes1, "on");
grid(uAxes1, "on");
legend(uAxes1, "on");

intervalChose = intervalInteg(intervalSample(end));
for j = (1 : 1 : length(intervalSample))
    plot(uAxes1, ...
        intervalChose, ...
        uSimul(j, :), ...
        "DisplayName", ...
        "u^\ast @ T = " + intervalSample(j));
end
u0 = arrayfun(uChose, intervalChose);
plot(uAxes1, intervalChose, u0, "DisplayName", "u_0", "LineStyle", "--");
%% Grafico di un specifico controllo ottimo
figure();
uAxes2 = axes();

uSingle = 10;
plot(uAxes2, intervalChose, ...
    uSimul(intervalSample == uSingle, :));

grid(uAxes2, "on");
xlabel("\tau");
ylabel("u^\ast(\tau)@ T=" + uSingle);
%% Grafici della variabile di stato
uSingle = find(intervalSample == 10);
TIN = intervalInteg(intervalSample(uSingle));

[xChose2, timeSimulChose2, XChose2] = simResp(AmatChose, Bmat, uSimul(uSingle, :), [TIN, TIN(end) + timeInteg]);

plotSlave(xChose2, timeSimulChose2, double(XChose1), timeSimulChose2(end));
%% RE: calcolo esatto del controllo ottimo
fprintf("A = ")
disp(AmatChose);
fprintf("B = ")
disp(Bmat);
fprintf("G(t) = ")
disp(GrmChose);
fprintf("x = ")
disp(XChose1);

v_RE = GrmChose \ XChose1;
u_RE = @(t1, t2) (transpose(expm(AmatChose * (t1 - t2)) * Bmat) * subs(v_RE, t, t1));

T_RE = 10;
u_REChose1 = simplify(u_RE(T_RE, t));
T_Plot = intervalInteg(T_RE);
u_REPlot = zeros(1, length(T_Plot));

fprintf("u^{\\ast}(t) = ")
disp(u_RE(T_RE, t));

figure();
ax_RE = axes();
for i = (1 : 1 : length(T_Plot))
    try
        u_REPlot(i) = subs(u_REChose1, t, T_Plot(i));
    catch ERR
        u_REPlot(i) = 10e10;
    end
end
plot(ax_RE, ...
    intervalInteg(T_RE), ...
    double(u_REPlot(:)) ...
    );
grid(ax_RE, "on");
%% RE: calcolo esatto dello stato
u_REChose2 = @(t) u_RE(T_RE, t);
[x_RE, t_RE, X_RE] = simResp(AmatChose, Bmat, u_REChose2, [0, T_RE]);

plotSlave(x_RE, t_RE, double(XChose1), timeSimulChose2(end));