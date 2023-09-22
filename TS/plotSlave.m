function plotSlave(x, ts, X, T)
    figure();
    subplot(2, 1, 1);
    xlabel("t");
    ylabel("x_1(t)");
    
    hold;
    plot(ts, x(1, :));
    plot(T, X(1), '*');
    grid;
    legend("x_1(t)", "x_1(" + ts(end) + ")");
    
    subplot(2, 1, 2);
    xlabel("t");
    ylabel("x_2(t)");
    
    hold;
    plot(ts, x(2, :));
    plot(T, X(2), '*');
    grid;
    legend("x_2(t)", "x_2(" + ts(end) + ")");
    
    figure();
    plot(x(1, :), x(2, :));
    xlabel("x_1");
    ylabel("x_2");
    hold;
    plot(X(1), X(2), "*");
    grid;
    legend("(x_1(t), x_2(t))", "(x_1(" + ts(end) + "), x_2(" + ts(end) + ")");
end