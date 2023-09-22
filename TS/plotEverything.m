function Jvect = plotEverything(l1, l2, ph, numax)
    % This function will plot state and control variables that minimizes
    % the cost index where
    % - the A matrix has eigenvalues equal to l1 and l2
    % - the state variable to be reached is on the unit circle and has
    % phase equal to ph
    % - the maximum number of steps in which the system will have to reach
    % the desired state is numax
    % It also returns the array of minimum index costs calculated
    if(numax < 2)
        error("Fourth argument must be greater than 2");
    end
    figure();
    axx = axes;
    grid;
    hold(axx, "on")
    xlabel(axx, "First state variable [x_1]");
    ylabel(axx, "Second state variable [x_2]");
    title(axx, "State variable plots");

    p = (0 : 9 : 360);
    r = zeros(length(p), 2);
    for i = (1 : length(p))
	r(i, :) = [cos(deg2rad(p(i))), sin(deg2rad(p(i)))];
    end

    plot(axx, r(:, 1), r(:, 2));

    figure();
    axu = axes;
    grid;
    hold(axu, "on")
    xlabel(axu, "Simulation time [t]");
    ylabel(axu, "Control variable [u(t)]");
    title(axu, "Control variable plots");

    t = (1 : 1 : numax - 1);
    x = zeros(2, numax);

    u = zeros(numax - 1);
    lgnd = {zeros(numax + 1)};
    Jvect = zeros(1, numax - 1);
	
    lgnd{1} = "x_1^2+x_2^2=1";
    for i = t
        [x(1 : 2, 1 : i + 2), u(1 : i + 1), Jvect(i)] = calculateResponse(l1, l2, ph, i + 1);
        lgnd{i + 1} = "\nu=" + (i + 1);
        
        plot(axx, x(1, 1 : 1 : i + 2), x(2, 1 : 1 : i + 2), "Marker", "o");    
	plot(axu, (0 : 1 : i), u(1 : i + 1), "Marker", "o");
    end
    
    legend(axu, lgnd{2 : length(lgnd)});
    legend(axx, lgnd);
    
    figure();
    axj = axes;
    plot(axj, t + 1, Jvect, "Marker", "o");
    grid;
    xlabel(axj, "Number of steps [\nu]");
    ylabel(axj, "Minimum cost index [J_\nu]");
    title(axj, "Minimum cost index plot");
end
