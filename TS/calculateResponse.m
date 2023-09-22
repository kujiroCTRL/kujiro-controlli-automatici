function [x, u, J] = calculateResponse(l1, l2, ph, nu)
    % This function will take as input:
    % - the eigen value for the A matrix, l1 and l2
    % - the pase of the desired state to be reached
    % - the maximum number of steps numax, for which the controll will be
    % calculated
    % and will return the array of state and control variables during the
    % simulation as well as the minimum cost index associated to that
    % control
    [A, B, x] = plotInit(l1, l2, ph);
    
    [u, J, w] = calculateReachCtrl(A, B, nu, x);
    x = zeros(2, nu + 1);
    
    for i = (2 : 1 : nu + 1)
    x(1 : 2, i) = A * x(1 : 2, i - 1) + B * u(i - 1);
    end
end
