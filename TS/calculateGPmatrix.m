function [G, P] = calculateGPmatrix(varargin)
    % This function takes at most 3 parameters:
    % - a matrix A
    % - a matrix B
    % - optionally an integer n
    % and calculates both the reachability in n steps matrix Pn(A, B)
    % and it's symmetric square equivalent G = P * P' for the minimum energy problem
    % If provided with only one parameter the function will interpret it as
    % the P matrix
    
    [~, temp] = size(varargin);
    
    if(temp == 1), P = varargin{1};
    else, P = calculatePmatrix(varargin{1 : temp});
    end
    
    G = P * P';
end
