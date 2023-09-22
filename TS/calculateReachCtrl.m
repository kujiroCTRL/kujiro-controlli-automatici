function [u, J, w] = calculateReachCtrl(varargin)
    % This function can operate in 2 distinct ways:
    % - if the number of parameters is 3 then it will interpret them as in
    % calculatePMatrix (first two are matrices A, B and the third is the reached state)
    % - if the number of parameters is 4 then it will interpret them the
    % following way
    % -- the first will be the matrix A
    % -- the second the matrix B
    % -- the third the number of steps n required to reach the state
    % -- the fourth the state to be reached
    % In both cases the function will return:
    % - the control vector u in the time interval from 0 to n - 1
    % - the w array (especially useful to calculate the state variable as time varies)
    % - the associated cost index (it's norm)
    [~, temp] = size(varargin);
    
    if(temp ~= 3 && temp ~= 4), error("Expected 3 or 4 parameters!");
    end

    [G, P] = calculateGPmatrix(varargin{1 : temp - 1});
    x = varargin{temp};
    
    v = invImg(x, G);
    w = P' * v;
    
    [rowP, colP] = size(P);
    n = rowP;
   	
    if(temp == 4), nu = varargin{3};
    else, nu = n;
    end

    p = floor(colP / nu);
    
    u = zeros(p, nu);
    J = w' * w;
    
	% To reorder w's elements, thus obtaining the control vector u(.), we can't use the flip() function: that's because the u(.) vector is not a simple reflection of w's entries but a reflection of a collection of blocks (of size p) along the first dimension
	for i = (0 : 1 : nu - 1)
        u(1 : p, i + 1) = w((nu - i - 1) * p + 1 : (nu - i) * p, 1);
    end
end
