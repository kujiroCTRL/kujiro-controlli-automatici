function [v, d] = invImg(x, G)
    % This function takes 2 parameters:
    % - a vector x
    % - a matrix P
    % and returns the calculated vector v (if any) such that
    % P * v = x, and the norm of the difference between x
    % and the image of v under P
     
    [rowx, ~] = size(x);
    [rowP, ~] = size(G);
    
    % If P is an nxm matrix, it's pseudo inverse will be an mxn matrix,
    % thus the number of rows of P must be equal to the number of rows
    % of x
    if(rowx ~= rowP), error("First and second parameter's sizes are inappropriate!");
    end

	% mldivide() also warns you whether the result is inaccurate (up to a fixed tolerance) but it's less accurate than pinv()
    % v = G\x; 
    v = pinv(G) * x;
    d = norm(G * v - x);
	     
	if(d > 1e-10)
		warning("Expected and actual state differ by " + d + " in magnitude");
	end
end
