function P = calculatePmatrix(varargin)
    % This function takes at most 3 parameters:
    % - a matrix A
    % - a matrix B
    % - optionally an integer n
    % and calculates the reachability in n steps-matrix Pn(A, B)
    
    [~, temp] = size(varargin);

    % There must be at least 2 parameters
    if(temp ~= 2 && temp ~= 3), error("Expected 2 or 3 parameters!");
    else
        A = varargin{1};
        
        B = varargin{2};
        
        [rowA, colA] = size(A);
        [rowB, colB] = size(B);

        % Matrices A and B don't have matching sizes
        if(rowA ~= colA), error("First parameter is not a square matrix!");
        elseif(colA ~= rowB), error("First and second parameter's sizes are inappropriate!");
        end
    end

    % If matrices A and B are well defined keep going
    if(temp == 3), n = int8(abs(varargin{3}));
    else, n = rowA;
    end
    
    % Define block P0
    P = zeros(rowA, n * colB);
    P(1 : rowA, 1 : colB) = B;
    temp = B;
    
    % Define and queue blocks Pi up to n - 1
    for i = (1 : 1 : n - 1)
       temp = A * temp;
        P(1 : rowA, i * colB  + 1: (i + 1) * colB) = temp;
    end
end