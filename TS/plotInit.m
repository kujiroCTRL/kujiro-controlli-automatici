
function [A, B, x] = plotInit(l1, l2, ph)
% This function simply returns the A and B matrix
% (A being a square diagonal matrix with eigen values l1 and l2) as well as
% a state vector on a unit circle with phase equal to ph
    A = ...
        [
            l1  0   ;
            0   l2  ;
        ];
    B = [1; 1];
    
    ph = deg2rad(ph);
    x = [cos(ph); sin(ph)];
end
