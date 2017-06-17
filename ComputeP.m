%% Compute P-Matrix
% Computes the Projection matrix
% Input: M, m
% Output: PMatrix
% Note: Run from Inside camera3dcaliberation.m
function [PMatrix]=ComputeP(M,m)
PMatrix=[M 0 0 0 0 -m(1,1)*M ;
    0 0 0 0 M -m(1,2)*M ];
end