%% Compute Vertices based on Homographies
% Input: H-Matrix, i, j
% Output: V(i,j)
% Note: Run from camera2dcaliberation.m
function [v_ij]=compute_v_ij(H,i,j)
v_ij=[H(1,i)*H(1,j) , H(1,i)*H(2,j)+H(2,i)*H(1,j) , H(2,i)*H(2,j) ,...
    H(3,i)*H(1,j)+H(1,i)*H(3,j) , H(3,i)*H(2,j)+H(2,i)*H(3,j) , H(3,i)*H(3,j)]';
end