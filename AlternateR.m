%% Alternate Method for Computing R
% Input: R-Matrix
% Output: Improved version of R-Matrix
% Note: Run from camera2dcaliberation.m
function[R_better]=AlternateR(R)
[U,S,Value]=svd(R);
R_better=U*Value;
R_better*R_better'; %Enforce identity matrix
end