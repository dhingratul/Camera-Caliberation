%% Comput R and T parametes
% Input: H-Matrix, K-Matrix
% Output: R, t
% Note: Run from camera2dcaliberation
function[R,t]=computeRT(H,A)
h1=H(:,1); h2=H(:,2); h3=H(:,3);
lambda=1/norm(inv(A)*h1);
r1 = lambda*inv(A)*h1;
lambda=1/norm(inv(A)*h2);
r2 = lambda*inv(A)*h2;
r3 = cross(r1,r2);
t = lambda*inv(A)*h3;
R = [r1, r3, r2];
end