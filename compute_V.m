%% Compute the closest Harris corner to each approximate grid corner
% Input: Image, H-Matrix, M(co-ordinates)
% Output: V matrix, H_new, P-correct
% Note: Run from camera2dcaliberation
function[V,H_new,p_correct]=compute_V(im,H,M_all)
m_approximate=approxCord(H,M_all);
figure, imshow(im);
hold on
plot(m_approximate(1,:),m_approximate(2,:),'bo');
title('Figure 1: Projected grid corners');
hold off
%% 2.3.2
figure, imshow(im); hold on;
sigma = 2;thresh = 500;radius = 2;
[cim,r,c,rsubp,csubp]=harris(rgb2gray(im),sigma,thresh,radius,1);
title('Figure 2: Harris corners ');
hold off;
%% 2.3.3
Distance=dist2(m_approximate(1:2,:)',[csubp,rsubp]);
[D_sorted,D_index]=sort(Distance,2);
p_correct=[csubp(D_index(:,1)) rsubp(D_index(:,1)) ones((9+1)*(7+1),1)];
figure, imshow(im);
title('Figure 3 : grid points ')
hold on;
plot(p_correct(:,1),p_correct(:,2),'rx');
hold off
%% 2.3.4
H_new=homography2d(M_all',p_correct');
H_new=H_new/H_new(3,3); %Normalize

%% 2.3.5
v12=compute_v_ij(H_new,1,2);
v11=compute_v_ij(H_new,1,1);
v22=compute_v_ij(H_new,2,2);
V=[v12';(v11-v22)'];