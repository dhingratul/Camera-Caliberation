clear all; close all;
%% 1.1
P=[]
M_cordinate=[2 2 2;-2 2 2;-2 2 -2;2 2 -2;2 -2 2;-2 -2 2;-2 -2 -2; 2 -2 -2]
m_cordinate=[422 323;178 323;118 483;482 483;438 73;162 73;78 117;522 117]
M_cordinate=[M_cordinate ones(8,1)] %Homogenous
m_cordinate=[m_cordinate ones(8,1)] %Homogenous
plot(m_cordinate(:,1),m_cordinate(:,2),'O')
%% 1.2 & 1.3
for i=1:length(M_cordinate)
    compt=ComputeP(M_cordinate(i,:),m_cordinate(i,:));
    P=[P;compt]
end
disp(P);
%% 1.4
[U,S,V_new]=svd(P)
m=V_new(:,end)
M=[m(1:4)'; m(5:8)'; m(9:12)']%Projection Matrix
% M=reshape(m,[3,4])
disp(M)
%% 1.5
[U_cam,S_cam,V_cam]=svd(M)
Cam_Hcordinates=V_cam(:,4)
Cam_cordinates=[Cam_Hcordinates(1,1)/Cam_Hcordinates(4,1) Cam_Hcordinates(2,1)/Cam_Hcordinates(4,1) Cam_Hcordinates(3,1)/Cam_Hcordinates(4,1) 1]

%% 1.6
M_prime=M(1:3,1:3)
M_prime=M_prime./M_prime(3,3) %Normalize m33
%% 1.7
Cos_theta_x=M_prime(3,3)/sqrt(M_prime(3,3)^2+M_prime(3,2)^2)
Sin_theta_x=-M_prime(3,2)/sqrt(M_prime(3,3)^2+M_prime(3,2)^2)
theta_x=acos(Cos_theta_x) %In radians
disp(theta_x);
R_x=[1 0 0;...
    0 Cos_theta_x -Sin_theta_x;...
    0 Sin_theta_x Cos_theta_x]
disp(R_x);
N=M_prime*R_x
disp(N);
%% 1.8
Cos_theta_z=N(2,2)/sqrt(N(2,1)^2+N(2,2)^2)
Sin_theta_z=-N(2,1)/sqrt(N(2,1)^2+N(2,2)^2)
theta_z=acos(Cos_theta_z) %In radians
R_z=[Cos_theta_z -Sin_theta_z 0;...
    Sin_theta_z Cos_theta_z 0;...
    0 0 1]
%% 1.9
R=R_x*[1 0 0;0 1 0;0 0 1]*R_z
K_matrix=M_prime*R
K_matrix=K_matrix./K_matrix(3,3)
u0=K_matrix(1,3)
v0=K_matrix(2,3)
disp([u0 v0]) %Co-ordinates
fl=(K_matrix(1,1)+K_matrix(2,2))/2 %Average focal Length
disp(fl)