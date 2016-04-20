%% 1.1
P=[];
 M_cordinate=[2 2 2;-2 2 2;-2 2 -2;2 2 -2;2 -2 2;-2 -2 2;-2 -2 -2; 2 -2 -2];
m_cordinate=[422 323;178 323;118 483;482 483;438 73;162 73;78 117;522 117];
M_cordinate=[M_cordinate ones(8,1)] %Homogenous
m_cordinate=[m_cordinate ones(8,1)] %Homogenous
%% 1.2 & 1.3
for i=1:length(M_cordinate)
    compt=ComputeP(M_cordinate(i,:),m_cordinate(i,:))
    P=[P;compt];
end
%% 1.4
[U,S,V]=svd(P);
m=V(:,end)
M=[m(1:4)'; m(5:8)'; m(9:12)']%Projection Matrix
% M=reshape(m,[3,4]) 
%% 1.5
[U_cam,S_cam,V_cam]=svd(M)
Cam_Hcordinates=V_cam(:,4)
Cam_cordinates=[Cam_Hcordinates(1,1)/Cam_Hcordinates(4,1) Cam_Hcordinates(2,1)/Cam_Hcordinates(4,1) Cam_Hcordinates(3,1)/Cam_Hcordinates(4,1)]
%% 1.6
M_prime=M(1:3,1:3)
scaling=M_prime(3,3)
M_prime=M_prime./scaling %Normalize m33
%% 1.7
Cos_theta_x=M_prime(3,3)/(sqrt(M_prime(3,3)^2+M_prime(3,2)^2))
Sin_theta_x=-M_prime(3,2)/(sqrt(M_prime(3,3)^2+M_prime(3,2)^2))
theta_x=acos(Cos_theta_x) %In radians
R_x=[1 0 0;0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)]
N=M_prime*R_x
theta_x=rad2deg(theta_x) %In degrees