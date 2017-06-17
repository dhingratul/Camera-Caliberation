%% 2d Camera Caliberation
% Input: Select the corners of the image grid in counter-clockwise
% direction
% Output:
clear all; close all;
%% 2.1
M_cordinate=[0 210 1;270 210 1;270 0 1;0,0 1]
M_cordinate=M_cordinate'

im2=imread('images2.png');
imshow(im2)
[x,y]=ginput(4)
m_cordinate_2=[x y ones(4,1)]
m_cordinate_2=m_cordinate_2'
H_2=homography2d(M_cordinate,m_cordinate_2)
H_2=H_2/H_2(3,3)
disp(H_2);
im9=imread('images9.png');
imshow(im9)
[x,y]=ginput(4)
m_cordinate_9=[x y ones(4,1)]
m_cordinate_9=m_cordinate_9'
H_9=homography2d(M_cordinate,m_cordinate_9)
H_9=H_9/H_9(3,3)
disp(H_9);
im12=imread('images12.png');
imshow(im12)
[x,y]=ginput(4)
m_cordinate_12=[x y ones(4,1)]
m_cordinate_12=m_cordinate_12'
H_12=homography2d(M_cordinate,m_cordinate_12)
H_12=H_12/H_12(3,3)
disp(H_12);
im20=imread('images20.png');
imshow(im20)
[x,y]=ginput(4)
m_cordinate_20=[x y ones(4,1)]
m_cordinate_20=m_cordinate_20'
H_20=homography2d(M_cordinate,m_cordinate_20)
H_20=H_20/H_20(3,3)
disp(H_20);
%% 2.2
B=[];
v_2=[compute_v_ij(H_2,1,2)'; (compute_v_ij(H_2,1,1)-compute_v_ij(H_2,2,2))']%im2
v_9=[compute_v_ij(H_9,1,2)'; (compute_v_ij(H_9,1,1)-compute_v_ij(H_9,2,2))']%im9
v_12=[compute_v_ij(H_12,1,2)'; (compute_v_ij(H_12,1,1)-compute_v_ij(H_12,2,2))']%im12
v_20=[compute_v_ij(H_20,1,2)'; (compute_v_ij(H_20,1,1)-compute_v_ij(H_20,2,2))']%im20
V=[v_2;v_9;v_12;v_20]
[U,S,Val]=svd(V)
b=Val(:,end)'
B11=b(1,1); B12=b(1,2); B22=b(1,3); B13=b(1,4); B23=b(1,5); B33=b(1,6)
B=[B11 B12 B13;...
    B12 B22 B23;...
    B13 B23 B33]
disp(B);
v_0=(B12*B13-B11*B23)/(B11*B22-B12^2)
lambda=B33-(B13^2+v_0*(B12*B13-B11*B23))/B11
alpha=sqrt(lambda/B11)
beta=sqrt(lambda*B11/(B11*B22-B12^2))
gamma=-B12*alpha^2*beta/lambda
u_0=gamma*v_0/alpha-B13*alpha^2/lambda
K_Matrix = [alpha, gamma, u_0;...
    0, beta, v_0;...
    0, 0, 1]
%R and t Matrix computation
[R_2,t_2]=computeRT(H_2,K_Matrix)
R_2'*R_2 %Not an identity matrix
[R_9,t_9]=computeRT(H_9,K_Matrix)
R_9'*R_9 %Not an identity matrix
[R_12,t_12]=computeRT(H_12,K_Matrix)
R_12'*R_12 %Not an identity matrix
[R_20,t_20]=computeRT(H_20,K_Matrix)
R_20'*R_20 %Not an identity matrix
%Alternate method
R_better_2=AlternateR(R_2)
R_better_2'*R_better_2 %Enforce identity matrix
R_better_9=AlternateR(R_9)
R_better_9'*R_better_9 %Enforce identity matrix
R_better_12=AlternateR(R_12)
R_better_12'*R_better_12 %Enforce identity matrix
R_better_20=AlternateR(R_20)
R_better_20'*R_better_20 %Enforce identity matrix
%% 2.3.1
M_all=[]
for i=0:9
    for j=0:7
        M_all=[M_all;i*30 j*30 1]
    end
end
[V_im2,H_2_n,m_correct_2]=compute_V(im2,H_2,M_all)
disp(H_2_n);
[V_im9,H_9_n,m_correct_9]=compute_V(im9,H_9,M_all)
disp(H_9_n)
[V_im12,H_12_n,m_correct_12]=compute_V(im12,H_12,M_all)
disp(H_12_n)
[V_im20,H_20_n,m_correct_20]=compute_V(im20,H_20,M_all)
disp(H_20_n)
%2.3.5 continued
V=[V_im2;V_im9;V_im12;V_im20]
[U,S,Value]=svd(V)
b=Value(:,end)'
B11=b(1,1);B12=b(1,2);B22=b(1,3);B13=b(1,4);B23=b(1,5);B33=b(1,6);
B=[B11 B12 B13;...
    B12 B22 B23;...
    B12 B23 B33]
v0 = (B12*B13 - B11*B23)/(B11*B22 - B12^2)
lambda = B33 - (B13^2 + v0*(B12*B13-B11*B23))/B11
alpha = sqrt(lambda/B11)
beta = sqrt(lambda*B11/(B11*B22-B12^2))
gamma = (-B12*alpha^2*beta)/lambda
u0 = gamma*v0/alpha - (B13*alpha^2)/lambda
K_matrix = [alpha gamma u0;...
    0 beta v0;...
    0 0 1]
disp(K_matrix);
[R_2,t_2]=computeRT(H_2,K_matrix)
disp(R_2); disp(t_2);
[R_9,t_9]=computeRT(H_9,K_matrix)
disp(R_9); disp(t_9);
[R_12,t_12]=computeRT(H_12,K_matrix)
disp(R_12); disp(t_12);
[R_20,t_20]=computeRT(H_20,K_matrix)
disp(R_20); disp(t_20);
A=K_matrix;
% save Homographies H_2_n H_9_n H_12_n H_20_n A R_2 R_9 R_12 R_20 t_2 t_9 t_12 t_20
%% Error reprojection
[avg_err_2,total_err_2]=projectError(M_all,H_2_n,im2,m_correct_2)
disp(avg_err_2);
[avg_err_9,total_err_9]=projectError(M_all,H_9_n,im9,m_correct_9)
disp(avg_err_9);
[avg_err_12,total_err_12]=projectError(M_all,H_12_n,im12,m_correct_12)
disp(avg_err_12);
[avg_err_20,total_err_20]=projectError(M_all,H_20_n,im20,m_correct_20)
disp(avg_err_20);
%% Comparison
[avg_err_old_2,tot_err_old_2]=computeErrorOld(H_2,m_correct_2,M_all)
[avg_err_old_9,tot_err_old_9]=computeErrorOld(H_9,m_correct_9,M_all)
[avg_err_old_12,tot_err_old_12]=computeErrorOld(H_12,m_correct_12,M_all)
[avg_err_old_20,tot_err_old_20]=computeErrorOld(H_20,m_correct_20,M_all)