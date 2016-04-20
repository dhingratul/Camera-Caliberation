addpath(genpath('/home/dhingratul/Dropbox/Academics/Spring2016/CS-534/HW/HW3/assignment3files'))
%% 2.1
M_cordinate=[0 210 1;270 210 1;270 0 1;0,0 1]
M_cordinate=M_cordinate'

im2=imread('images2.png');
imshow(im2)
[x,y]=ginput(4)
m_cordinate_1=[x y ones(4,1)]
m_cordinate_1=m_cordinate_1'
H_2=homography2d(M_cordinate,m_cordinate_1);
H_2=H_2/H_2(3,3)

im9=imread('images9.png');
imshow(im9)
[x,y]=ginput(4)
m_cordinate_9=[x y ones(4,1)]
m_cordinate_9=m_cordinate_9'
H_9=homography2d(M_cordinate,m_cordinate_9);
H_9=H_9/H_9(3,3)

im12=imread('images12.png');
imshow(im12)
[x,y]=ginput(4)
m_cordinate_12=[x y ones(4,1)]
m_cordinate_12=m_cordinate_12'
H_12=homography2d(M_cordinate,m_cordinate_12);
H_12=H_12/H_12(3,3)

im20=imread('images20.png');
imshow(im20)
[x,y]=ginput(4)
m_cordinate_20=[x y ones(4,1)]
m_cordinate_20=m_cordinate_20'
H_20=homography2d(M_cordinate,m_cordinate_20);
H_20=H_20/H_20(3,3)

%% 2.2
v_1=[compute_v_ij(H_2,1,2)'; (compute_v_ij(H_2,1,1)-compute_v_ij(H_2,2,2))']%im2
v_2=[compute_v_ij(H_9,1,2)'; (compute_v_ij(H_9,1,1)-compute_v_ij(H_9,2,2))']%im9
v_3=[compute_v_ij(H_12,1,2)'; (compute_v_ij(H_12,1,1)-compute_v_ij(H_12,2,2))']%im12
v_4=[compute_v_ij(H_20,1,2)'; (compute_v_ij(H_20,1,1)-compute_v_ij(H_20,2,2))']%im20
V=[v_1;v_2;v_3;v_4]
[U,S,Val]=svd(V)
b=Val(:,end)'
B11=b(1), B12=b(2), B22=b(3), B13=b(4), B23=b(5), B33=b(6)
B=[B11 B12 B13; B12 B22 B23;B13 B23 B33]
v_0=(B12*B13-B11*B23)/(B11*B22-B12^2)
lambda=B33-(B13^2+v_0*(B12*B13-B11*B23))/B11
alpha=sqrt(lambda/B11)
beta=sqrt(lambda*B11/(B11*B22-B12^2))
gamma=-B12*alpha^2*beta/lambda
u_0=gamma*v_0/alpha-B13*alpha^2/lambda
A = [alpha, gamma, u_0; 0, beta, v_0; 0, 0, 1]
%R and t Matrix computation
[R_2,t_2]=computeRT(H_2,A)
R_2*R_2' %Not an identity matrix
[R_9,t_9]=computeRT(H_9,A)
[R_12,t_12]=computeRT(H_12,A)
[R_20,t_20]=computeRT(H_20,A)
%Alternate method
[U,S,Value]=svd(R_2)
R_better=U*Value;
R_better*R_better' %Enforce identity matrix
%% 2.3.1
M_all=[];
for i=0:9
    for j=0:7
        M_all=[M_all;i*30 j*30 1];
    end
end
m_approx=approxCord(H_2,M_all);
imshow(im2);
hold on
plot(m_approx(1,:),m_approx(2,:),'bo');
hold off
%% 2.3.2
sigma = 2;thresh = 500;radius = 2;
[cim,r,c,rsubp,csubp]=harris(rgb2gray(im2),sigma,thresh,radius,1);
title('Figure 2 : Harris corners for im2');