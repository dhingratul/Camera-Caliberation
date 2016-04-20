function [v_ij]=compute_v_ij(H,i,j)
    v_ij=[H(1,i)*H(1,j) , H(1,i)*H(2,j)+H(2,i)*H(1,j) , H(2,i)*H(2,j) ,...
        H(3,i)*H(1,j)+H(1,i)*H(3,j) , H(3,i)*H(2,j)+H(2,i)*H(3,j) , H(3,i)*H(3,j)]';
end