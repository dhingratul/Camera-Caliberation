function[m_approx]=approxCord(H,M_all)
m_approx=H*M_all';
for j=1:length(m_approx) % Normalize
    m_approx(:,j) = m_approx(:,j) / m_approx(3,j);
end
end