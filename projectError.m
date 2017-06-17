%% Computes Projection Error
% Input: M, H, im, m
% Output: Avg error, total error, re-projection error
% Note: Run from camera2dcaliberation.m
function[avg_err,total,err_reproject]=projectError(M_all,H,im,m_correct)
m_project=H*M_all';
for j=1:length(m_project)
    % Normalize
    m_project(:,j) = m_project(:,j) /m_project(3,j);
end
m_project = m_project';
err_x = [m_correct(:,1) m_project(:,1)];
err_y = [m_correct(:,2) m_project(:,2)];
figure,imshow(im); hold on;
plot(m_correct(:,1), m_correct(:,2),'bx');
plot(m_project(:,1), m_project(:,2), 'ro');
plot(err_x', err_y');
hold off
err_reproject = [err_x,err_y];
total = sum(sqrt((err_x(:,1)-err_x(:,2)).^2 + (err_y(:,1)-err_y(:,2)).^2));
avg_err = total/80;
end