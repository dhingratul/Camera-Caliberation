%% Compute Error
% Note: Run from camera2dcaliberation.m
function[avg_err_reproject_old,tot_err_old]=computeErrorOld(H,p_correct,M_all)
project_old=H*M_all';
for i=1:length(project_old)
    project_old(:,i)=project_old(:,i)/project_old(3,i);
end
project_old=project_old';
err_x_old = [p_correct(:,1) project_old(:,1)];
err_y_old = [p_correct(:,2) project_old(:,2)];
err_reproject_old = [err_x_old,err_y_old];
tot_err_old = sum(sqrt((err_x_old(:,1)-err_x_old(:,2)).^2 + (err_y_old(:,1)-err_y_old(:,2)).^2));
avg_err_reproject_old = tot_err_old/80;
end