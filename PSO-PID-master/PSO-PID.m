% 开发人员：Jason
% 开发地点：Tsinghua University
% 开发时间：2024.9.25
% 开发内容：PSO优化的PID控制程序
function z=PSO_PID(x)
assignin('base','Kp',x(1));
assignin('base','Ki',x(2));
assignin('base','Kd',x(3));
[~,~,y_out]=sim('thetapid',[0,50]);
z=y_out(end,1);   %%表示 取这个矩阵的第一列最后一行的数据。
