% 开发人员：Jason
% 开发地点：Tsinghua University
% 开发时间：2024.9.25
% 开发内容：PSO算法
% 用粒子群算法对PID控制器的三个参数进行优化

%% 清空环境
clear
clc
%% 参数设置
% w = 0.8;      % 惯性因子
c1 = 2;       % 加速常数
c2 = 2;       % 加速常数
Dim = 3;            % 维数  后面用其为列数
SwarmSize = 100;    % 粒子群规模   后面用其为函数
ObjFun = @PSO_PID;  % 待优化函数句柄      %%***   定义的优化  函数PSO_PID  且时  句柄  形式
MaxIter = 50;      % 最大迭代次数  
MinFit = 0.01;       % 最小适应值 （fitness function）       %%***   fitness function适应函数的  最小值

%%可以对应找下面的 四个变量设置
Vmax = 1;
Vmin = -1;
Ub = [30 30 30];
Lb = [0 0 0];

%% 粒子群初始化
    Range = ones(SwarmSize,1)*(Ub-Lb); %产生随机向量
    Swarm = rand(SwarmSize,Dim).*Range + ones(SwarmSize,1)*Lb; % 初始化粒子群位置  是个  100*3 的矩阵
    VStep = rand(SwarmSize,Dim);  % 初始化速度   维度   100*3 的矩阵
    fSwarm = zeros(SwarmSize,1);  %适应度值的初始化  ：0       100*1  的0向量   
for i=1:SwarmSize
    disp(['粒子群初始化进度:',num2str(i)]);
    fSwarm(i,:) = ObjFun(Swarm(i,:));                                         % 粒子群的适应值，更改函数  feval的用法
end

%% 个体极值和群体极值
[bestf,  bestindex]=min(fSwarm);             %bestf数组中的最小值
zbest=Swarm(bestindex,:);                                            % 全局最佳  ：位置
gbest=Swarm;                                                              % 个体最佳  ：个体最佳位置就是初始化位置
fgbest=fSwarm;                                                           % 个体最佳适应值        ：初始化适应度值
fzbest=bestf;                                                               % 全局最佳适应值         ：初始化中最优值

%% 迭代寻优
iter = 0;
y_fitness = zeros(1,MaxIter);   % 预先产生4个空矩阵

K_p = zeros(1,MaxIter);         
K_i = zeros(1,MaxIter);
K_d = zeros(1,MaxIter);
while( (iter < MaxIter) && (fzbest > MinFit) )
    for j=1:SwarmSize
%         % 速度更新
%         disp(['循环次数：',num2str(iter),'粒子群位置：',num2str(j)]);
        w=0.9-iter*0.8/MaxIter;
        VStep(j,:) = w*VStep(j,:) + c1*rand*(gbest(j,:) - Swarm(j,:)) + c2*rand*(zbest - Swarm(j,:));
        if VStep(j,:)>Vmax, VStep(j,:)=Vmax; 
           end
        if VStep(j,:)<Vmin, VStep(j,:)=Vmin;
           end
        % 位置更新
        Swarm(j,:)=Swarm(j,:)+VStep(j,:);
        for k=1:Dim
            if Swarm(j,k)>Ub(k), Swarm(j,k)=Ub(k); 
               end
            if Swarm(j,k)<Lb(k), Swarm(j,k)=Lb(k); 
               end
        end
        % 适应值
        fSwarm(j,:) = ObjFun(Swarm(j,:));         %更改函数  feval的用法
        % 个体最优更新     
        if fSwarm(j) < fgbest(j)
            gbest(j,:) = Swarm(j,:);      %个体位置更新
            fgbest(j) = fSwarm(j);       %个体适应值更新
        end
        % 群体最优更新
        if fSwarm(j) < fzbest 
            zbest = Swarm(j,:);           %全体最优位置更新
            fzbest = fSwarm(j);          %全体最优适应值更新
        end
    end 
    disp(['循环次数：',num2str(iter)]);
    disp(['最佳适应度：',num2str(fzbest)]);
    disp(['最佳个体：',num2str(zbest(1)),' ',num2str(zbest(2)),' ',num2str(zbest(3))]);    %陈列kp,ki,kd
    iter = iter+1;                      % 迭代次数更新
    y_fitness(1,iter) = fzbest;         % 为绘图做准备
    K_p(1,iter) = zbest(1);
    K_i(1,iter) = zbest(2);
    K_d(1,iter) = zbest(3);
end
%% 绘图输出
figure(1)      % 绘制性能指标ITAE的变化曲线
plot(y_fitness,'LineWidth',2)
title('最优个体适应值','fontsize',18);
xlabel('迭代次数','fontsize',18);ylabel('适应值','fontsize',18);
set(gca,'Fontsize',18);

figure(2)      % 绘制PID控制器参数变化曲线
plot(K_p)
hold on
plot(K_i,'k','LineWidth',3)
plot(K_d,'--r')
title('Kp、Ki、Kd 优化曲线','fontsize',18);
xlabel('迭代次数','fontsize',18);ylabel('参数值','fontsize',18);
set(gca,'Fontsize',18);
legend('Kp','Ki','Kd');
