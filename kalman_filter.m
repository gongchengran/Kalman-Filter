% 初始化
clear
close all

% 第一部分，生成目标真实运动轨迹
s = 0.1; % 步长
A = [1, s, s^2 / 2; % 状态转移矩阵
     0, 1, s;
     0, 0, 1];
Q = [0, 0, 0; % 过程噪声的协方差矩阵，一般而言是受到某个外力的扰动
     0, 0, 0;
     0, 0, 0.1^2];
xA = []; % xA：用来记录所有状态
x0 = [100, 1, 2]'; % 目标初始位置为100，初始速度为1，初始加速度为2
x = x0;
for k = 1 : 100  % 100s的匀加速运动
    x = A * x + sqrt(Q) * [randn,randn,randn]';  % 标准正态随机分布
    xA = [xA x];
end
t = 0.1:0.1:10;
figure(1)
plot(t, xA(1,:), 'b')
title('目标位置随时间变化')
hold on
figure(2)
plot(t, xA(2,:))
title('目标速度随时间变化')
figure(3)
plot(t, xA(3,:))
title('目标加速度随时间变化')

% 第二部分，生成测量数据
H = [1, 0, 0]; % 测量矩阵
R = 5^2; % 测量噪声的协方差矩阵，由于测量数据是一维的，这里的R是一个标量
for i = 1 : 100
     zA(i) = H * xA(:,i) + sqrt(R) * randn;
end
figure(1)
plot(t, zA, 'r')
hold on
legend('真实值','测量值')

% 第三部分，开始进行kalman滤波
P0=[1, 0, 0;
    0, 1, 0;
    0, 0, 1]; % 随便给出的初始的协方差矩阵
for i = 1 : 100
    X0 = x0;
    X_k_k_1 = A * X0 ;  %卡尔曼滤波第1个递推公式
    P_k_k_1 = A*P0*A' + Q; %卡尔曼滤波第2个递推公式
    K_k = P_k_k_1 * H' * inv(H*P_k_k_1*H' + R);%卡尔曼滤波第3个递推公式
    X_k = X_k_k_1+K_k*(zA(i) - H*X_k_k_1);%卡尔曼滤波第4个递推公式
    P_k = P_k_k_1 - K_k * H * P_k_k_1;%卡尔曼滤波第5个递推公式

    X_kv(:,i) = X_k;
    x0 = X_k;
    P0 = P_k;
end

figure(1)
plot(t,X_kv(1,:),'g')
legend('真实值','测量值','滤波值') 