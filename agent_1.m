% clear;
close all;
% clc;
import casadi.*
%% 构建相关模型和函数
iter_times = 1980;               %迭代次数
h = 0.005;                      %时间间隔
N = 20;                         %离散区间数
obstacles1=[];                   %障碍物的状态
obs_safe = 1;                   %安全距离2
wx=100;                         %追踪的权重
wv=20;                          %速度的权重
wd=0;                          %惰性惩罚权重

obstacles=obstacles1;
[obstacle_num_ex,~]=size(obstacles);
obstacle_num=obstacle_num_ex+1;
%obstacle_num=obstacle_num_ex+2;

% 模型变量
p_state = SX.sym('p_State_x',3);                                    %智能体的状态变化,第一维度是x状态，第二维度是y状态，第三维度是角度状态
u = SX.sym('u_v',2);                                                %智能体速度和角度联合输入控制量，第一个是线速度第二个是角速度
ref = SX.sym('ref',5);                                              %智能体的参考输入,前三个分别为状态的参考，后两个分别为线速度和角速度的参考

%运动模型构建
State = p_state + h*[u(1)*cos(p_state(3));u(1)*sin(p_state(3));u(2)];         %智能体的非完整约束模型

%代价函数构建
J=wx*(State(1) - ref(1))^2+wx*(State(2) - ref(2))^2+wx*(State(2) - ref(2))^2+wx*(State(3) - ref(3))^2+...           
  +wv*(u(1)-ref(4))^2+0*(u(2)-ref(5))^2+wd*(1-tanh(u(1)));   %其中第一部分是追踪的代价，第二部分是速度输入的代价，第三部分为惰性惩罚的代价

%输入输出函数构建，即MPC每个时阈的代价函数值
f = Function('f', {p_state, u ,ref}, {State,J}, {'x','u','p'}, {'xf', 'qf'});   %其中输入为智能体当前的状态和当前的输入，以及参考的输入，输出为智能体下一个时刻的状态和目标函数值

%初始化状态量定义
Uk = SX.sym('uk',N*2);        %输入向量
Para=SX.sym('Para',N+1+obstacle_num,5);    %输入前N行为分别对应的参考输入（前三为参考的xy和角度，后二维参考的角速度和线速度），当前状态(第1列到第3列)，最后obstacle_num行为障碍物状态（障碍物的x和y）
g=SX.sym('uk',N*obstacle_num);
lbg=obs_safe*ones(N*obstacle_num,1);
ubg=inf*ones(N*obstacle_num,1);
Jf=0;                         %N个时阈代价值初始化

%构建优化求解问题
for i=1:N
    % 构建代价函数
    if i==1                                %若为到一个时刻的代价函数，则p_state为当前的状态
        Xk=Para(N+1,1:3)';
    end
    Fk = f('x', Xk, 'u', Uk((i-1)*2+1:2*i,1), 'p', Para(i,:)');     %由到一个时刻所产生的函数映射
    Xk = Fk.xf;
    Jf=Jf+Fk.qf;

    % 构建不等式避障约束
    for obs = 1:obstacle_num
        g((i-1)*obstacle_num+obs,1) = norm(Xk(1:2)-Para(N+1+obs,1:2)',2);
    end
end

%优化求解参数设定
prob = struct('f', Jf, 'x', Uk, 'g', g, 'p',Para);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =5;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver = nlpsol('solver', 'ipopt', prob,opts);

%% 优化求解
Para_x=zeros(N+1+obstacle_num,5);
Para_x(N+1,1:3)=x1(:,1)';
DV=zeros(2,iter_times);
DX=zeros(3,iter_times);
T=zeros(iter_times,1);
for i=1:obstacle_num_ex
    Para_x(N+1+i,1:2)=obstacles(i,:);
end
for i=1:iter_times
    DX(:,i)=Para_x(N+1,1:3)';
    Para_x(1:N,1:3)=x1(:,i+1:i+N)';
    Para_x(1:N,4:5)=v1(:,i:i+N-1)';
    Para_x(N+1+obstacle_num_ex+1,1:2)=min_distance_obs(14, -0.2, 0.5, 100, Para_x(N+1,1), Para_x(N+1,2))';
    %Para_x(end,1:2)=min_distance_obs;
    %Para_x((N+1+obstacle_num_ex+2,1:2)=min_distance_obs;
    tic
    sol = solver('lbg', lbg, 'ubg', ubg,'p',Para_x);
    v = full(sol.x);
    T(i,1)=toc;
    DV(:,i)=[v(1,1);v(2,1)];
    Para_x(N+1,1:3)=Para_x(N+1,1:3)+h*[v(1,1)*cos(Para_x(N+1,3));v(1,1)*sin(Para_x(N+1,3));v(2,1)]';
end
DX1=DX;
DV1=DV;
T1=T;
%%
figure
hold on;
plot(DX1(1,:),DX1(2,:),'r');
plot(x1(1,1:iter_times),x1(2,1:iter_times),':r');
aplha1=0:pi/40:2*pi;
r1=0.5;
g1=r1*cos(aplha1)+14;
h1=r1*sin(aplha1)-0.2;
plot(g1,h1,'-');
fill(g1,h1,'k')
hold on
aplha2=0:pi/40:2*pi;
r2=0.5;
g2=r2*cos(aplha2)+4;
h2=r2*sin(aplha2)+2.7;
plot(g2,h2,'-');
fill(g2,h2,'k')
hold on
aplha3=0:pi/40:2*pi;
r3=0.5;
g3=r3*cos(aplha3)+4;
h3=r3*sin(aplha3)-2.9;
plot(g3,h3,'-');
fill(g3,h3,'k')
hold on
aplha4=0:pi/40:2*pi;
r4=1.2;
g4=r4*cos(aplha4)+5.7;
h4=r4*sin(aplha4)+2.7;
plot(g4,h4,'-');
fill(g4,h4,'k')
hold on
aplha5=0:pi/40:2*pi;
r51=2;
r52=0.6;
g5=r51*cos(aplha5)+6.5;
h5=r52*sin(aplha5)-2.9;
plot(g5,h5,'-');
fill(g5,h5,'k')
title('智能体的MPC轨迹图');
xlabel('m');
ylabel('m');
hold on
axis equal

figure
hold on;
plot(0:ts:(iter_times-1)*ts,DX1(3,:),'r');
plot(0:ts:(iter_times-1)*ts,x1(3,1:iter_times),':r');
title('智能体1的MPC角度图');
xlabel('s');
ylabel('rad');


figure
hold on;
plot(0:ts:(iter_times-1)*ts,DV1(1,:),'r');
plot(0:ts:(iter_times-1)*ts,v1(1,1:iter_times),'r');
title('智能体1的MPC线速度图');
xlabel('s');
ylabel('m/s');

figure
hold on;
plot(0:ts:(iter_times-1)*ts,DV1(2,:),'r');
plot(0:ts:(iter_times-1)*ts,v1(2,1:iter_times),'r');
title('智能体1的MPC角速度图');
xlabel('s');
ylabel('rad/s');

figure
hold on;
plot(0:ts:(iter_times-1)*ts,T1(:,1)*1000,'r');
title('智能体1的MPC计算时间图');
xlabel('s');
ylabel('ms');

%%
%智能体1的障碍物函数
function minpose = min_distance_obs(a, b, r, inter, px_cur, py_cur)  %计算移动时的偏航角
aplha = 0:pi/inter:2*pi;
cir_x = r*cos(aplha) + a;
cir_y = r*sin(aplha) + b;
len = length(cir_x);
p_x_enlarge = px_cur.*ones(1, len);
p_y_enlarge = py_cur.*ones(1, len);
distance_vect = (p_x_enlarge - cir_x).^2 + (p_y_enlarge - cir_y).^2;
[~, index] =  min(distance_vect);
minpose = [cir_x(index),cir_y(index)];
end 
