% clear;
close all;
% clc;
import casadi.*
%% �������ģ�ͺͺ���
iter_times = 1980;               %��������
h = 0.005;                      %ʱ����
N = 20;                         %��ɢ������
obstacles1=[];                   %�ϰ����״̬
obs_safe = 1;                   %��ȫ����2
wx=100;                         %׷�ٵ�Ȩ��
wv=20;                          %�ٶȵ�Ȩ��
wd=0;                          %���Գͷ�Ȩ��

obstacles=obstacles1;
[obstacle_num_ex,~]=size(obstacles);
obstacle_num=obstacle_num_ex+1;
%obstacle_num=obstacle_num_ex+2;

% ģ�ͱ���
p_state = SX.sym('p_State_x',3);                                    %�������״̬�仯,��һά����x״̬���ڶ�ά����y״̬������ά���ǽǶ�״̬
u = SX.sym('u_v',2);                                                %�������ٶȺͽǶ������������������һ�������ٶȵڶ����ǽ��ٶ�
ref = SX.sym('ref',5);                                              %������Ĳο�����,ǰ�����ֱ�Ϊ״̬�Ĳο����������ֱ�Ϊ���ٶȺͽ��ٶȵĲο�

%�˶�ģ�͹���
State = p_state + h*[u(1)*cos(p_state(3));u(1)*sin(p_state(3));u(2)];         %������ķ�����Լ��ģ��

%���ۺ�������
J=wx*(State(1) - ref(1))^2+wx*(State(2) - ref(2))^2+wx*(State(2) - ref(2))^2+wx*(State(3) - ref(3))^2+...           
  +wv*(u(1)-ref(4))^2+0*(u(2)-ref(5))^2+wd*(1-tanh(u(1)));   %���е�һ������׷�ٵĴ��ۣ��ڶ��������ٶ�����Ĵ��ۣ���������Ϊ���Գͷ��Ĵ���

%�������������������MPCÿ��ʱ�еĴ��ۺ���ֵ
f = Function('f', {p_state, u ,ref}, {State,J}, {'x','u','p'}, {'xf', 'qf'});   %��������Ϊ�����嵱ǰ��״̬�͵�ǰ�����룬�Լ��ο������룬���Ϊ��������һ��ʱ�̵�״̬��Ŀ�꺯��ֵ

%��ʼ��״̬������
Uk = SX.sym('uk',N*2);        %��������
Para=SX.sym('Para',N+1+obstacle_num,5);    %����ǰN��Ϊ�ֱ��Ӧ�Ĳο����루ǰ��Ϊ�ο���xy�ͽǶȣ����ά�ο��Ľ��ٶȺ����ٶȣ�����ǰ״̬(��1�е���3��)�����obstacle_num��Ϊ�ϰ���״̬���ϰ����x��y��
g=SX.sym('uk',N*obstacle_num);
lbg=obs_safe*ones(N*obstacle_num,1);
ubg=inf*ones(N*obstacle_num,1);
Jf=0;                         %N��ʱ�д���ֵ��ʼ��

%�����Ż��������
for i=1:N
    % �������ۺ���
    if i==1                                %��Ϊ��һ��ʱ�̵Ĵ��ۺ�������p_stateΪ��ǰ��״̬
        Xk=Para(N+1,1:3)';
    end
    Fk = f('x', Xk, 'u', Uk((i-1)*2+1:2*i,1), 'p', Para(i,:)');     %�ɵ�һ��ʱ���������ĺ���ӳ��
    Xk = Fk.xf;
    Jf=Jf+Fk.qf;

    % ��������ʽ����Լ��
    for obs = 1:obstacle_num
        g((i-1)*obstacle_num+obs,1) = norm(Xk(1:2)-Para(N+1+obs,1:2)',2);
    end
end

%�Ż��������趨
prob = struct('f', Jf, 'x', Uk, 'g', g, 'p',Para);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =5;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver = nlpsol('solver', 'ipopt', prob,opts);

%% �Ż����
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
title('�������MPC�켣ͼ');
xlabel('m');
ylabel('m');
hold on
axis equal

figure
hold on;
plot(0:ts:(iter_times-1)*ts,DX1(3,:),'r');
plot(0:ts:(iter_times-1)*ts,x1(3,1:iter_times),':r');
title('������1��MPC�Ƕ�ͼ');
xlabel('s');
ylabel('rad');


figure
hold on;
plot(0:ts:(iter_times-1)*ts,DV1(1,:),'r');
plot(0:ts:(iter_times-1)*ts,v1(1,1:iter_times),'r');
title('������1��MPC���ٶ�ͼ');
xlabel('s');
ylabel('m/s');

figure
hold on;
plot(0:ts:(iter_times-1)*ts,DV1(2,:),'r');
plot(0:ts:(iter_times-1)*ts,v1(2,1:iter_times),'r');
title('������1��MPC���ٶ�ͼ');
xlabel('s');
ylabel('rad/s');

figure
hold on;
plot(0:ts:(iter_times-1)*ts,T1(:,1)*1000,'r');
title('������1��MPC����ʱ��ͼ');
xlabel('s');
ylabel('ms');

%%
%������1���ϰ��ﺯ��
function minpose = min_distance_obs(a, b, r, inter, px_cur, py_cur)  %�����ƶ�ʱ��ƫ����
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
