clc;
close all;
%%
%带障碍物轨迹图绘制
figure
hold on;
plot(DX1(1,:),DX1(2,:),'r',DX2(1,:),DX2(2,:),'g',DX3(1,:),DX3(2,:),'b');
plot(x1(1,1:iter_times),x1(2,1:iter_times),':r',x2(1,1:iter_times),x2(2,1:iter_times),':g',x3(1,1:iter_times),x3(2,1:iter_times),':b');
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
plot(0:ts:(iter_times-1)*ts,DX1(3,:), '-r',0:ts:(iter_times-1)*ts,DX2(3,:), '-g',0:ts:(iter_times-1)*ts,DX3(3,:), '-b');
title('智能体的MPC角度状态图');
xlabel('s');
ylabel('\theta');

figure
hold on;
plot(0:ts:(iter_times-1)*ts,T1(:,1)*1000,'r');
plot(0:ts:(iter_times-1)*ts,T2(:,1)*1000,'g');
plot(0:ts:(iter_times-1)*ts,T3(:,1)*1000,'b');
title('智能体的MPC计算时间图');
xlabel('s');
ylabel('ms');

%速度图绘制
figure
hold on;
plot(0:ts:(iter_times-1)*ts,DV1(1,:),'r',0:ts:(iter_times-1)*ts,DV2(1,:),'g',0:ts:(iter_times-1)*ts,DV3(1,:),'b');
plot(0:ts:(iter_times-1)*ts,v1(1,1:iter_times),':r',0:ts:(iter_times-1)*ts,v2(1,1:iter_times),':g',0:ts:(iter_times-1)*ts,v3(1,1:iter_times),':b');
title('智能体的MPC线速度图');
xlabel('s');
ylabel('m/s');
%%
%% 画图
figure;
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
hold on
title('智能体的原轨迹图');
xlabel('m');
ylabel('m');
plot(x1(1,:),x1(2,:), '-r',x2(1,:),x2(2,:), '-g',x3(1,:),x3(2,:), '-b');
axis equal
hold on;

figure
plot(st,x1(3,1:2000), '-r',st,x2(3,1:2000), '-g',st,x3(3,1:2000), '-b');
title('智能体的原角度状态图');
xlabel('s');
ylabel('\theta');

figure
plot(st,dv1o1, '-r',st,dv2o2, '-g',st,dv2o4, '-.g',st,dv3o3, '-b',st,dv3o5, '-.b');
legend('dv1o1','dv2o2','dv2o4','dv3o3','dv3o5','NumColumns',2);
title('智能体的原距离障碍物距离图');
xlabel('s');
ylabel('m');

figure
plot(st,Pr1, '-r',st,Pr2, '-g',st,Pr3, '-b');
