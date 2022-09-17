clc
clear all
close all
%% 初始值设定
t=0;                %时间起始值
FM=9;               %移动/编队任务的任务增益
FR=9;               %编队重构任务的任务增益
OA=20;              %避障任务的任务增益
d_OA=1;             %避障的安全距离
J=2000;            %迭代次数
T=10;               %总时长
ts=T/J;             %采样间隔
%% 小车1初始值
v1=[0;0];             %小车1的初始差速速度
theta1=0;             %小车1的初始偏航角
Theta1=[cos(theta1),0;sin(theta1),0;0,1];  %小车1的非完整约束矩阵
dotx1=Theta1*v1;      %小车1的初始广义速度 
P1=[-1;0];            %小车1的初始广义位置 
x1=[P1;theta1];       %小车1的初始状态
Pr1=0;                %小车1优先级 0为FM，1为FR，2为OA
%% 小车2初始值
v2=[0;0];             %小车2的初始差速速度
theta2=0;             %小车2的初始偏航角
Theta2=[cos(theta2),0;sin(theta2),0;0,1];  %小车2的非完整约束矩阵
dotx2=Theta2*v2;      %小车2的初始广义速度 
P2=[-4;4];            %小车2的初始广义位置 
x2=[P2;theta2];       %小车2的初始状态 
Pr2=0;                %小车2优先级 
%% 小车3初始值
v3=[0;0];             %小车3的初始差速速度
theta3=0;             %小车3的初始偏航角
Theta3=[cos(theta3),0;sin(theta3),0;0,1];  %小车3的非完整约束矩阵
dotx3=Theta3*v3;      %小车3的初始广义速度 
P3=[-4;-4];           %小车3的初始广义位置 
x3=[P3;theta3];       %小车3的初始状态
Pr3=0;                %小车3优先级 
%% 质心相关参数
xc=1/3*(x1+x2+x3);    %质心的初始状态 
P1cd=[2;0];           %小车1与质心的期望相对位置
P2cd=[-1;3];          %小车2与质心的期望相对位置
P3cd=[-1;-3];         %小车3与质心的期望相对位置
P21d=[-2;0];        %编队重构时小车2与小车1的期望相对位置
P31d=[-4;0];          %编队重构时小车3与小车1的期望相对位置
%% MPC参数
%输入的参数
Nc=5;                       %控制时阈
oa=2;                        %障碍物感知范围,oa为K时刻之前考虑的障碍物
ob=10;                       %ob为k时刻及k时刻之后k+ob-1时刻的障碍物
wt=10;                        %轨迹跟踪的权重
wv=1;                        %速度跟踪的权重
%障碍物存储
OB1=zeros(2,J);             %智能体1障碍物集合初始化
OB21=zeros(2,J); OB22=zeros(2,J);  %智能体2障碍物集合初始化
OB31=zeros(2,J); OB32=zeros(2,J);  %智能体3障碍物集合初始化


%% 循环部分
tic
for j=1:J 
    t=j*ts;              %行为控制部分时间累计
    st(j)=t;             %记录时间
    Pcd(:,j)=[-2+2*t;0];   %编队任务的期望轨迹
    P1o1=min_distance_obs(14, -0.2, 0.5, 100, x1(1,j), x1(2,j))';           %小车1与障碍物1最近障碍物点
    OB1(:,j)=P1o1;                                                          %智能体1障碍物实时存储
    P2o2=min_distance_obs(4, 2.7, 0.5, 100, x2(1,j), x2(2,j))';             %小车2与障碍物2最近障碍物点
    P2o4=min_distance_obs(5.7, 2.7, 1.2, 100, x2(1,j), x2(2,j))';           %小车2与障碍物4最近障碍物点
    OB21(:,j)=P2o2;OB22(:,j)=P2o4;                                          %智能体2两个障碍物实时存储
    P3o3=min_distance_obs(4, -2.9, 0.5, 100, x3(1,j), x3(2,j))';            %小车3与障碍物3最近障碍物点
    P3o5=min_distance_obs2(6.5, -2.9, 2, 0.6, 100, x3(1,j), x3(2,j))';      %小车3与障碍物5最近障碍物点
    OB31(:,j)=P3o3;OB32(:,j)=P3o5;                                          %障碍物3两个障碍物存储
    dv1o1(j)=norm(x1(1:2,j)-P1o1);    %小车1与障碍物1之间的距离
    dv2o2(j)=norm(x2(1:2,j)-P2o2);    %小车2与障碍物2之间的距离
    dv2o4(j)=norm(x2(1:2,j)-P2o4);    %小车2与障碍物4之间的距离
    dv3o3(j)=norm(x3(1:2,j)-P3o3);    %小车3与障碍物3之间的距离
    dv3o5(j)=norm(x3(1:2,j)-P3o5);    %小车3与障碍物5之间的距离
   %% 小车1迭代部分    
    JFM1=eye(3);                    %小车1的雅可比矩阵
    if j==1
    Pd1_FM(:,j)=Pcd(:,j)+P1cd;  %小车1的编队期望位置  
    theta_FM1(j)=0;                 %小车1期望的偏航角
    dotPd1_FM=[2;0;theta_FM1(j)];      %小车1编队任务期望速度导数    
    else 
    Pd1_FM(:,j)=Pcd(:,j)+P1cd;  %小车1的编队期望位置       
    theta_FM1(j)=A_thetad(Pd1_FM(1,j),Pd1_FM(2,j),x1(1,j),x1(2,j)); %计算小车1的期望的偏航角
    dotPd1_FM=[2;0;theta_FM1(j)];        %小车1编队任务期望速度导数
    end    
    if j<1248 || j>1560
    xFM1(:,j)=[Pd1_FM(:,j);theta_FM1(j)];   %小车1编队任务的期望任务函数
    VFM1(:,j)=dotPd1_FM+pinv(JFM1)*FM*(xFM1(:,j)-x1(:,j));  %小车1的编队任务的的速度
    Upsilon_OA1=((x1(1:2,j)-P1o1)/dv1o1(j))';      %对应论文中的Upsilon_OA
    JOA1=[Upsilon_OA1,zeros(1,1);zeros(1,2),1];  %小车1避障任务的雅可比矩阵
    theta_OA1(j)=B_thetad(P1o1(1),P1o1(2),x1(1,j),x1(2,j)); %小车1避障任务的期望角度
    xOA1(:,j)=[d_OA;theta_OA1(j)];   %小车1避障任务的期望任务函数
    VOA1(:,j)=pinv(JOA1)*OA*(xOA1(:,j)-[dv1o1(j);x1(3,j)]); %小车1避障任务的的速度
    dotx1(:,j)=VFM1(:,j)+(eye(3)-pinv(JFM1)*JFM1)*VOA1(:,j); %小车1的速度
    Theta1=[cos(x1(3,j)),0;sin(x1(3,j)),0;0,1];  %小车1的非完整约束矩阵
    v1(:,j)=pinv(Theta1)*dotx1(:,j);    %小车1的差速速度
    Pr1(j)=0;
    else
    xFM1(:,j)=[Pd1_FM(:,j);theta_FM1(j)];   %小车1编队任务的期望任务函数
    VFM1(:,j)=dotPd1_FM+pinv(JFM1)*FM*(xFM1(:,j)-x1(:,j));  %小车1的编队任务的的速度
    Upsilon_OA1=((x1(1:2,j)-P1o1)/dv1o1(j))';      %对应论文中的Upsilon_OA
    JOA1=[Upsilon_OA1,zeros(1,1);zeros(1,2),1];    %小车1避障任务的雅可比矩阵
    theta_OA1(j)=B_thetad(P1o1(1),P1o1(2),x1(1,j),x1(2,j)); %小车1避障任务的期望角度
    xOA1(:,j)=[d_OA;theta_OA1(j)];   %小车1避障任务的期望任务函数
    VOA1(:,j)=pinv(JOA1)*OA*(xOA1(:,j)-[dv1o1(j);x1(3,j)]); %小车1避障任务的的速度
    dotx1(:,j)=VOA1(:,j)+(eye(3)-pinv(JOA1)*JOA1)*VFM1(:,j); %小车1的速度
    Theta1=[cos(x1(3,j)),0;sin(x1(3,j)),0;0,1];  %小车1的非完整约束矩阵
    v1(:,j)=pinv(Theta1)*dotx1(:,j);    %小车1的差速速度
    Pr1(j)=2;
    end
    x1(:,j+1)= x1(:,j)+Theta1*v1(:,j)*ts;  %小车1的位置更新    
    %% 小车2，3迭代部分 %Key Part
    JFM2=eye(3);                    %小车2编队保持的雅可比矩阵
    JFR2=eye(3);                    %小车2编队重构的雅可比矩阵
    if j==1
    Pd2_FM(:,j)=Pcd(:,j)+P2cd;  %小车2的编队期望位置    
    theta_FM2(j)=0;                 %小车2期望的偏航角
    dotPd2_FM=[2;0;theta_FM2(j)];      %小车2编队任务期望速度导数    
    else 
    Pd2_FM(:,j)=Pcd(:,j)+P2cd;  %小车2的编队期望位置       
    theta_FM2(j)=A_thetad(Pd2_FM(1,j),Pd2_FM(2,j),x2(1,j),x2(2,j)); %计算小车2的期望的偏航角
    dotPd2_FM=[2;0;theta_FM2(j)];        %小车2编队任务期望速度导数
    end 
    if j==1
    Pd2_FR(:,j)=Pd1_FM(:,j)+P21d;  %小车2的编队重构期望位置    
    theta_FR2(j)=0;                 %小车2期望的偏航角
    dotPd2_FR=[2;0;theta_FR2(j)];   %小车2编队任务期望速度导数    
    else  
    Pd2_FR(:,j)=Pd1_FM(:,j)+P21d;   %小车2的编队重构期望位置        
    theta_FR2(j)=A_thetad(Pd2_FR(1,j),Pd2_FR(2,j),x2(1,j),x2(2,j));  %计算小车2的期望的偏航角
    dotPd2_FR=[2;0;theta_FR2(j)];        %小车2编队任务期望速度导数
    end 
    JFM3=eye(3);                    %小车3编队保持的雅可比矩阵
    JFR3=eye(3);                    %小车3编队重构的雅可比矩阵
    if j==1
    Pd3_FM(:,j)=Pcd(:,j)+P3cd;      %小车3的编队期望位置    
    theta_FM3(j)=0;                 %小车3期望的偏航角
    dotPd3_FM=[2;0;theta_FM3(j)];   %小车3编队任务期望速度导数    
    else 
    Pd3_FM(:,j)=Pcd(:,j)+P3cd;  %小车3的编队期望位置       
    theta_FM3(j)=A_thetad(Pd3_FM(1,j),Pd3_FM(2,j),x3(1,j),x3(2,j));  %计算小车3的期望的偏航角
    dotPd3_FM=[2;0;theta_FM3(j)];        %小车3编队任务期望速度导数
    end 
    if j==1
    Pd3_FR(:,j)=Pd1_FM(:,j)+P31d;   %小车3的编队重构期望位置    
    theta_FR3(j)=0;                 %小车3期望的偏航角
    dotPd3_FR=[1;0;theta_FR3(j)];   %小车3编队任务期望速度导数    
    else 
    Pd3_FR(:,j)=Pd1_FM(:,j)+P31d;   %小车3的编队重构期望位置        
    theta_FR3(j)=A_thetad(Pd3_FR(1,j),Pd3_FR(2,j),x3(1,j),x3(2,j));  %计算小车3的期望的偏航角
    dotPd3_FR=[1;0;theta_FR3(j)];        %小车3编队任务期望速度导数
    end
    if dv2o4(j)<dv2o2(j)            %小车2离哪个障碍物近
    Po2=P2o4;
    dv2o(j)=dv2o4(j);
    else
    Po2=P2o2;
    dv2o(j)=dv2o2(j);
    end
    if dv3o5(j)<dv3o3(j)            %小车3离哪个障碍物近
    Po3=P3o5;
    dv3o(j)=dv3o5(j);
    else
    Po3=P3o3;
    dv3o(j)=dv3o3(j);
    end
    if j>=417 && j<=1190
    xFM2(:,j)=[Pd2_FM(:,j);theta_FM2(j)];   %小车2编队任务的期望任务函数
    VFM2(:,j)=dotPd2_FM+pinv(JFM2)*FM*(xFM2(:,j)-x2(:,j));  %小车2的编队任务的的速度
    xFR2(:,j)=[Pd2_FR(:,j);theta_FR2(j)];   %小车2编队任务的期望任务函数
    VFR2(:,j)=dotPd2_FR+pinv(JFR2)*FR*(xFR2(:,j)-x2(:,j));  %小车2的编队任务的的速度
    Upsilon_OA2=((x2(1:2,j)-Po2)/dv2o(j))';      %对应论文中的Upsilon_OA    
    JOA2=[Upsilon_OA2,zeros(1,1);zeros(1,2),1];    %小车2避障任务的雅可比矩阵    
    theta_OA2(j)=B_thetad(Po2(1),Po2(2),x2(1,j),x2(2,j)); %小车2避障任务的期望角度    
    xOA2(:,j)=[d_OA;theta_OA2(j)];   %小车2避障任务的期望任务函数
    VOA2(:,j)=pinv(JOA2)*OA*(xOA2(:,j)-[dv2o(j);x2(3,j)]); %小车2避障任务的的速度
    dotx2(:,j)=VFR2(:,j)+(eye(3)-pinv(JFR2)*JFR2)*(VFM2(:,j)+(eye(3)-pinv(JFM2)*JFM2)*VOA2(:,j)); %小车2的速度
    Theta2=[cos(x2(3,j)),0;sin(x2(3,j)),0;0,1];  %小车2的非完整约束矩阵
    v2(:,j)=pinv(Theta2)*dotx2(:,j);    %小车2的差速速度
    Pr2(j)=1;
    xFM3(:,j)=[Pd3_FM(:,j);theta_FM3(j)];   %小车3编队任务的期望任务函数
    VFM3(:,j)=dotPd3_FM+pinv(JFM3)*FM*(xFM3(:,j)-x3(:,j));  %小车3的编队任务的的速度
    xFR3(:,j)=[Pd3_FR(:,j);theta_FR3(j)];   %小车3编队任务的期望任务函数
    VFR3(:,j)=dotPd3_FR+pinv(JFR3)*FR*(xFR3(:,j)-x3(:,j));  %小车3的编队任务的的速度
    Upsilon_OA3=((x3(1:2,j)-Po3)/dv3o(j))';      %对应论文中的Upsilon_OA    
    JOA3=[Upsilon_OA3,zeros(1,1);zeros(1,2),1];    %小车3避障任务的雅可比矩阵    
    theta_OA3(j)=B_thetad(Po3(1),Po3(2),x3(1,j),x3(2,j)); %小车3避障任务的期望角度    
    xOA3(:,j)=[d_OA;theta_OA3(j)];   %小车2避障任务的期望任务函数
    VOA3(:,j)=pinv(JOA3)*OA*(xOA3(:,j)-[dv3o(j);x3(3,j)]); %小车3避障任务的的速度                   
    dotx3(:,j)=VFR3(:,j)+(eye(3)-pinv(JFR3)*JFR3)*(VFM3(:,j)+(eye(3)-pinv(JFM3)*JFM3)*VOA3(:,j)); %小车3的速度
    Theta3=[cos(x3(3,j)),0;sin(x3(3,j)),0;0,1];  %小车3的非完整约束矩阵
    v3(:,j)=pinv(Theta3)*dotx3(:,j);    %小车3的差速速度
    Pr3(j)=1;
    else        
    xFM2(:,j)=[Pd2_FM(:,j);theta_FM2(j)];   %小车2编队任务的期望任务函数
    VFM2(:,j)=dotPd2_FM+pinv(JFM2)*FM*(xFM2(:,j)-x2(:,j));  %小车2的编队任务的的速度
    xFR2(:,j)=[Pd2_FR(:,j);theta_FR2(j)];   %小车2编队任务的期望任务函数
    VFR2(:,j)=dotPd2_FR+pinv(JFR2)*FR*(xFR2(:,j)-x2(:,j));  %小车2的编队任务的的速度
    Upsilon_OA2=((x2(1:2,j)-P2o2)/dv2o2(j))';      %对应论文中的Upsilon_OA    
    JOA2=[Upsilon_OA2,zeros(1,1);zeros(1,2),1];    %小车2避障任务的雅可比矩阵    
    theta_OA2(j)=B_thetad(P2o2(1),P2o2(2),x2(1,j),x2(2,j)); %小车2避障任务的期望角度    
    xOA2(:,j)=[d_OA;theta_OA2(j)];   %小车2避障任务的期望任务函数
    VOA2(:,j)=pinv(JOA2)*OA*(xOA2(:,j)-[dv2o2(j);x2(3,j)]); %小车2避障任务的的速度        
    dotx2(:,j)=VFM2(:,j)+(eye(3)-pinv(JFM2)*JFM2)*(VFR2(:,j)+(eye(3)-pinv(JFR2)*JFR2)*VOA2(:,j)); %小车2的速度
    Theta2=[cos(x2(3,j)),0;sin(x2(3,j)),0;0,1];  %小车2的非完整约束矩阵
    v2(:,j)=pinv(Theta2)*dotx2(:,j);    %小车2的差速速度
    Pr2(j)=0;
    xFM3(:,j)=[Pd3_FM(:,j);theta_FM3(j)];   %小车3编队任务的期望任务函数
    VFM3(:,j)=dotPd3_FM+pinv(JFM3)*FM*(xFM3(:,j)-x3(:,j));  %小车3的编队任务的的速度
    xFR3(:,j)=[Pd3_FR(:,j);theta_FR3(j)];   %小车3编队任务的期望任务函数
    VFR3(:,j)=dotPd3_FR+pinv(JFR3)*FR*(xFR3(:,j)-x3(:,j));  %小车3的编队任务的的速度
    Upsilon_OA3=((x3(1:2,j)-P3o3)/dv3o3(j))';      %对应论文中的Upsilon_OA    
    JOA3=[Upsilon_OA3,zeros(1,1);zeros(1,2),1];    %小车3避障任务的雅可比矩阵    
    theta_OA3(j)=B_thetad(P3o3(1),P3o3(2),x3(1,j),x3(2,j)); %小车3避障任务的期望角度    
    xOA3(:,j)=[d_OA;theta_OA3(j)];   %小车2避障任务的期望任务函数
    VOA3(:,j)=pinv(JOA3)*OA*(xOA3(:,j)-[dv3o3(j);x3(3,j)]); %小车3避障任务的的速度       
    dotx3(:,j)=VFM3(:,j)+(eye(3)-pinv(JFM3)*JFM3)*(VFR3(:,j)+(eye(3)-pinv(JFR3)*JFR3)*VOA3(:,j)); %小车3的速度        
    Theta3=[cos(x3(3,j)),0;sin(x3(3,j)),0;0,1];  %小车3的非完整约束矩阵
    v3(:,j)=pinv(Theta3)*dotx3(:,j);    %小车3的差速速度
    Pr3(j)=0;
    end
    x2(:,j+1)= x2(:,j)+Theta2*v2(:,j)*ts;  %小车2的位置更新
    x3(:,j+1)= x3(:,j)+Theta3*v3(:,j)*ts;  %小车2的位置更新
    %% 质心迭代更新
    xc(:,j+1)=(x1(:,j+1)+x2(:,j+1)+x3(:,j+1))/3;  %小车质心的位置更新   
    end
toc



%% 画图
figure(1);
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
plot(x1(1,:),x1(2,:), '-r',x2(1,:),x2(2,:), '-g',x3(1,:),x3(2,:), '-b');
axis equal
hold on;
figure(2);
plot(st,x1(3,1:2000), '-r',st,x2(3,1:2000), '-g',st,x3(3,1:2000), '-b');
figure(3);
plot(st,dv1o1, '-r',st,dv2o2, '-g',st,dv2o4, '-.g',st,dv3o3, '-b',st,dv3o5, '-.b');
legend('dv1o1','dv2o2','dv2o4','dv3o3','dv3o5','NumColumns',2);
figure(4);
plot(st,Pr1, '-r',st,Pr2, '-g',st,Pr3, '-b');


%% 函数
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
function minpose2 = min_distance_obs2(a2, b2, r21,r22, inter2, px_cur2, py_cur2)  %计算移动时的偏航角
aplha2 = 0:pi/inter2:2*pi;
cir_x2 = r21*cos(aplha2) + a2;
cir_y2 = r22*sin(aplha2) + b2;
len2 = length(cir_x2);
p_x_enlarge2 = px_cur2.*ones(1, len2);
p_y_enlarge2 = py_cur2.*ones(1, len2);
distance_vect2 = (p_x_enlarge2 - cir_x2).^2 + (p_y_enlarge2 - cir_y2).^2;
[~, index] =  min(distance_vect2);
minpose2 = [cir_x2(index),cir_y2(index)];
end 
function theta_FM=A_thetad(Pxd2,Pyd2,Px,Py)  %计算移动时的偏航角
if abs(Pyd2-Py)>0.03
theta_FM=atan2(Pyd2-Py,Pxd2-Px);
else
theta_FM=0;
end
end
function theta_OA=B_thetad(Px,Py,Pox,Poy)   %计算避障时的偏航角
if Py>Poy
theta_OA=atan2(Poy-Py,Pox-Px)+pi/2;
else if Py<=Poy
theta_OA=atan2(Poy-Py,Pox-Px)-pi/2;
end
end
end

