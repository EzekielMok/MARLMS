clc
clear all
close all
%% ��ʼֵ�趨
t=0;                %ʱ����ʼֵ
FM=9;               %�ƶ�/����������������
FR=9;               %����ع��������������
OA=20;              %�����������������
d_OA=1;             %���ϵİ�ȫ����
J=2000;            %��������
T=10;               %��ʱ��
ts=T/J;             %�������
%% С��1��ʼֵ
v1=[0;0];             %С��1�ĳ�ʼ�����ٶ�
theta1=0;             %С��1�ĳ�ʼƫ����
Theta1=[cos(theta1),0;sin(theta1),0;0,1];  %С��1�ķ�����Լ������
dotx1=Theta1*v1;      %С��1�ĳ�ʼ�����ٶ� 
P1=[-1;0];            %С��1�ĳ�ʼ����λ�� 
x1=[P1;theta1];       %С��1�ĳ�ʼ״̬
Pr1=0;                %С��1���ȼ� 0ΪFM��1ΪFR��2ΪOA
%% С��2��ʼֵ
v2=[0;0];             %С��2�ĳ�ʼ�����ٶ�
theta2=0;             %С��2�ĳ�ʼƫ����
Theta2=[cos(theta2),0;sin(theta2),0;0,1];  %С��2�ķ�����Լ������
dotx2=Theta2*v2;      %С��2�ĳ�ʼ�����ٶ� 
P2=[-4;4];            %С��2�ĳ�ʼ����λ�� 
x2=[P2;theta2];       %С��2�ĳ�ʼ״̬ 
Pr2=0;                %С��2���ȼ� 
%% С��3��ʼֵ
v3=[0;0];             %С��3�ĳ�ʼ�����ٶ�
theta3=0;             %С��3�ĳ�ʼƫ����
Theta3=[cos(theta3),0;sin(theta3),0;0,1];  %С��3�ķ�����Լ������
dotx3=Theta3*v3;      %С��3�ĳ�ʼ�����ٶ� 
P3=[-4;-4];           %С��3�ĳ�ʼ����λ�� 
x3=[P3;theta3];       %С��3�ĳ�ʼ״̬
Pr3=0;                %С��3���ȼ� 
%% ������ز���
xc=1/3*(x1+x2+x3);    %���ĵĳ�ʼ״̬ 
P1cd=[2;0];           %С��1�����ĵ��������λ��
P2cd=[-1;3];          %С��2�����ĵ��������λ��
P3cd=[-1;-3];         %С��3�����ĵ��������λ��
P21d=[-2;0];        %����ع�ʱС��2��С��1���������λ��
P31d=[-4;0];          %����ع�ʱС��3��С��1���������λ��
%% MPC����
%����Ĳ���
Nc=5;                       %����ʱ��
oa=2;                        %�ϰ����֪��Χ,oaΪKʱ��֮ǰ���ǵ��ϰ���
ob=10;                       %obΪkʱ�̼�kʱ��֮��k+ob-1ʱ�̵��ϰ���
wt=10;                        %�켣���ٵ�Ȩ��
wv=1;                        %�ٶȸ��ٵ�Ȩ��
%�ϰ���洢
OB1=zeros(2,J);             %������1�ϰ��Ｏ�ϳ�ʼ��
OB21=zeros(2,J); OB22=zeros(2,J);  %������2�ϰ��Ｏ�ϳ�ʼ��
OB31=zeros(2,J); OB32=zeros(2,J);  %������3�ϰ��Ｏ�ϳ�ʼ��


%% ѭ������
tic
for j=1:J 
    t=j*ts;              %��Ϊ���Ʋ���ʱ���ۼ�
    st(j)=t;             %��¼ʱ��
    Pcd(:,j)=[-2+2*t;0];   %�������������켣
    P1o1=min_distance_obs(14, -0.2, 0.5, 100, x1(1,j), x1(2,j))';           %С��1���ϰ���1����ϰ����
    OB1(:,j)=P1o1;                                                          %������1�ϰ���ʵʱ�洢
    P2o2=min_distance_obs(4, 2.7, 0.5, 100, x2(1,j), x2(2,j))';             %С��2���ϰ���2����ϰ����
    P2o4=min_distance_obs(5.7, 2.7, 1.2, 100, x2(1,j), x2(2,j))';           %С��2���ϰ���4����ϰ����
    OB21(:,j)=P2o2;OB22(:,j)=P2o4;                                          %������2�����ϰ���ʵʱ�洢
    P3o3=min_distance_obs(4, -2.9, 0.5, 100, x3(1,j), x3(2,j))';            %С��3���ϰ���3����ϰ����
    P3o5=min_distance_obs2(6.5, -2.9, 2, 0.6, 100, x3(1,j), x3(2,j))';      %С��3���ϰ���5����ϰ����
    OB31(:,j)=P3o3;OB32(:,j)=P3o5;                                          %�ϰ���3�����ϰ���洢
    dv1o1(j)=norm(x1(1:2,j)-P1o1);    %С��1���ϰ���1֮��ľ���
    dv2o2(j)=norm(x2(1:2,j)-P2o2);    %С��2���ϰ���2֮��ľ���
    dv2o4(j)=norm(x2(1:2,j)-P2o4);    %С��2���ϰ���4֮��ľ���
    dv3o3(j)=norm(x3(1:2,j)-P3o3);    %С��3���ϰ���3֮��ľ���
    dv3o5(j)=norm(x3(1:2,j)-P3o5);    %С��3���ϰ���5֮��ľ���
   %% С��1��������    
    JFM1=eye(3);                    %С��1���ſɱȾ���
    if j==1
    Pd1_FM(:,j)=Pcd(:,j)+P1cd;  %С��1�ı������λ��  
    theta_FM1(j)=0;                 %С��1������ƫ����
    dotPd1_FM=[2;0;theta_FM1(j)];      %С��1������������ٶȵ���    
    else 
    Pd1_FM(:,j)=Pcd(:,j)+P1cd;  %С��1�ı������λ��       
    theta_FM1(j)=A_thetad(Pd1_FM(1,j),Pd1_FM(2,j),x1(1,j),x1(2,j)); %����С��1��������ƫ����
    dotPd1_FM=[2;0;theta_FM1(j)];        %С��1������������ٶȵ���
    end    
    if j<1248 || j>1560
    xFM1(:,j)=[Pd1_FM(:,j);theta_FM1(j)];   %С��1������������������
    VFM1(:,j)=dotPd1_FM+pinv(JFM1)*FM*(xFM1(:,j)-x1(:,j));  %С��1�ı������ĵ��ٶ�
    Upsilon_OA1=((x1(1:2,j)-P1o1)/dv1o1(j))';      %��Ӧ�����е�Upsilon_OA
    JOA1=[Upsilon_OA1,zeros(1,1);zeros(1,2),1];  %С��1����������ſɱȾ���
    theta_OA1(j)=B_thetad(P1o1(1),P1o1(2),x1(1,j),x1(2,j)); %С��1��������������Ƕ�
    xOA1(:,j)=[d_OA;theta_OA1(j)];   %С��1�������������������
    VOA1(:,j)=pinv(JOA1)*OA*(xOA1(:,j)-[dv1o1(j);x1(3,j)]); %С��1��������ĵ��ٶ�
    dotx1(:,j)=VFM1(:,j)+(eye(3)-pinv(JFM1)*JFM1)*VOA1(:,j); %С��1���ٶ�
    Theta1=[cos(x1(3,j)),0;sin(x1(3,j)),0;0,1];  %С��1�ķ�����Լ������
    v1(:,j)=pinv(Theta1)*dotx1(:,j);    %С��1�Ĳ����ٶ�
    Pr1(j)=0;
    else
    xFM1(:,j)=[Pd1_FM(:,j);theta_FM1(j)];   %С��1������������������
    VFM1(:,j)=dotPd1_FM+pinv(JFM1)*FM*(xFM1(:,j)-x1(:,j));  %С��1�ı������ĵ��ٶ�
    Upsilon_OA1=((x1(1:2,j)-P1o1)/dv1o1(j))';      %��Ӧ�����е�Upsilon_OA
    JOA1=[Upsilon_OA1,zeros(1,1);zeros(1,2),1];    %С��1����������ſɱȾ���
    theta_OA1(j)=B_thetad(P1o1(1),P1o1(2),x1(1,j),x1(2,j)); %С��1��������������Ƕ�
    xOA1(:,j)=[d_OA;theta_OA1(j)];   %С��1�������������������
    VOA1(:,j)=pinv(JOA1)*OA*(xOA1(:,j)-[dv1o1(j);x1(3,j)]); %С��1��������ĵ��ٶ�
    dotx1(:,j)=VOA1(:,j)+(eye(3)-pinv(JOA1)*JOA1)*VFM1(:,j); %С��1���ٶ�
    Theta1=[cos(x1(3,j)),0;sin(x1(3,j)),0;0,1];  %С��1�ķ�����Լ������
    v1(:,j)=pinv(Theta1)*dotx1(:,j);    %С��1�Ĳ����ٶ�
    Pr1(j)=2;
    end
    x1(:,j+1)= x1(:,j)+Theta1*v1(:,j)*ts;  %С��1��λ�ø���    
    %% С��2��3�������� %Key Part
    JFM2=eye(3);                    %С��2��ӱ��ֵ��ſɱȾ���
    JFR2=eye(3);                    %С��2����ع����ſɱȾ���
    if j==1
    Pd2_FM(:,j)=Pcd(:,j)+P2cd;  %С��2�ı������λ��    
    theta_FM2(j)=0;                 %С��2������ƫ����
    dotPd2_FM=[2;0;theta_FM2(j)];      %С��2������������ٶȵ���    
    else 
    Pd2_FM(:,j)=Pcd(:,j)+P2cd;  %С��2�ı������λ��       
    theta_FM2(j)=A_thetad(Pd2_FM(1,j),Pd2_FM(2,j),x2(1,j),x2(2,j)); %����С��2��������ƫ����
    dotPd2_FM=[2;0;theta_FM2(j)];        %С��2������������ٶȵ���
    end 
    if j==1
    Pd2_FR(:,j)=Pd1_FM(:,j)+P21d;  %С��2�ı���ع�����λ��    
    theta_FR2(j)=0;                 %С��2������ƫ����
    dotPd2_FR=[2;0;theta_FR2(j)];   %С��2������������ٶȵ���    
    else  
    Pd2_FR(:,j)=Pd1_FM(:,j)+P21d;   %С��2�ı���ع�����λ��        
    theta_FR2(j)=A_thetad(Pd2_FR(1,j),Pd2_FR(2,j),x2(1,j),x2(2,j));  %����С��2��������ƫ����
    dotPd2_FR=[2;0;theta_FR2(j)];        %С��2������������ٶȵ���
    end 
    JFM3=eye(3);                    %С��3��ӱ��ֵ��ſɱȾ���
    JFR3=eye(3);                    %С��3����ع����ſɱȾ���
    if j==1
    Pd3_FM(:,j)=Pcd(:,j)+P3cd;      %С��3�ı������λ��    
    theta_FM3(j)=0;                 %С��3������ƫ����
    dotPd3_FM=[2;0;theta_FM3(j)];   %С��3������������ٶȵ���    
    else 
    Pd3_FM(:,j)=Pcd(:,j)+P3cd;  %С��3�ı������λ��       
    theta_FM3(j)=A_thetad(Pd3_FM(1,j),Pd3_FM(2,j),x3(1,j),x3(2,j));  %����С��3��������ƫ����
    dotPd3_FM=[2;0;theta_FM3(j)];        %С��3������������ٶȵ���
    end 
    if j==1
    Pd3_FR(:,j)=Pd1_FM(:,j)+P31d;   %С��3�ı���ع�����λ��    
    theta_FR3(j)=0;                 %С��3������ƫ����
    dotPd3_FR=[1;0;theta_FR3(j)];   %С��3������������ٶȵ���    
    else 
    Pd3_FR(:,j)=Pd1_FM(:,j)+P31d;   %С��3�ı���ع�����λ��        
    theta_FR3(j)=A_thetad(Pd3_FR(1,j),Pd3_FR(2,j),x3(1,j),x3(2,j));  %����С��3��������ƫ����
    dotPd3_FR=[1;0;theta_FR3(j)];        %С��3������������ٶȵ���
    end
    if dv2o4(j)<dv2o2(j)            %С��2���ĸ��ϰ����
    Po2=P2o4;
    dv2o(j)=dv2o4(j);
    else
    Po2=P2o2;
    dv2o(j)=dv2o2(j);
    end
    if dv3o5(j)<dv3o3(j)            %С��3���ĸ��ϰ����
    Po3=P3o5;
    dv3o(j)=dv3o5(j);
    else
    Po3=P3o3;
    dv3o(j)=dv3o3(j);
    end
    if j>=417 && j<=1190
    xFM2(:,j)=[Pd2_FM(:,j);theta_FM2(j)];   %С��2������������������
    VFM2(:,j)=dotPd2_FM+pinv(JFM2)*FM*(xFM2(:,j)-x2(:,j));  %С��2�ı������ĵ��ٶ�
    xFR2(:,j)=[Pd2_FR(:,j);theta_FR2(j)];   %С��2������������������
    VFR2(:,j)=dotPd2_FR+pinv(JFR2)*FR*(xFR2(:,j)-x2(:,j));  %С��2�ı������ĵ��ٶ�
    Upsilon_OA2=((x2(1:2,j)-Po2)/dv2o(j))';      %��Ӧ�����е�Upsilon_OA    
    JOA2=[Upsilon_OA2,zeros(1,1);zeros(1,2),1];    %С��2����������ſɱȾ���    
    theta_OA2(j)=B_thetad(Po2(1),Po2(2),x2(1,j),x2(2,j)); %С��2��������������Ƕ�    
    xOA2(:,j)=[d_OA;theta_OA2(j)];   %С��2�������������������
    VOA2(:,j)=pinv(JOA2)*OA*(xOA2(:,j)-[dv2o(j);x2(3,j)]); %С��2��������ĵ��ٶ�
    dotx2(:,j)=VFR2(:,j)+(eye(3)-pinv(JFR2)*JFR2)*(VFM2(:,j)+(eye(3)-pinv(JFM2)*JFM2)*VOA2(:,j)); %С��2���ٶ�
    Theta2=[cos(x2(3,j)),0;sin(x2(3,j)),0;0,1];  %С��2�ķ�����Լ������
    v2(:,j)=pinv(Theta2)*dotx2(:,j);    %С��2�Ĳ����ٶ�
    Pr2(j)=1;
    xFM3(:,j)=[Pd3_FM(:,j);theta_FM3(j)];   %С��3������������������
    VFM3(:,j)=dotPd3_FM+pinv(JFM3)*FM*(xFM3(:,j)-x3(:,j));  %С��3�ı������ĵ��ٶ�
    xFR3(:,j)=[Pd3_FR(:,j);theta_FR3(j)];   %С��3������������������
    VFR3(:,j)=dotPd3_FR+pinv(JFR3)*FR*(xFR3(:,j)-x3(:,j));  %С��3�ı������ĵ��ٶ�
    Upsilon_OA3=((x3(1:2,j)-Po3)/dv3o(j))';      %��Ӧ�����е�Upsilon_OA    
    JOA3=[Upsilon_OA3,zeros(1,1);zeros(1,2),1];    %С��3����������ſɱȾ���    
    theta_OA3(j)=B_thetad(Po3(1),Po3(2),x3(1,j),x3(2,j)); %С��3��������������Ƕ�    
    xOA3(:,j)=[d_OA;theta_OA3(j)];   %С��2�������������������
    VOA3(:,j)=pinv(JOA3)*OA*(xOA3(:,j)-[dv3o(j);x3(3,j)]); %С��3��������ĵ��ٶ�                   
    dotx3(:,j)=VFR3(:,j)+(eye(3)-pinv(JFR3)*JFR3)*(VFM3(:,j)+(eye(3)-pinv(JFM3)*JFM3)*VOA3(:,j)); %С��3���ٶ�
    Theta3=[cos(x3(3,j)),0;sin(x3(3,j)),0;0,1];  %С��3�ķ�����Լ������
    v3(:,j)=pinv(Theta3)*dotx3(:,j);    %С��3�Ĳ����ٶ�
    Pr3(j)=1;
    else        
    xFM2(:,j)=[Pd2_FM(:,j);theta_FM2(j)];   %С��2������������������
    VFM2(:,j)=dotPd2_FM+pinv(JFM2)*FM*(xFM2(:,j)-x2(:,j));  %С��2�ı������ĵ��ٶ�
    xFR2(:,j)=[Pd2_FR(:,j);theta_FR2(j)];   %С��2������������������
    VFR2(:,j)=dotPd2_FR+pinv(JFR2)*FR*(xFR2(:,j)-x2(:,j));  %С��2�ı������ĵ��ٶ�
    Upsilon_OA2=((x2(1:2,j)-P2o2)/dv2o2(j))';      %��Ӧ�����е�Upsilon_OA    
    JOA2=[Upsilon_OA2,zeros(1,1);zeros(1,2),1];    %С��2����������ſɱȾ���    
    theta_OA2(j)=B_thetad(P2o2(1),P2o2(2),x2(1,j),x2(2,j)); %С��2��������������Ƕ�    
    xOA2(:,j)=[d_OA;theta_OA2(j)];   %С��2�������������������
    VOA2(:,j)=pinv(JOA2)*OA*(xOA2(:,j)-[dv2o2(j);x2(3,j)]); %С��2��������ĵ��ٶ�        
    dotx2(:,j)=VFM2(:,j)+(eye(3)-pinv(JFM2)*JFM2)*(VFR2(:,j)+(eye(3)-pinv(JFR2)*JFR2)*VOA2(:,j)); %С��2���ٶ�
    Theta2=[cos(x2(3,j)),0;sin(x2(3,j)),0;0,1];  %С��2�ķ�����Լ������
    v2(:,j)=pinv(Theta2)*dotx2(:,j);    %С��2�Ĳ����ٶ�
    Pr2(j)=0;
    xFM3(:,j)=[Pd3_FM(:,j);theta_FM3(j)];   %С��3������������������
    VFM3(:,j)=dotPd3_FM+pinv(JFM3)*FM*(xFM3(:,j)-x3(:,j));  %С��3�ı������ĵ��ٶ�
    xFR3(:,j)=[Pd3_FR(:,j);theta_FR3(j)];   %С��3������������������
    VFR3(:,j)=dotPd3_FR+pinv(JFR3)*FR*(xFR3(:,j)-x3(:,j));  %С��3�ı������ĵ��ٶ�
    Upsilon_OA3=((x3(1:2,j)-P3o3)/dv3o3(j))';      %��Ӧ�����е�Upsilon_OA    
    JOA3=[Upsilon_OA3,zeros(1,1);zeros(1,2),1];    %С��3����������ſɱȾ���    
    theta_OA3(j)=B_thetad(P3o3(1),P3o3(2),x3(1,j),x3(2,j)); %С��3��������������Ƕ�    
    xOA3(:,j)=[d_OA;theta_OA3(j)];   %С��2�������������������
    VOA3(:,j)=pinv(JOA3)*OA*(xOA3(:,j)-[dv3o3(j);x3(3,j)]); %С��3��������ĵ��ٶ�       
    dotx3(:,j)=VFM3(:,j)+(eye(3)-pinv(JFM3)*JFM3)*(VFR3(:,j)+(eye(3)-pinv(JFR3)*JFR3)*VOA3(:,j)); %С��3���ٶ�        
    Theta3=[cos(x3(3,j)),0;sin(x3(3,j)),0;0,1];  %С��3�ķ�����Լ������
    v3(:,j)=pinv(Theta3)*dotx3(:,j);    %С��3�Ĳ����ٶ�
    Pr3(j)=0;
    end
    x2(:,j+1)= x2(:,j)+Theta2*v2(:,j)*ts;  %С��2��λ�ø���
    x3(:,j+1)= x3(:,j)+Theta3*v3(:,j)*ts;  %С��2��λ�ø���
    %% ���ĵ�������
    xc(:,j+1)=(x1(:,j+1)+x2(:,j+1)+x3(:,j+1))/3;  %С�����ĵ�λ�ø���   
    end
toc



%% ��ͼ
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


%% ����
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
function minpose2 = min_distance_obs2(a2, b2, r21,r22, inter2, px_cur2, py_cur2)  %�����ƶ�ʱ��ƫ����
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
function theta_FM=A_thetad(Pxd2,Pyd2,Px,Py)  %�����ƶ�ʱ��ƫ����
if abs(Pyd2-Py)>0.03
theta_FM=atan2(Pyd2-Py,Pxd2-Px);
else
theta_FM=0;
end
end
function theta_OA=B_thetad(Px,Py,Pox,Poy)   %�������ʱ��ƫ����
if Py>Poy
theta_OA=atan2(Poy-Py,Pox-Px)+pi/2;
else if Py<=Poy
theta_OA=atan2(Poy-Py,Pox-Px)-pi/2;
end
end
end

