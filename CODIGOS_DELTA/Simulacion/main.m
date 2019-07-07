
%% inicio

clc;
clear all;
%eslabon

r_f=5e-2; % parametro f o de junta mayot  5  eslabon corto
r_e=8e-2;  % parametro e de junta menor eslabon largo
%triangulo
f=8e-2; % parametro f de junta
e=6e-2; % parametro e de junta 6

syms z real

param=[r_f,r_e,f,e,z]; % parametros guardados


m=40;
n=3; 
r0=[0,0,-0.1]; 
% puntos objetivos
rGoal=zeros(n+1,3); % objetivos
rGoal(1,:)=r0'; % objetivos
rGoal(2,:)=[-0.01,-0.03,-0.12]; %objetivos
rGoal(3,:)=[-0.01,0.025,-0.06]; % objetivos
rGoal(4,:)=[-0.05,0,-0.1];

trajectory=zeros(3,m,n);
angles=zeros(3,m,n);
for i=1:n
    trajectory(:,:,i)=CalcTrajectory(rGoal(i,:),rGoal(i+1,:),m); % calculo de trayectoria con los puntos objetivos
    angles(:,:,i)=CalcTrajectoryAngles(trajectory(:,:,i),param)*pi/180; % calculo de angulos
end

for i=1:n
    Animation(angles(:,:,i),trajectory(:,:,i),param); % animacion del movimiento 
   
    hold on
   
end
    
%%
hold on
q=40
a=0
na=1
cuent=0;
while na<=3
for a=1:q
    cuent=cuent+1
   plot3(trajectory(1,a,na),trajectory(2,a,na),trajectory(3,a,na)) 
   s(cuent,:)=[trajectory(1,a,na) trajectory(2,a,na) trajectory(3,a,na)]
   
   
    hold on
end
na=na+1
end
%%
cuent=40
while cuent>=1
    %plot3(s(cuent,1),s(cuent,2),s(cuent,3))
    scatter3(s(cuent,1),s(cuent,2),s(cuent,3),'*','green')
    cuent=cuent-1
end

%%
l1 = 10; % length of first arm
l2 = 7; % length of second arm

theta1 = 0:0.1:pi/2; % all possible theta1 values
theta2 = 0:0.1:pi/2; % all possible theta2 values
theta3 = 0:0.1:pi/2; % all possible theta2 values
[THETA1,THETA2] = meshgrid(theta1,theta2); % generate a grid of theta1 and theta2 values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset
  plot(X(:),Y(:),'r.'); 
  %%
  clear all
close all
%Cinematica directa
 r_f=5e-2; % parametro f o de junta mayot  5  eslabon corto
    r_e=8e-2;  % parametro e de junta menor eslabon largo
%triangulo
    f=8e-2; % parametro f de junta
    e=6e-2; % parametro e de junta 6

    syms z real

    param=[r_f,r_e,f,e,z]; % parametros guardados

hold on
i=1;
n=-2*pi
cont=0;
theta1 = -pi*pi:0.5:pi; % all possible theta1 values
theta2 = -pi:0.5:pi; % all possible theta2 values
theta3 = -pi:0.5:pi; % all possible theta2 values
[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3); % generate a grid of theta1 and theta2 values
%[THETA1,THETA2] = meshgrid(theta1,theta2); % generate a grid of theta1 and theta2 values
%para theta1
tam=size(THETA1);
M1 = reshape(THETA1,[tam(1)*tam(2),tam(3)]);
tama=size(M1);
N1 = reshape(M1,[tama(1)*tama(2),1]);
%para theta2
tam2=size(THETA2);
M2 = reshape(THETA2,[tam2(1)*tam2(2),tam2(3)]);
tama2=size(M2);
N2 = reshape(M2,[tama2(1)*tama2(2),1]);
%para theta3
tam3=size(THETA3);
M3 = reshape(THETA3,[tam3(1)*tam3(2),tam3(3)]);
tama3=size(M3);
N3 = reshape(M3,[tama3(1)*tama3(2),1]);

%--------------------------------------------------------------------------------%
%%
%clear all
close all
%Cinematica directa
 r_f=5e-2; % parametro f o de junta mayot  5  eslabon corto
    r_e=8e-2;  % parametro e de junta menor eslabon largo
%triangulo
    f=8e-2; % parametro f de junta
    e=6e-2; % parametro e de junta 6

    syms z real

    param=[r_f,r_e,f,e,z]; % parametros guardados

hold on
i=1;
n=1
cont=0;
%theta1 = 0:0.1:pi/2; % all possible theta1 values
%theta2 = 0:0.1:pi/2; % all possible theta2 values
%theta3 = 0:0.1:pi/2; % all possible theta2 values
k=size(N3)
while n<=k(1)
     cont=cont+1;
    st=ForwardKinematics(N1(n),N2(n),N3(n),param);
    if size(st)==0
    st=[0 0 0];
    end
    h(n,:)=st;
    h(n,:)=h(n,:)*1;
    %hold on
    %scatter3(ans(1),ans(2),ans(3))
    n=n+1;
   
end
%%


for i=1 : n
   
    %h(i,:)=ForwardKinematics(i,i,i,param)
    hold on
    scatter3(h(i,1),h(i,2),h(i,3),'*','r')
   
end

%%
Q=F;
 %scatter3(h(i,1)*100,h(i,2)*100,h(i,3)*100)
 n=1;
 r=1;
 cont=1;
 while n<=3
     while r<=4
        l(cont)=F(r,2,n);
         cont=cont+1;
         r=r+1;
     end
     r=1;
     n=n+1;
 end
 
 %%
 % dibujo de trayectorias
 plot m1
 