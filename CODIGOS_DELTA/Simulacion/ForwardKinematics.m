function [ r ] = ForwardKinematics( t1,t2,t3,param )
%(t1 t2 t3) -> (x0 y0 z0)
%contenedor de parametros: [r_f,r_e,f,e,z=symbolic valor]

%% parametros
%longitud:
r_f=param(1); %in m
r_e=param(2);

%longitud del triangulo:
f=param(3); %en mm
e=param(4);

%efector final
z=param(5);

%% init
%virtualizacion del conector:
k=(f-e)/(2*sqrt(3));
J1=[0;-k-r_f*cos(t1);-r_f*sin(t1)]; %x1=0->pproyeccion en el plano YZ
% J2=[((f-e)/(2*sqrt(3))+r_f*cos(t2))*cos(30*pi/180);...
%     ((f-e)/(2*sqrt(3))+r_f*cos(t2))*sin(30*pi/180);...
%     -r_f*sin(t2)];
% J3=[-((f-e)/(2*sqrt(3))+r_f*cos(t3))*cos(30*pi/180);...
%     ((f-e)/(2*sqrt(3))+r_f*cos(t3))*sin(30*pi/180);...
%     -r_f*sin(t3)];
J2=[(k+r_f*cos(t2))*cos(30*pi/180);...
    (k+r_f*cos(t2))*sin(30*pi/180);...
    -r_f*sin(t2)];
J3=[(-k-r_f*cos(t3))*cos(30*pi/180);...
    (k+r_f*cos(t3))*sin(30*pi/180);...
    -r_f*sin(t3)];

%constante proporcional para el efector final
w1=J1(1)^2 + J1(2)^2 + J1(3)^2;
w2=J2(1)^2 + J2(2)^2 + J2(3)^2;
w3=J3(1)^2 + J3(2)^2 + J3(3)^2;

%calculo J1 de posicion
x1=J1(1); x2=J2(1); x3=J3(1);
y1=J1(2); y2=J2(2); y3=J3(2);
z1=J1(3); z2=J2(3); z3=J3(3);

d=(y2-y1)*x3-(y3-y1)*x2;
a1=(1/d)*((z2-z1)*(y3-y1)-(z3-z1)*(y2-y1));
b1=(-1/(2*d))*((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1));
a2=(-1/d)*((z2-z1)*x3-(z3-z1)*x2);
b2=(1/(2*d))*((w2-w1)*x3-(w3-w1)*x2);

%% numerica
%resolucion ecuacion cuadratica!
a=(a1^2+a2^2+1);
b=2*(a1*b1+a2*(b2-y1)-z1);
c=(b1^2+(b2-y1)^2 +z1^2-r_e^2);
z=(min(solve(a*z^2+b*z+c==0,z)));
%insertar z:
x=@(z)a1*z+b1;
y=@(z)a2*z+b2;

%vector de retorno de posicion
r=eval([x(z),y(z),z]');

end

