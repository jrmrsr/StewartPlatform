%{
[Home1,Angle_1]=CalcAngles(83.5,32.81,30.6,42.7,61.95,0,0);
DISP1=['Home Angle 1: ',num2str(Home1),'   Tilt Angle 1: ', num2str(Angle_1)];
disp(DISP1);

[Home2,Angle_2]=CalcAngles(-13.3,88.72,30.6,32.3,67.95,0,30);
DISP2=['Home Angle 2: ',num2str(Home2),'   Tilt Angle 2: ', num2str(Angle_2)];
disp(DISP2);

[Home3,Angle_3]=CalcAngles(-70.17,55.91,30.6,-75,6,0,30);
DISP3=['Home Angle 3: ',num2str(Home3),'   Tilt Angle 3: ', num2str(Angle_3)];
disp(DISP3);

[Home4,Angle_4]=CalcAngles(-70.17,-55.91,30.6,-75,-6,0,330);
DISP4=['Home Angle 4: ',num2str(Home4),'   Tilt Angle 4: ', num2str(Angle_4)];
disp(DISP4);

[Home5,Angle_5]=CalcAngles(-13.33,-88.71,30.6,32.3,-67.95,0,330);
DISP5=['Home Angle 5: ',num2str(Home5),'   Tilt Angle 5: ', num2str(Angle_5)];
disp(DISP5);

[Home6,Angle_6]=CalcAngles(83.5,-32.81,30.6,42.7,-61.95,0,0);
DISP6=['Home Angle 6: ',num2str(Home6),'   Tilt Angle 6: ', num2str(Angle_6)];
disp(DISP6);
%}

%function [alphao, alpha] = CalcAngles(xb,yb,zb,xp,yp,zp,Beta)

%% Lengths 
s= 177.4 %140 % 177.4; %% linkage length
a= 35; % 35; %% Servo arm length

%% Angles
Pitch=0.1309;  %% Theta
Roll=0;   %% Q
Yaw=0;    %% Cactus

Beta=30; %% Angle of x axis to plane of servo arm rotation

%% Points for linkage top
xp=23.8;
yp=50;
zp=0;

%% Points centre of rotation of motors
xb=-13.3;
yb=88.72;
zb=0;         %30.6;

%% Height when at home
ho= sqrt(s*s+a*a-(xp-xb)^2-(yp-yb)^2)-zp;

%% Points for top coordinate system  
xt=0;
yt=0;
zt=ho;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PRB =[cos(Yaw)*cos(Pitch), -sin(Yaw)*cos(Roll)+cos(Yaw)*sin(Pitch)*sin(Roll), sin(Yaw)*sin(Roll)+cos(Yaw)*sin(Pitch)*cos(Roll);
      sin(Yaw)*cos(Pitch), cos(Yaw)*cos(Roll)+sin(Yaw)*sin(Pitch)*sin(Roll), -cos(Yaw)*sin(Roll)+sin(Yaw)*sin(Pitch)*cos(Roll);
      -sin(Pitch), cos(Pitch)*sin(Roll), cos(Pitch)*cos(Roll)];


  
T=[xt;yt;zt]; %% Distance from centre of base to centre of platform 3x1 matrix

pi=[xp;yp;zp]; %% Position of linkage top 3x1 matrix

bi=[xb; yb; zb]; %% 3x1 matrix

li=T+PRB*pi-bi;

qi=T+PRB*pi;

%% Angle for home position

HOME=[xp; yp; ho+zp]; %% Location of top end of linkage 

lo=[xp-xb; yp-yb; ho+zp];

Lo=2*a*a;
Mo=2*a*(xp-xb);
No=2*a*(ho+zp);

alphao= asin(Lo/(sqrt(Mo*Mo+No*No))-atan(Mo/No));

%% Servo Angle 

%L=li.^2-(s*s-a*a);

%lsquared=(xq*xq+yq*yq+zq*zq)+(xb*xb+yb*yb+zb*zb)-2*(xq*xb+yq*yb+zq*zb);
lsquared=(qi(1)*qi(1)+qi(2)*qi(2)+qi(3)*qi(3))+(xb*xb+yb*yb+zb*zb)-2*(qi(1)*xb+qi(2)*yb+qi(3)*zb);

L=lsquared-(s*s-a*a);
M=2*a*(zp-zb);
N=2*a*(cos(Beta)*(xp-xb)+sin(Beta)*(yp-yb));

Ltest= dot(li,li);
Ltest2=-(s*s-a*a);
test= L/sqrt(M*M+N*N);

alpha= asin(L/(sqrt(M*M+N*N)))-atan(N/M);
