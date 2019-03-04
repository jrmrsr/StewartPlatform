

[Home1,Angle_1]=CalcAngles(0,0,0,0,0,0,0);
DISP1=['Home Angle 1: ',num2str(Home1),'   Tilt Angle 1: ', num2str(Angle_1)];
disp(DISP1);

[Home2,Angle_2]=CalcAngles(0,0,0,0,0,0,0);
DISP2=['Home Angle 2: ',num2str(Home2),'   Tilt Angle 2: ', num2str(Angle_2)];
disp(DISP2);

[Home3,Angle_3]=CalcAngles(0,0,0,0,0,0,0);
DISP3=['Home Angle 3: ',num2str(Home3),'   Tilt Angle 3: ', num2str(Angle_3)];
disp(DISP3);

[Home4,Angle_4]=CalcAngles(0,0,0,0,0,0,0);
DISP4=['Home Angle 4: ',num2str(Home4),'   Tilt Angle 4: ', num2str(Angle_4)];
disp(DISP4);

[Home5,Angle_5]=CalcAngles(0,0,0,0,0,0,0);
DISP5=['Home Angle 5: ',num2str(Home5),'   Tilt Angle 5: ', num2str(Angle_5)];
disp(DISP5);

[Home6,Angle_6]=CalcAngles(0,0,0,0,0,0,0);
DISP6=['Home Angle 6: ',num2str(Home6),'   Tilt Angle 6: ', num2str(Angle_6)];
disp(DISP6);



function [alphao, alpha] = CalcAngles(xb,yb,zb,xp,yp,zp,Beta)

%%lengths 
s=0;%%linkage length
a=0;%%servo arm length

%%angles
Pitch=0;  %%theta
Roll=0;   %% Q
Yaw=0;    %%Cactus


%Beta=0; %%angle of x axis to plane of servo arm rotation

%%Points for linkage top
%xp=0;
%yp=0;
%zp=0;


%%points centre of rotation of motors
%xb=0;
%yb=0;
%zb=0;

%%%%height when at home
ho= sqrt(s*s+a*a-(xp-xb)^2-(yp-yb)^2)-zp;

%%points for top coordinate system  
xt=0;
yt=0;
zt=ho;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PRB =[cos(Yaw)*cos(Pitch), -sin(Yaw)*cos(Roll)+cos(Yaw)*sin(Pitch)*sin(Roll), sin(Yaw)*sin(Roll)+cos(Yaw)*sin(Pitch)*cos(Roll);
      sin(Yaw)*cos(Pitch), cos(Yaw)*cos(Roll)+sin(Yaw)*sin(Pitch)*sin(Roll), -cos(Yaw)*sin(Roll)+sin(Yaw)*sin(Pitch)*cos(Roll);
      -sin(Pitch), cos(Pitch)*sin(Roll), cos(Pitch)*cos(Roll)];


  
T=[xt;yt;zt]; %%Distance from centre of base to centre of platform 3x1 matrix

pi=[xp;yp;zp]; %% position of linkage top  3x1 matrix

bi=[xb; yb; zb];  %% 3x1 matrix

li=T+PRB*pi-bi;


%%%% angle for HOME POSITION%%%%%%%%%%%%

HOME=[xp; yp; ho+zp];   %%location of top end of linkage 

lo=[xp-xb; yp-yb; ho+zp];

Lo=2*a*a;
Mo=2*a*(xp-xb);
No=2*a*(ho+zp);

alphao= asin(Lo/(sqrt(Mo*Mo+No*No))-atan(Mo/No));


%%% SERVO ANGLE %%%%%%%%%%%%%%%%%%%%%%%%

L=li.^2-(s*s-a*a);
M=2*a*(zp-zb);
N=2*a*(cos(Beta)*(xp-xb)+sin(Beta)*(yp-yb));

alpha= asin(L/(sqrt(M*M+N*N))-atan(N/M));
end