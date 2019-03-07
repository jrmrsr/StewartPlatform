

[Home1,Angle_1]=CalcAngles(83.5,32.81,0,42.7,61.95,0,0);
%[Home1,Angle_1]=CalcAngles(83.5,32.81,0,38.0885,29.9711,0,0); %test
DISP1=['Home Angle 1: ',num2str(Home1),'   Tilt Angle 1: ', num2str(Angle_1)];
disp(DISP1);


[Home2,Angle_2]=CalcAngles(-13.3,88.72,0,32.3,67.95,0,120);
%[Home2,Angle_2]=CalcAngles(-13.3,88.72,0,6.9115,47.9711,0,120);
DISP2=['Home Angle 2: ',num2str(Home2),'   Tilt Angle 2: ', num2str(Angle_2)];
disp(DISP2);

% WHY OS HOME 3 XB, YB, ZB SO SIMILAR TO EACH OTHER? 
[Home3,Angle_3]=CalcAngles(-70.17,55.91,0,-75,6,0,120);
%[Home3,Angle_3]=CalcAngles(-70.17,55.91,0,-43.125,18,0,120);
DISP3=['Home Angle 3: ',num2str(Home3),'   Tilt Angle 3: ', num2str(Angle_3)];
disp(DISP3);

[Home4,Angle_4]=CalcAngles(-70.17,-55.91,0,-75,-6,0,240);
%[Home4,Angle_4]=CalcAngles(-70.17,-55.91,0,-43.125,-18,0,240);
DISP4=['Home Angle 4: ',num2str(Home4),'   Tilt Angle 4: ', num2str(Angle_4)];
disp(DISP4);

[Home5,Angle_5]=CalcAngles(-13.33,-88.71,0,32.3,-67.95,0,240);
%[Home5,Angle_5]=CalcAngles(-13.33,-88.71,0,6.9115,-47.9711,0,240);
DISP5=['Home Angle 5: ',num2str(Home5),'   Tilt Angle 5: ', num2str(Angle_5)];
disp(DISP5);

[Home6,Angle_6]=CalcAngles(83.5,-32.81,0,42.7,-61.95,0,0);
%[Home6,Angle_6]=CalcAngles(83.5,-32.81,0,38.0885,-29.9711,0,0);
DISP6=['Home Angle 6: ',num2str(Home6),'   Tilt Angle 6: ', num2str(Angle_6)];
disp(DISP6);






function [alphao, alpha] = CalcAngles(xb,yb,zb,xp,yp,zp,Beta)


%%lengths 
s= 90; %177.4;%%linkage length
a= 35 ; %26.67; %28.33;   %35;%%servo arm length


%%angles
theta=deg2rad(0);  %%theta (rotation about y)
phi=deg2rad(0);   %% Q (rotation about x)
psi=deg2rad(0);    %%Cactus (rotation about z)

%{
% Jose Is dumb
gain =1.75;
xb=xb*gain;
yb=yb*gain;
zb=zb*gain;
xp=xp*gain;
yp=yp*gain;
zp=zp*gain;
%}


%{
Beta=30; %%angle of x axis to plane of servo arm rotation

%%Points for linkage top
xp=23.8;
yp=50;
zp=0;


%%points centre of rotation of motors
xb=-13.3;
yb=88.72;
zb=0;         %30.6;
%}


%%%%height when at home
ho= sqrt(s*s+a*a-(xp-xb)^2-(yp-yb)^2)-zp;




%%points for top coordinate system  
xt=0;
yt=0;
zt=ho;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%psi is yaw
%theta is pitch
%asfdja;sldfkj is roll
PRB =[cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
      sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
      -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];


  
T=[xt;yt;zt]; %%Distance from centre of base to centre of platform 3x1 matrix

pi=[xp;yp;zp]; %% position of linkage top  3x1 matrix

bi=[xb; yb; zb];  %% 3x1 matrix

li=T+mtimes(PRB,pi)-bi;

qi=T+mtimes(PRB,pi);


%%%% angle for HOME POSITION%%%%%%%%%%%%
%%%assumes servo arm is prependicular to rod?
HOME=[xp; yp; ho+zp];   %%location of top end of linkage 

lo=[xp-xb; yp-yb; ho+zp];
Lo=2*a*a;
Mo=2*a*(xp-xb);
No=2*a*(ho+zp);

alphao= rad2deg(asin(Lo/(sqrt(Mo*Mo+No*No)))-atan(Mo/No));

%%% SERVO ANGLE %%%%%%%%%%%%%%%%%%%%%%%%
%L=li.^2-(s*s-a*a);
%lsquared=(xq*xq+yq*yq+zq*zq)+(xb*xb+yb*yb+zb*zb)-2*(xq*xb+yq*yb+zq*zb);
lsquared=(qi(1)*qi(1)+qi(2)*qi(2)+qi(3)*qi(3))+(xb*xb+yb*yb+zb*zb)-2*(qi(1)*xb+qi(2)*yb+qi(3)*zb);

L=lsquared-(s*s-a*a);
M=2*a*(qi(3)-zb);
N=2*a*(cos(deg2rad(Beta))*(qi(1)-xb)+sin(deg2rad(Beta))*(qi(2)-yb));

Ltest= dot(li,li);
Ltest2=-(s*s-a*a);
test= L/sqrt(M*M+N*N); % if test is greater than teh absolute value of 1, angles are imaginary

DISPTest=['Test ', num2str(test)];
disp(DISPTest);

alpha= rad2deg(asin(L/(sqrt(M*M+N*N)))-atan(N/M));

%svector=qi-(bi+);



end