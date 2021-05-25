%%FKT
  
% =========================================================================
% Authors: © Ole Madsen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%UR5 parameters and representation of forward kinematics%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
L1=Link('d',0,'a',0,'alpha',0,'modified');
L2=Link('d',0,'a',0,'alpha',90*pi/180,'offset',90/180*pi,'modified');
L3=Link('d',0,'a',425,'alpha',0,'modified');
L4=Link('d',109,'a',392.430,'alpha',0,'offset',-90/180*pi,'modified');
L5=Link('d',93.65,'a',0,'alpha',-90*pi/180,'modified');
L6=Link('d',0,'a',0,'alpha',90*pi/180,'modified');
 
UR_5=SerialLink([L1 L2 L3 L4 L5 L6],'name','UR5');
 
 
%% Frames for transformation between Base -> {0} and {6} -> Wrist
TB_0=[1 0 0  0;
      0 1 0  0;
      0 0 1 89.2;
      0 0 0  1];
 
 T6_W=[-1 0 0  0;
       0 -1 0  0;
       0 0 1  82;
       0 0 0  1];
 
 
%Mini test:
Qtest=[110 90+90 30 -30+90 60 20].*pi/180;
 
T0_6=cast(UR_5.fkine(Qtest),'like',T6_W)
TB_W=TB_0*T0_6*T6_W
 
%T06target = inv(TB_0)*TB_W*inv(T6_W)
%transl(-521.411,-256.142,-419.593)*trotz(-146.141*pi/180)*troty(-18.747*pi/180)*trotx(-143.994*pi/180)
%inv(TB_0)*TB_W*inv(T6_W)
 
RPY=tr2rpy(TB_W,'zyx')*180/pi;
 
URpos = [TB_W(1,4) TB_W(2,4) TB_W(3,4) RPY(1) RPY(2) RPY(3)];
 
 
  
 
 
%% Section1
% Make input
 
%TBTtarget = transl(-521.411,-256.142,-419.593)*trotz(-146.141*pi/180)*troty(-18.747*pi/180)*trotx(-143.994*pi/180)
TBTtarget=TB_W;



% % % Tranformations matrix fra B til T fra RoboDK %%%%%% for 10, 20, 30....
% [    -0.786357,    -0.607604,     0.111619,  -521.411309 ;
%      -0.527587,     0.566511,    -0.633022,  -256.142081 ;
%       0.321394,    -0.556670,    -0.766044,  -419.593026 ;
%       0.000000,     0.000000,     0.000000,     1.000000 ];

 
%% Inverse kinamatics
 
%% Set basic data:
 
TB0=[1 0 0  0;
      0 1 0  0;
      0 0 1 89.2;
      0 0 0  1];
 
T6W= [-1 0 0   0;
       0 -1 0  0;
       0 0  1 82;
       0 0 0  1];
  
TWT=eulerZYX2T(0,0,0,0,0,0);


 
%% compute To6target
T06target = inv(TB0)*TBTtarget*inv(TWT);
% T06target =[T06targetp(1,1)  T06targetp(1,2)   T06targetp(1,3)  T06targetp(1,4);
%           T06targetp(2,1)    T06targetp(2,2)   T06targetp(2,3)  T06targetp(2,4);
%           T06targetp(3,1)    T06targetp(3,2)   T06targetp(3,3)  T06targetp(3,4);
%            0                       0                0                1.0000];
%  
%% compute theta1
P05=T06target*[0;0;-82;1];
phi1=atan2(P05(2),P05(1));
phi2=acos(109/sqrt(P05(2)^2+P05(1)^2));
 
theta1 = phi1+phi2+pi/2;
 
%%%%Calculating Theta5
theta5=acos((T06target(1,4)*sin(theta1)-T06target(2,4)*cos(theta1)-109)/82);

%Calculating Theta6
T60 = inv(T06target);
theta6= atan2((-T60(2,1)*sin(theta1) + T60(2,2)*cos(theta1))/sin(theta5), (T60(1,1)*sin(theta1)-T60(1,2)*cos(theta1))/sin(theta5));
 
%Calculating Theta3
T01 = cast(L1.A(theta1),'like',T60);
T45 = cast(L5.A(theta5),'like',T60);
T56 = cast(L6.A(theta6),'like',T60);
 
T10 = inv(T01);
T54 = inv(T45);
T65 = inv(T56);
 
T14 = T10*T06target*inv(T6W)*T65*T54;
% T14=[T14p(1,1)    T14p(1,2)   T14p(1,3)  T14p(1,4);
%      T14p(2,1)    T14p(2,2)   T14p(2,3)  T14p(2,4);
%      T14p(3,1)    T14p(3,2)   T14p(3,3)  T14p(3,4);
%          0          0           0          1.0000]
K1= 425;
K2= 392.430;
K3 = sqrt(T14(1,4)^2+T14(3,4)^2);
 
theta3 = acos((K3^2-K1^2-K2^2)/(2*K1*K2));

%Calculating theta2

phi3 = atan2(-T14(3,4),-T14(1,4));
phi4 = asin((-K1*sin(theta3))/K3);

theta2=phi3-phi4;


% theta2 = atan2(-T14(3,4),-T14(1,4))-asin((-K1*sin(theta3))/K3)


% Calculating theta4
T12 = cast(L2.A(theta2),'like',T60);
T23 = cast(L3.A(theta3),'like',T60);

T31g= T12*T23;
T31= inv(T31g);
T34= T31*T14;

theta4= atan2(T34(2,1), T34(1,1));
 
theta1DEG=theta1*180/pi
theta2DEG=theta2*180/pi
theta3DEG=theta3*180/pi
theta4DEG=theta4*180/pi 
theta5DEG=theta5*180/pi
theta6DEG=theta6*180/pi