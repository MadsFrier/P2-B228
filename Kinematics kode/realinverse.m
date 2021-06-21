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
       0 0 1 82;
       0 0 0  1];
%%
%Mini test:
Qtest=[11 22+90 33 44+90 55 66];

%Target = transl(400.497,-81.105,-478.248)*trotz(-8.540)*troty(5.021)*trotx(35.690) % The target takes a 4X4 transform matrix 

T0_6= cast(UR_5.fkine(Qtest),'like',T6_W);
%%
TB_W=TB_0*T0_6*T6_W;

T06target = inv(TB_0)*TB_W*inv(T6_W);
 
RPY=tr2rpy(TB_W,'zyx')*180/pi;
 
URpos = [TB_W(1,4) TB_W(2,4) TB_W(3,4) RPY(1) RPY(2) RPY(3)];
%% Section1 
% Make input

%TBTtarget = transl(-521.411,-256.142,-419.593)*trotz(-146.141*pi/180)*troty(-18.747*pi/180)*trotx(-143.994*pi/180)
TBTtarget=TB_W;

%% Set basic data:

TB0=[1 0 0  0;
      0 1 0  0;
      0 0 1 89.2;
      0 0 0  1];
  
 T6W=[-1 0 0  0;
       0 -1 0  0;
       0 0 1 82;
       0 0 0  1];
   
TWT= eul2tform([0 0 0]); %eulerZYX2T(0,0,0,0,0,0);

%% compute To6target
T06target = inv(TB0)*TBTtarget*inv(TWT);

%% compute theta1
P05=T06target*[0;0;-82;1];
phi1=atan2(P05(2),P05(1));
phi2=acos(109/sqrt(P05(2)^2+P05(1)^2));

theta1(1) = phi1+phi2+pi/2;
theta1(2) = phi1-phi2+pi/2;
%% 

theta5(1,1)= acos((T06target(1,4)*sin(theta1(1))-T06target(2,4)*cos(theta1(1))-109)/82);
theta5(2,1)= acos((T06target(1,4)*sin(theta1(2))-T06target(2,4)*cos(theta1(2))-109)/82);
theta5(1,2)= -acos((T06target(1,4)*sin(theta1(1))-T06target(2,4)*cos(theta1(1))-109)/82);
theta5(2,2)= -acos((T06target(1,4)*sin(theta1(2))-T06target(2,4)*cos(theta1(2))-109)/82);
%% 

T60 = inv(T06target);
theta6(1,1)= atan2((-T60(2,1)*sin(theta1(1)) + T60(2,2)*cos(theta1(1)))/sin(theta5(1,1)), (T60(1,1)*sin(theta1(1))-T60(1,2)*cos(theta1(1)))/sin(theta5(1,1)));
theta6(2,1)= atan2((-T60(2,1)*sin(theta1(2)) + T60(2,2)*cos(theta1(2)))/sin(theta5(1,2)), (T60(1,1)*sin(theta1(2))-T60(1,2)*cos(theta1(2)))/sin(theta5(1,2)));

theta6(1,2)= atan2((-T60(2,1)*sin(theta1(1)) + T60(2,2)*cos(theta1(1)))/sin(theta5(2,1)), (T60(1,1)*sin(theta1(1))-T60(1,2)*cos(theta1(1)))/sin(theta5(1,2)));
theta6(2,2)= atan2((-T60(2,1)*sin(theta1(2)) + T60(2,2)*cos(theta1(2)))/sin(theta5(2,2)), (T60(1,1)*sin(theta1(2))-T60(1,2)*cos(theta1(2)))/sin(theta5(2,2)));

%% 
for i = [1 2]
    for j = [1 2]
    
T01 = cast(L1.A(theta1(i)),'like',T60);
T45 = cast(L5.A(theta5(i,j)),'like',T60);
T56 = cast(L6.A(theta6(i,j)),'like',T60);
theta2 = 0;
T12 = cast(L2.A(theta2),'like',T60);

T10 = inv(T01);
T54 = inv(T45);
T65 = inv(T56);

T14 = T10*T06target*inv(T6W)*T65*T54
T15 = T14*T45;
T25 = inv(T12)*T15;
% T10*T06target*inv(T6W)*T65
L1= 425;
L2= 392.430;
L3 = sqrt(T14(1,4)^2+T14(3,4)^2);
% L4 = sqrt(T25(1,4)^2+T25(3,4)^2)
L4 = sqrt(T15(1,4)^2+T15(3,4)^2);
% sqrt(T15(1,4)^2+T15(3,4)^2+T15(2,4)^2)

L5 = 93.65;

theta3(i,j) = acos((L3^2-L1^2-L2^2)/(2*L1*L2));
theta3(i,j,2) = -acos((L3^2-L1^2-L2^2)/(2*L1*L2));
theta2(i,j) = -(acos(-(L2^2-L1^2-L3^2)/(2*L1*L3))+atan2(T14(3,4),-T14(1,4)));
theta4(i,j) = acos((L3^2-L5^2-L4^2)/(2*L3*L5));
    end
end
Possible_Solutions = [ theta1(1) theta2(1,1) theta3(1,1,1) theta4(1,1) theta5(1,1) theta6(1,1); % i = 1  j = 1 theta3_1
                       theta1(1) theta2(1,2) theta3(1,2,1) theta4(1,2) theta5(1,2) theta6(1,2); % i = 1  j = 2 theta3_1
                       theta1(1) theta2(1,1) theta3(1,1,2) theta4(1,1) theta5(1,1) theta6(1,1); % i = 1  j = 1 theta3_2
                       theta1(1) theta2(1,2) theta3(1,2,2) theta4(1,2) theta5(1,2) theta6(1,2); % i = 1  j = 2 theta3_2
                       
                       theta1(2) theta2(2,1) theta3(2,1,1) theta4(2,1) theta5(2,1) theta6(2,1); % i = 2  j = 1 theta3_1
                       theta1(2) theta2(2,2) theta3(2,2,1) theta4(2,2) theta5(2,2) theta6(2,2); % i = 2  j = 2 theta3_1
                       theta1(2) theta2(2,1) theta3(2,1,2) theta4(2,1) theta5(2,1) theta6(2,1); % i = 2  j = 1 theta3_2
                       theta1(2) theta2(2,2) theta3(2,2,2) theta4(2,2) theta5(2,2) theta6(2,2);] .*(180/pi) % i = 2  j = 2 theta3_2
                   
% acos((L2^2-L5^2-L4^2)/(2*L2*L5)) %Til T25

% acos((L3^2+L5^2-L4^2)/(2*L3*L5))+theta2+theta3;
% acos((L3^2+L5^2-L4^2)/(2*L3*L5))
% asin((L1*sin(theta3))/93.65)
% atan2(T14(3,1),T14(2,1))+theta2+theta3;

%TEST FOR COSINUSRELATION%%%
% lilleb=425+392.430
% tryout= acos((lilleb^2+L5^2-sqrt(lilleb^2+L5^2)^2)/(2*lilleb*L5))
% tryoutDEG=tryout*180/pi

%theta1DEG=theta1*180/pi
%theta2DEG=theta2*180/pi
%theta3DEG=theta3*180/pi
%theta4DEG=theta4*180/pi
% 180-vinkelB+vinkelC
%theta5DEG=theta5*180/pi
%theta6DEG=theta6*180/pi
