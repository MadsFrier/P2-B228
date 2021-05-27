L1=Link('d',0,'a',0,'alpha',0,'modified')
L2=Link('d',0,'a',0,'alpha',90*pi/180,'offset',90/180*pi,'modified')
L3=Link('d',0,'a',425,'alpha',0,'modified')
L4=Link('d',109,'a',392.430,'alpha',0,'offset',-90/180*pi,'modified')
L5=Link('d',93.65,'a',0,'alpha',-90*pi/180,'modified')
L6=Link('d',0,'a',0,'alpha',90*pi/180,'modified')
 
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
 
 
%Mini test:
 
Qtest=[10 20+90 30 40+90 50 60].*pi/180
 
T0_6=cast(UR_5.fkine(Qtest),'like',T6_W)
TB_W=TB_0*T0_6*T6_W
 
%Transfor to angle axis (theta+K)
theta = acos((TB_W(1,1)+TB_W(2,2)+TB_W(3,3)-1)/2)
if (abs(theta) < 0.000001)
    K = [0;0;0];
else
    K=1/(2*sin(theta))*[TB_W(3,2)-TB_W(2,3);TB_W(1,3)-TB_W(3,1);TB_W(2,1)-TB_W(1,2)]
end
 
UROri = [K(1)*theta*180/pi K(2)*theta*180/pi K(3)*theta*180/pi]
 
URpos = [TB_W(1,4) TB_W(2,4) TB_W(3,4) UROri(1) UROri(2) UROri(3)]