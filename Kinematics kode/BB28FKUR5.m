%% Startup rutine

clc
close
clear

% global robot

% Generate a Robolink object RDK. This object interfaces with RoboDK.
RDK = Robolink;

% Display the list of all items in the main tree
fprintf('Available items in the station:\n');
disp(RDK.ItemList());

robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT);

if robot.Valid() == 0
    error('No robot selected');
end

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
 
 
%RDK test
TestAngles = [11 22 33 44 55 66];
Qtest= (TestAngles + [0 90 0 90 0 0]).*pi/180;
robot.MoveJ(TestAngles);
pose = robot.Pose();
eul = tform2eul(robot.Pose(), 'zyx').*180/pi;
RDKPose = [pose(1,4) pose(2,4) pose(3,4) eul(1) eul(2) eul(3)]

%Matlab test
T0_6=cast(UR_5.fkine(Qtest),'like',T6_W);
TB_W=TB_0*T0_6*T6_W;
eul = tform2eul(TB_W, 'zyx').*180/pi;
CalcPose = [TB_W(1,4) TB_W(2,4) TB_W(3,4) eul(1) eul(2) eul(3)]

distance = 0;
for i = length(CalcPose) 
    distance = distance + (CalcPose(i)- RDKPose(i))^2
end 