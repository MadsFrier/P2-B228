from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
RDK = Robolink()

#Instantiate gripper and base for paren/child relationship
gripper = RDK.Item('RobotiQ 2F85')
base = RDK.Item('UR5 Base');


#Instantiate openGripper target
openGripper = RDK.AddTarget('gripper_open', base, gripper)
#Make openGripper target a joint target (ignore target placement)
openGripper.setAsJointTarget()
#Define the open position of the gripper (parameter is in mm)
openGripper.setJoints([85])

#Instantiate closeGripper target
closeGripper = RDK.AddTarget('gripper_close', base, gripper)
#Make closeGripper target a joint target (ignore target placement)
closeGripper.setAsJointTarget()
#Define the closed position of the gripper (parameter is in mm)
closeGripper.setJoints([0])



