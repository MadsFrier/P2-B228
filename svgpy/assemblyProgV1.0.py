#libs
from robolink import *
from robodk import *

import PySimpleGUI as sg

import svgpy.engraving as eng

#import os.path
import time

#GUI setup
relevantpath = False

sg.theme('DarkBlack')

selectBottomCoverColor = (
    'Black', 'White', 'Blue')

selectFuse = (
    '0', '1 (bottom)', '1 (top)', '2')

selectTopCoverType = (
    'Flat', 'Curved')

selectTopCoverColor = (
    'Black', 'White', 'Blue')

width = max(map(len, selectBottomCoverColor))+1

layout = [
    [sg.Text("Select bottom cover color")], [sg.Combo(selectBottomCoverColor, size=(width, 5), enable_events=True, key='SBCC')],
    [sg.Text("Select amount of fuses")], [sg.Combo(selectFuse, size=(width, 5), enable_events=True, key='SAOF')],
    [sg.Text("Select top cover type")], [sg.Combo(selectTopCoverType, size=(width, 5), enable_events=True, key='STCT')],
    [sg.Text("Select top cover color")], [sg.Combo(selectTopCoverColor, size=(width, 5), enable_events=True, key='STCC')],
    [sg.Text("Choose engraving as text and/or an image")], [sg.Input(do_not_clear=True, enable_events=True, key='textbox')],
    [sg.Button("Select image", key='SI')],
    [sg.Button("Place order", key='PO')],
]

window = sg.Window("Dummy Phone Order", layout, finalize=True)

#Main loop for GUI
while True:
    event, values = window.read()
    #print(event, values)

    if len(values["textbox"])>10:
        window.Element('textbox').Update(values['textbox'][:-1])

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'SI':
        imagePath = sg.popup_get_file("ALL FILES")
        relevantpath = True
        #if we cancel = dont engrave


    if event == 'PO':
        #if values['SBCC'] is "" or values['SAOF'] is "" or values['STCT'] is "" or values['STCC'] is "":
        #    sg.popup_error(f'Fill everything out!')
        #else:
            #Place order
            break

window.close()

##################################################

#Robolink instantiations
RL = Robolink()

robot = RL.Item('UR5')
gripper = RL.Item('gripper')
progReset = RL.Item('Reset')

home = RL.Item('home')

bottomCoverTarget = RL.Item('bottomCover')

#fuseTarget = RL.Item('fuse')

PCBTarget = RL.Item('PCB')

topCurvedCoverTarget = RL.Item('curvedCover')
topFlatCoverTarget = RL.Item('flatCover')

bottomCoverCarrierTarget = RL.Item('bottomCarrier')

topCoverCarrierTarget = RL.Item('topCover')

coverFrame = RL.Item('coverFrame') 
carrierFrame = RL.Item('carrierFrame')
PCBFrame = RL.Item('PCBFrame')
fuseFrame = RL.Item('fuseFrame')
coverTurnFrame = RL.Item('coverTurnFrame')
#Cover instances

#Bottom instantiations
blackBottomCover = RL.Item('blackBottomCover')
whiteBottomCover = RL.Item('whiteBottomCover')
blueBottomCover = RL.Item('blueBottomCover')

#Curved top instantiations
blackCurvedCover = RL.Item('blackCurvedCover')
whiteCurvedCover = RL.Item('whiteCurvedCover')
blueCurvedCover = RL.Item('blueCurvedCover')

#Flat top instantiations
blackFlatCover = RL.Item('blackFlatCover')
whiteFlatCover = RL.Item('whiteFlatCover')
blueFlatCover = RL.Item('blueFlatCover')

#PCB and fuse instantiations
PCBPart = RL.Item('PCBPart')
fusePart1 = RL.Item('fusePart1')
fusePart2 = RL.Item('fusePart2')
fuseTarget = RL.Item('fuseTarget')
fusePlace1 = RL.Item('fusePlace1')
fusePlace2 = RL.Item('fusePlace2')

dropCoverTarget = RL.Item('dropCover')
grabCoverTarget = RL.Item('grabCover')

robot.setSpeed(200)
robot.MoveJ(home)
progReset.RunProgram()
progReset.WaitFinished()

#Gripper close distance
def setGripper(closeDistance):
    gripper.setJoints([float(closeDistance)])

def attachCovers(coverType, frame, color):
    if coverType == 'bottom':
        if color == 0:
            blackBottomCover.setParentStatic(RL.Item(frame))
        elif color == 1:
            whiteBottomCover.setParentStatic(RL.Item(frame))
        elif color == 2:
            blueBottomCover.setParentStatic(RL.Item(frame))
            
    elif coverType == 'Curved':
        isCurved = True
        if color == 0:
            blackCurvedCover.setParentStatic(RL.Item(frame))
            coverToEngrave = blackCurvedCover
        elif color == 1:
            whiteCurvedCover.setParentStatic(RL.Item(frame))
            coverToEngrave = whiteCurvedCover
        elif color == 2:
            blueCurvedCover.setParentStatic(RL.Item(frame))
            coverToEngrave = blueCurvedCover
    elif coverType == 'Flat':
        isCurved = False
        if color == 0:
            blackFlatCover.setParentStatic(RL.Item(frame))
            coverToEngrave = blackFlatCover
        elif color == 1:
            whiteFlatCover.setParentStatic(RL.Item(frame))
            coverToEngrave = whiteFlatCover
        elif color == 2:
            blueFlatCover.setParentStatic(RL.Item(frame))
            coverToEngrave = blueFlatCover

#Function for placing right cover
def getCovers(coverType):
    setGripper(85)
    robot.setPoseFrame(coverFrame)
    colorToInt = {
        "Black": 0,
        "White": 1,
        "Blue": 2
    }
    if coverType == 'bottom':
        color = colorToInt[values['SBCC']]
        approach = bottomCoverTarget.Pose()*transl(200*color, 0, -236)
        grip = bottomCoverTarget.Pose()*transl(200*color, 0, -136)
    elif coverType == 'Curved':
        color = colorToInt[values['STCC']]
        print(color)
        print(values['STCC'])
        approach = topCurvedCoverTarget.Pose()*transl(200*color, 0, -236)
        grip = topCurvedCoverTarget.Pose()*transl(200*color, 0, -136)
    elif coverType == 'Flat':
        color = colorToInt[values['STCC']]
        approach = topFlatCoverTarget.Pose()*transl(200*color, 0, -236)
        grip = topFlatCoverTarget.Pose()*transl(200*color, 0, -136)
        
    robot.MoveL(approach)
    robot.MoveL(grip)
    time.sleep(1)
    setGripper(60)

    #Attaches appropriate cover to gripper
    attachCovers(coverType, 'gripper', color)
    time.sleep(1)
    robot.MoveL(approach)
    return color
    

def coverToCarrier(coverType, color):
    
    if coverType == 'bottom':
        robot.setPoseFrame(carrierFrame)
        approachCarrierTarget = bottomCoverCarrierTarget.Pose()*transl(0, 0, -236)
        grip = bottomCoverCarrierTarget.Pose()*transl(0, 0, -136)
    else:
        approachCarrierTarget = topCoverCarrierTarget.Pose()*transl(0, 0, -236)
        grip = topCoverCarrierTarget.Pose()*transl(0, 0, -136)
    
    robot.MoveL(approachCarrierTarget)
    robot.MoveL(grip)
    time.sleep(1)
    setGripper(85)

    # attaches apropriate cover to gripper
    attachCovers(coverType, 'carrierFrame', color)
    
    robot.MoveL(approachCarrierTarget)

    return color


#Function for placing PCB
def getPCB(color):
    robot.setPoseFrame(PCBFrame)

    approach = PCBTarget.Pose()*transl(0, 0, -254)
    grip = PCBTarget.Pose()*transl(0, 0, -154)

    robot.MoveL(approach)
    robot.MoveL(grip)
    time.sleep(1)
    setGripper(55)

    #Attaches appropriate cover to gripper
    PCBPart.setParentStatic(gripper)
    time.sleep(1)
    robot.MoveL(approach)

    robot.setPoseFrame(carrierFrame)
    approachCarrierTarget = topCoverCarrierTarget.Pose()*transl(0, 0, -236)
    grip = topCoverCarrierTarget.Pose()*transl(0, 0, -136)

    robot.MoveL(approachCarrierTarget)
    robot.MoveL(grip)
    time.sleep(1)
    setGripper(85)

    if color == 0:
        PCBPart.setParentStatic(blackBottomCover)
    elif color == 1:
        PCBPart.setParentStatic(whiteBottomCover)
    elif color == 2:
        PCBPart.setParentStatic(blueBottomCover)

    PCBPart.setPose(transl(2, -4.86, 3.215))

    # attaches apropriate cover to gripper
    robot.MoveL(approachCarrierTarget)

secondFuse = False
def getFuse(fuse1, fuse2 , secondFuse):
    if fuse1 == True or fuse2 == True:
        setGripper(85)
        robot.setPoseFrame(fuseFrame)
    
        approach = fuseTarget.Pose()*transl(0, 0, -200)
        grip = fuseTarget.Pose()*transl(0, 0, -161)

        robot.MoveL(approach)
        robot.MoveL(grip)
        time.sleep(1)
        setGripper(4.5)
        if secondFuse == False:
            fusePart1.setParentStatic(gripper)
        else:
            fusePart2.setParentStatic(gripper)
        
        time.sleep(1)
        robot.MoveL(approach)
        if secondFuse == False:
            fusePart2.setPose(fusePart2.Pose()*transl(0,0,-5))

        robot.setPoseFrame(carrierFrame)
        if fuse1 == True:
            approach = fusePlace1.Pose()*transl(0, 0, -200)
            grip = fusePlace1.Pose()*transl(0, 0, -161)
            fuse1 = False
        else:
            approach = fusePlace2.Pose()*transl(0, 0, -200)
            grip = fusePlace2.Pose()*transl(0, 0, -161)
            fuse2 = False

        robot.MoveL(approach)
        robot.MoveL(grip)
        if secondFuse == False:
            fusePart1.setParentStatic(PCBPart)
        else:
            fusePart2.setParentStatic(PCBPart)
           
        time.sleep(1)
        setGripper(85)
        time.sleep(1)
        robot.MoveL(approach)
        secondFuse = True
        return fuse1, fuse2, secondFuse
    else:
        return False, False, False

#flip phone
def flipaDaTable(coverType, color):
    robot.setPoseFrame(coverTurnFrame)

    approach = dropCoverTarget.Pose()*transl(0,0, -136)
    drop = dropCoverTarget.Pose()*transl(0,0,0)
    robot.MoveJ(approach)
    robot.MoveL(drop)

    setGripper(85)
    attachCovers(coverType, 'coverTurnFrame', color)
    time.sleep(1)

    approach = grabCoverTarget.Pose()*transl(0,0, -136)
    grab = grabCoverTarget.Pose()*transl(0,0,0)
    robot.MoveJ(approach)
    robot.MoveL(grab)

    setGripper(60)
    attachCovers(coverType, 'gripper', color)
    time.sleep(1)


    

color = getCovers('bottom')
coverToCarrier('bottom', color)
#getPCB(color)


#make me pretty
if values['SAOF'] == '0':
    fuse1 = False
    fuse2 = False
elif values['SAOF'] == '1 (bottom)':
    fuse1 = True
    fuse2 = False
elif values['SAOF'] == '1 (top)':
    fuse1 = False
    fuse2 = True
else:
    fuse1 = True
    fuse2 = True

facs = getFuse(fuse1, fuse2, False)

#getFuse(facs[0], facs[1], facs[2])

robot.MoveJ(home)

#getCovers(values['STCT'])

#engrave
print(values['textbox'])
if values['textbox'] != "" and relevantpath:
    print('both')
elif values['textbox'] != "" and not relevantpath:
    eng.setup(coverToEngrave, isCurved)
    print('only text')
elif values['textbox'] == "" and relevantpath:
    print('only pic')
elif values['textbox'] == "" and not relevantpath:
    print('no engraving')

flipaDaTable(values['STCT'], color)
 
