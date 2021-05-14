from robolink import *
from robodk import *

from svgpy.svg import *
import cairo
import sys 
import os
import re

SIZE_BOARD = [114, 60]      # Size of the image. The image will be scaled keeping its aspect ratio
BEZEL_SIZE =  6             #This is used to remove the outer bezel of the cover that will not be engraved
OffsetToCenter = [(SIZE_BOARD[0]+BEZEL_SIZE)/4, (SIZE_BOARD[1]+BEZEL_SIZE)/2]
MM_X_PIXEL = 1             # in mm. The path will be cut depending on the pixel size. If this value is changed it is recommended to scale the pixel object
IMAGE_FILE = '/World map.svg'# Path of the SVG image, it can be relative to the current RL station
TEXT_FILE = 't'

#--------------------------------------------------------------------------------
# function definitions:

def setup(coverToEngrave, isCurved):
    # delete any frames made in a previous run if any
    image = RL.Item('Frame Draw')
    if image.Valid() and image.Type() == ITEM_TYPE_FRAME: image.Delete()
    targetFrame = RL.Item('Frame Draw Abs')
    if targetFrame.Valid() and targetFrame.Type() == ITEM_TYPE_FRAME: targetFrame.Delete()
    EngravingFrame = RL.Item('Engraving Frame')
    if EngravingFrame.Valid() and EngravingFrame.Type() == ITEM_TYPE_FRAME: EngravingFrame.Delete()

    board_draw = coverToEngrave
    
    # get the robot, frame and tool objects
    robot = RL.Item('UR5')

    #engravingTool = robot.setPoseTool(transl(0,0,159))
    EngravingFrame = RL.Item('engraving') #RL.AddFrame('Engraving Frame', EngravingTool)
    robot.setPoseFrame(EngravingFrame)
    if isCurved:
        EngravingStart = RL.Item('engravingCurved')
    else:
        EngravingStart = RL.Item('engravingFlat')
    robot.MoveJ(EngravingStart)

    framedraw = RL.AddFrame('Frame Draw', EngravingFrame)
    framedraw.setVisible(True, True)
    if isCurved:
        framedraw.setPose(transl(0,0,1.5)*rotz(pi/2))
    else:
        framedraw.setPose(transl(0,0,2)*rotz(pi/2))
    framedraw.setParentStatic(board_draw)

    framedrawAbs = RL.AddFrame('Frame Draw Abs', framedraw)
    framedrawAbs.setPose(transl(0,0,-21.5))
    framedrawAbs.setParentStatic(RL.Item('UR5 Base'))
    robot.setPoseFrame(framedrawAbs)
    tooldraw = RL.Item('Gripper')

    # get the pixel reference to draw
    pixel_ref = RL.Item('Cylinder')
    pixel_ref.Copy()
    return 0
    
def ImgEngrave(IMAGE_FILE, robot, isCurved):
    # select the file to draw
    svgfile = path_stationfile + IMAGE_FILE
    # import the SVG file
    svgdata = svg_load(svgfile)

    IMAGE_SIZE = Point(SIZE_BOARD[0]/2,SIZE_BOARD[1])   # size of the image in MM
    svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)
    size_img = svgdata.size_poly()  # returns the size of the current polygon

    #We will need the center of the image, we can find it using the size of the image.
    #As we will scale the image to the cover

    #Our Path takes and svg image and- converts them to a set of path segments, which is made up of points.
    #Between each path a approach point is made.
    APPROACH = 0.2  # approach distance in MM for each path

    for path in svgdata:
    
        np = path.nPoints()
        
        # robot movement: approach to the first target
        p_0 = path.getPoint(0)
        if isCurved:
                p_0Z = -4+calcZ_coord(-p_0.y)
        else:
                p_0Z = 0
        target0 = transl(p_0.x - OffsetToCenter[0], p_0.y -OffsetToCenter[1], p_0Z)
        target0_app = target0*transl(0,0,-APPROACH)
        framedraw = RL.Item('Frame Draw')
        framedrawAbs = RL.Item('Frame Draw Abs')
        #AddTarget = RL.AddTarget('Target', framedrawAbs)
        #AddTarget.setPose(target0)
        #AddTarget_app = RL.AddTarget('App', framedrawAbs)
        #AddTarget_app.setPose(target0_app)
        robot.MoveL(target0_app)
        robot.MoveL(target0)
        for i in range(np):
            p_i = path.getPoint(i)
            p_i.x = -p_i.x + OffsetToCenter[0]
            p_i.y = -p_i.y +OffsetToCenter[1]
            v_i = path.getVector(i)
            
            if isCurved:
                p_iZ = -4+calcZ_coord(-p_i.y - 28.5)
                pt_pose = point3D_2_pose(p_i, v_i)
            else:
                p_iZ = 0
                pt_pose = point2D_2_pose(p_i, v_i)
               
            target = transl(-p_i.x, -p_i.y, p_iZ)
            #AddTarget = RL.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
        pathEnd = target*transl(0,0,-APPROACH)
    return 0

def StrEngrave(TEXT_FILE, robot, isCurved):
    # select the file to draw
    
    svgfile = path_stationfile + '/' + makeSVG(TEXT_FILE)
    # import the SVG file
    svgdata = svg_load(svgfile) 

    IMAGE_SIZE = Point(SIZE_BOARD[0]/4,SIZE_BOARD[1])   # size of the image in MM
    svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)
    size_img = svgdata.size_poly()  # returns the size of the current polygon

    #Our Path takes and svg image and- converts them to a set of path segments, which is made up of points.
    #Between each path a approach point is made.
    APPROACH = 2  # approach distance in MM for each path

    for path in svgdata:
    
        np = path.nPoints()
            
        # robot movement: approach to the first target
        p_0 = path.getPoint(0)
        if isCurved:
            p_0Z = -4+calcZ_coord(-p_0.y)
        else:
            p_0Z = 0
        target0 = transl(p_0.x + OffsetToCenter[0], p_0.y -OffsetToCenter[1], p_0Z)*rotz(-pi/2)
        target0_app = target0*transl(0,0,-APPROACH)
        framedraw = RL.Item('Frame Draw')
        framedrawAbs = RL.Item('Frame Draw Abs')
        #AddTarget0 = RL.AddTarget('Target0', framedrawAbs)
        #AddTarget0.setPose(target0)
        #AddTarget_app = RL.AddTarget('App', framedrawAbs)
        #AddTarget_app.setPose(target0_app)
        robot.MoveL(target0_app)
        robot.MoveL(target0)
        
        for i in range(np):
            p_i = path.getPoint(i)
            p_i.x = -p_i.x - OffsetToCenter[0]
            p_i.y = -p_i.y + OffsetToCenter[1]
            v_i = path.getVector(i)
            
            if isCurved:
                p_iZ = -4+calcZ_coord(-p_i.y- 28.5)
                pt_pose = point3D_2_pose(p_i, v_i)
            else:
                p_iZ = 0
                pt_pose = point2D_2_pose(p_i, v_i)
                
            target = transl(-p_i.x, -p_i.y, p_iZ)*rotz(-pi/2)
            #AddTarget = RL.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
        pathEnd = target*transl(0,0,-APPROACH)
    return 0

def makeSVG(TEXT_FILE):
# creating a SVG surface
    with cairo.SVGSurface("StrEng.svg", 100, 100) as surface:
        Context = cairo.Context(surface) # creating a cairo context object for SVG surface # useing Context method	
        Context.set_source_rgb(1, 0, 0) # setting color of the context
        Context.set_font_size(50)# approximate text height
        Context.select_font_face("Arial", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL) # Font Style
        Context.move_to(35, 45) # position for the text
        Context.text_path(TEXT_FILE) # displays the text
        Context.set_line_width(2)# Width of outline
        Context.stroke()# stroke out the color and width property
        return "StrEng.svg"

def point3D_2_pose(point, tangent):
    """Converts a 2D point to a 3D pose in the XY plane including rotation being tangent to the path"""
    CircleOffset = 28.5             #The curved cover is made with a radius of 92.5 mm, with a maximum distance of 4.5mm from the flat plane of the covers corners   
                                    #This circle offset is the horisontal distance between the cirlce crossing the horisontal axis and the origin, when the circle is translated downwards so that the highest point is 4.5 above the origin
                                    #The Points that are passed to the zCoord-function are now all positive and fit with the function used in calcZ_coord.
    return transl(point.x, point.y, 1.1+calcZ_coordTri(point.y))*rotz(-tangent.angle()) #-calcZ_coord(point.y - CircleOffset)

def point2D_2_pose(point, tangent):
    """Converts a 2D point to a 3D pose in the XY plane including rotation being tangent to the path"""
    return transl(point.x, point.y, 0)*rotz(-tangent.angle())

def calcZ_coord(yCoord):                            #Because we have turned our framedraw 90 degrees compared to a conventional Cartesian coord system, the function curves over y instead of x.
     zCoord = yCoord*yCoord + 57.6*yCoord + 17.19   #These calculations were found using the cirlce's equation, thIs is only an approximation, but it is very close. It is only an approximation.
                                                    #it is an approximation because the denominator of a fraction becomes a sum containing the zCoord, so either a recursion seems neccesary of some significant amount of math to split them apart. 
     zCoord = zCoord/176 + 6.4                       
     return zCoord

def calcZ_coordTri(yCoord):                  #A different way to calculate the z-coordinate using the pythogorean theroem         
    zCoord = sqrt(92.5**2 - yCoord**2)-94.5                      
    return zCoord
#--------------------------------------------------------------------------------

#########################################################################################################################

#libs
from robolink import *
from robodk import *

import PySimpleGUI as sg

#import svgpy.engraving as eng

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
path_stationfile = RL.getParam('PATH_OPENSTATION')
RL.setSimulationSpeed(8)
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

palletFrame = RL.Item('Pallet Frame')
robot.setSpeed(200)
robot.MoveJ(home)
progReset.RunProgram()
progReset.WaitFinished()

#Gripper close distance
def setGripper(closeDistance):
    gripper.setJoints([float(closeDistance)])

def attachCovers(coverType, frame, color):
    global coverToEngrave
    global isCurved
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
        approach = bottomCoverTarget.Pose()*transl(200*color, 0, -100)
        grip = bottomCoverTarget.Pose()*transl(200*color, 0, 0)
    elif coverType == 'Curved':
        color = colorToInt[values['STCC']]
        print(color)
        print(values['STCC'])
        approach = topCurvedCoverTarget.Pose()*transl(200*color, 0, -100)
        grip = topCurvedCoverTarget.Pose()*transl(200*color, 0, 0)
    elif coverType == 'Flat':
        color = colorToInt[values['STCC']]
        approach = topFlatCoverTarget.Pose()*transl(200*color, 0, -100)
        grip = topFlatCoverTarget.Pose()*transl(200*color, 0, 0)
        
    robot.MoveL(approach)
    robot.MoveL(grip)
    #time.sleep(1)
    setGripper(60)

    #Attaches appropriate cover to gripper
    attachCovers(coverType, 'gripper', color)
    #time.sleep(1)
    robot.MoveL(approach)
    return color
    

def coverToCarrier(coverType, color):
    
    if coverType == 'bottom':
        robot.setPoseFrame(carrierFrame)
        approachCarrierTarget = bottomCoverCarrierTarget.Pose()*transl(0, 0, -100)
        grip = bottomCoverCarrierTarget.Pose()*transl(0, 0, -0)
    else:
        approachCarrierTarget = topCoverCarrierTarget.Pose()*transl(0, 0, -100)
        grip = topCoverCarrierTarget.Pose()*transl(0, 0, -0)
    
    robot.MoveL(approachCarrierTarget)
    robot.MoveL(grip)
    #time.sleep(1)
    setGripper(85)

    # attaches apropriate cover to gripper
    attachCovers(coverType, 'carrierFrame', color)
    
    robot.MoveL(approachCarrierTarget)

    return color

def coverToPallet(coverType, color):

    setGripper(60)
    robot.setPoseFrame(carrierFrame)
    
    # attaches apropriate cover to gripper
    attachCovers(coverType, 'gripper', color)
    robot.MoveJ(transl(0,0,150))
                
    robot.setPoseFrame(palletFrame)
    robot.MoveJ(transl(63.565,88.658,187.103)*rotx(degToRad(-180))*rotz(degToRad(-89.218)))

#Function for placing PCB
def getPCB(color):
    robot.setPoseFrame(PCBFrame)

    approach = PCBTarget.Pose()*transl(0, 0, -100)
    grip = PCBTarget.Pose()*transl(0, 0, 0)

    robot.MoveL(approach)
    robot.MoveL(grip)
    #time.sleep(1)
    setGripper(55)

    #Attaches appropriate cover to gripper
    PCBPart.setParentStatic(gripper)
    #time.sleep(1)
    robot.MoveL(approach)

    robot.setPoseFrame(carrierFrame)
    approachCarrierTarget = topCoverCarrierTarget.Pose()*transl(0, 0, -100)
    grip = topCoverCarrierTarget.Pose()*transl(0, 0, 0)

    robot.MoveL(approachCarrierTarget)
    robot.MoveL(grip)
    #time.sleep(1)
    setGripper(85)

    if color == 0:
        PCBPart.setParentStatic(blackBottomCover)
    elif color == 1:
        PCBPart.setParentStatic(whiteBottomCover)
    elif color == 2:
        PCBPart.setParentStatic(blueBottomCover)

    PCBPart.setPose(transl(2.5, -4.86, 3.5))

    # attaches apropriate cover to gripper
    robot.MoveL(approachCarrierTarget)
    secondFuse = False
    return 0

def degToRad (eulDeg);
    eulDeg = eulDeg*2pi
    eulDeg = eulDeg/180
    return eulDeg

def getFuse(fuse1, fuse2 , secondFuse):
    if fuse1 == True or fuse2 == True:
        setGripper(85)
        robot.setPoseFrame(fuseFrame)
    
        approach = fuseTarget.Pose()*transl(0, 0, -60)
        grip = fuseTarget.Pose()*transl(0, 0, -30) #-161

        robot.MoveL(approach)
        robot.MoveL(grip)
        #time.sleep(1)
        setGripper(4.5)
        if secondFuse == False:
            fusePart1.setParentStatic(gripper)
        else:
            fusePart2.setParentStatic(gripper)
        
        #time.sleep(1)
        robot.MoveL(approach)
        if secondFuse == False:
            fusePart2.setPose(fusePart2.Pose()*transl(0,0,-5))
            

        robot.setPoseFrame(carrierFrame)
        if fuse1 == True:
            approach = fusePlace1.Pose()*transl(0, 0, -60)
            grip = fusePlace1.Pose()*transl(0, 0, -30) 
            fuse1 = False
        else:
            approach = fusePlace2.Pose()*transl(0, 0, -60)
            grip = fusePlace2.Pose()*transl(0, 0, -30) 
            fuse2 = False

        robot.MoveL(approach)
        robot.MoveL(grip)
        if secondFuse == False:
            fusePart1.setParentStatic(PCBPart)
        else:
            fusePart2.setParentStatic(PCBPart)
           
        #time.sleep(1)
        setGripper(85)
        #time.sleep(1)
        robot.MoveL(approach)
        secondFuse = True
        return fuse1, fuse2, secondFuse
    else:
        return False, False, False

#flip top cover
def flipaDaTable(coverType, color):
    colorToInt = {
        "Black": 0,
        "White": 1,
        "Blue": 2
    }
    robot.setPoseFrame(coverTurnFrame)

    approach = dropCoverTarget.Pose()*transl(0,100, 0)
    drop = dropCoverTarget.Pose()*transl(0,0,0)
    robot.MoveJ(approach)
    robot.MoveL(drop)

    setGripper(85)
    attachCovers(coverType, 'coverTurnFrame', colorToInt[color])
    #time.sleep(1)
    robot.MoveL(RL.Item('approach grab'))

    approach = grabCoverTarget.Pose()*transl(0,0, -100)
    grab = grabCoverTarget.Pose()*transl(0,0,0)
    robot.MoveJ(approach)
    robot.MoveL(grab)

    setGripper(60)
    attachCovers(coverType, 'gripper', colorToInt[color])
    #time.sleep(1)

    robot.setPoseFrame(carrierFrame)
    approach = topCoverCarrierTarget.Pose()*transl(0,0, -100)
    drop = topCoverCarrierTarget.Pose()*transl(0,-2.5,12.5)
    robot.MoveJ(approach)
    robot.MoveL(drop)
    setGripper(85)
    attachCovers(coverType, 'PCBPart', colorToInt[color])
    #robot.MoveJ(approach)


color = getCovers('bottom')
coverToCarrier('bottom', color)
getPCB(color)


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

getFuse(facs[0], facs[1], facs[2])

robot.MoveJ(home)

getCovers(values['STCT'])

#engrave
print(values['textbox'])
if values['textbox'] != "" and relevantpath:
    print('both')
    setup(coverToEngrave, isCurved)
    StrEngrave(values['textbox'], robot, isCurved)
elif values['textbox'] != "" and not relevantpath:
    setup(coverToEngrave, isCurved)
    StrEngrave(values['textbox'], robot, isCurved)
    print('only text')
elif values['textbox'] == "" and relevantpath:
    print('only pic')
elif values['textbox'] == "" and not relevantpath:
    print('no engraving')

flipaDaTable(values['STCT'], values['STCC'])


