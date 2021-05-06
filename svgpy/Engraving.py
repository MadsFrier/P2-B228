from robolink import *    # API to communicate with RoboDK
from robodk import *      # robodk robotics toolbox
import cairo
import sys 
import os
import re

PIXELS_AS_OBJECTS = True    # Set to True to generate PDF or HTML simulations that include the drawn path
TCP_KEEP_TANGENCY = False   # Set to True to keep the tangency along the path
SIZE_BOARD = [107, 52]      # Size of the image. The image will be scaled keeping its aspect ratio
BEZEL_SIZE = 2              #This is used to remove the outer bezel of the cover that will not be engraved
OffsetToCenter = [(SIZE_BOARD[0]+BEZEL_SIZE)/4, (SIZE_BOARD[1]+BEZEL_SIZE)/2]
MM_X_PIXEL = 0.1             # in mm. The path will be cut depending on the pixel size. If this value is changed it is recommended to scale the pixel object
IMAGE_FILE = '/World map.svg'# Path of the SVG image, it can be relative to the current RDK station
TEXT_FILE = 't'

#--------------------------------------------------------------------------------
# function definitions:

def EngravingSetup(IMAGE_FILE,TEXT_FILE, isCurved):
    # delete any frames made in a previous run if any
    image = RDK.Item('Frame Draw')
    if image.Valid() and image.Type() == ITEM_TYPE_FRAME: image.Delete()
    targetFrame = RDK.Item('Frame Draw Abs')
    if targetFrame.Valid() and targetFrame.Type() == ITEM_TYPE_FRAME: targetFrame.Delete()
    EngravingFrame = RDK.Item('Engraving Frame')
    if EngravingFrame.Valid() and EngravingFrame.Type() == ITEM_TYPE_FRAME: EngravingFrame.Delete()

    TopCover = 'Curved Cover' 
    board_draw = RDK.Item(TopCover)
    
    # get the robot, frame and tool objects
    robot = RDK.ItemUserPick('', ITEM_TYPE_ROBOT)
    EngravingTool = RDK.Item('Engraving Tool')
    EngravingFrame = RDK.Item('Frame 5') #RDK.AddFrame('Engraving Frame', EngravingTool)
    #EngravingFrame.setPose(transl(171.867,50.001,249.408))
    #EngravingFrame.setPose(EngravingFrame.Pose()*rotx(3.873)*roty(53.297)*rotz(-3.095))
    #EngravingStart = RDK.AddTarget('Engraving Start',EngravingFrame) #[-145.119000, -56.335383, 102.992642, -110.819712, 54.032459, 47.440227] 
    #robot.setPoseFrame(EngravingFrame)
    if isCurved:
        #EngravingStart.setPose(transl(0,0,-13.110))
        EngravingStart = RDK.Item('Engraving Start Curved')
    else:
        #EngravingStart.setPose(transl(0,0,-10))
        EngravingStart = RDK.Item('Engraving Start Flat')
    robot.MoveJ(EngravingStart)

    framedraw = RDK.AddFrame('Frame Draw', board_draw)
    framedraw.setVisible(False, True)
    framedraw.setPose(transl(0,0,2))
    framedrawAbs = RDK.AddFrame('Frame Draw Abs', framedraw)
    framedrawAbs.setPose(transl(0,0,0))
    framedrawAbs.setParentStatic(RDK.Item('UR5 Base'))
    robot.setPoseFrame(framedrawAbs)
    tooldraw = RDK.Item('Gripper')

    # get the pixel reference to draw
    pixel_ref = RDK.Item('Cylinder')
    pixel_ref.Copy()
    #ImgEngrave(IMAGE_FILE, robot, framedraw, isCurved)
    StrEngrave(TEXT_FILE, robot, framedraw, isCurved)
    robot.MoveJ(EngravingStart)
    return 0
    
def ImgEngrave(IMAGE_FILE, robot, framedraw, isCurved):
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
        framedrawAbs = RDK.Item('Frame Draw Abs')
##        AddTarget = RDK.AddTarget('Target', framedrawAbs)
##        AddTarget.setPose(target0)
##        AddTarget_app = RDK.AddTarget('App', framedrawAbs)
##        AddTarget_app.setPose(target0_app)
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
            #AddTarget = RDK.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
        pathEnd = target*transl(0,0,-APPROACH)
    return 0

def StrEngrave(TEXT_FILE, robot, framedraw, isCurved):
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
        target0 = transl(p_0.x + OffsetToCenter[0], p_0.y -OffsetToCenter[1], p_0Z)
        target0_app = target0*transl(0,0,-APPROACH)
##        framedrawAbs = RDK.Item('Frame Draw Abs')
##        AddTarget0 = RDK.AddTarget('Target0', framedrawAbs)
##        AddTarget0.setPose(target0)
##        AddTarget_app = RDK.AddTarget('App', framedrawAbs)
##        AddTarget_app.setPose(target0_app)
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
                
            target = transl(-p_i.x, -p_i.y, p_iZ)
            #AddTarget = RDK.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
        pathEnd = target*transl(0,0,-APPROACH)
    return 0

def makeSVG(TEXT_FILE):
# creating a SVG surface
# here geek95 is file name & 700, 700 is dimension
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
    return transl(point.x, point.y, calcZ_coordTri(point.y))*rotz(-tangent.angle()) #-calcZ_coord(point.y - CircleOffset)

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
# Program start
RDK = Robolink()
path_stationfile = RDK.getParam('PATH_OPENSTATION')
from svgpy.svg import *
RDK.setSimulationSpeed(1)
EngravingSetup(IMAGE_FILE, TEXT_FILE, True)

