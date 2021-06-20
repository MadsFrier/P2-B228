from robolink import *
from robodk import *
from .svg import *
import cairo
import sys 
import os
import re

SIZE_BOARD = [107, 45]      # Size of the image. The image will be scaled keeping its aspect ratio
BEZEL_SIZE =  6             #This is used to remove the outer bezel of the cover that will not be engraved
OffsetToCenter = [(SIZE_BOARD[0]+BEZEL_SIZE)/4, (SIZE_BOARD[1]+BEZEL_SIZE)/2]     #We will need the center of the image, we can find it using the size of the image. As we will scale the image to the cover
MM_X_PIXEL = 0.15             # in mm. The path will be cut depending on the pixel size. If this value is changed it is recommended to scale the pixel object
RL = Robolink()
pixel_ref = RL.Item('Cylinder')
#--------------------------------------------------------------------------------
# function definitions:

def setup(coverToEngrave, isCurved):
    # delete any frames made in a previous run if any 

    board_draw = coverToEngrave
    
    # get the robot, frame and tool objects
    robot = RL.Item('UR5')

    EngravingFrame = RL.Item('engraving') 
    robot.setPoseFrame(EngravingFrame)
    if isCurved:
        EngravingStart = RL.Item('engravingCurved')
    else:
        EngravingStart = RL.Item('engravingFlat')
    robot.MoveL(EngravingStart)

    #Make the frame for the drawing
    framedraw = RL.AddFrame('Frame Draw', board_draw)
    framedraw.setVisible(True, True)
    if isCurved:
        framedraw.setPose(transl(30,15,54.500)*rotz(90*pi/180)*roty(90*pi/180))
    else:
        framedraw.setPose(transl(30,11.8,54.500)*rotz(90*pi/180)*roty(90*pi/180))

    framedrawAbs = RL.AddFrame('Frame Draw Abs', framedraw)
    if isCurved:
        framedrawAbs.setPose(transl(0,0,-23.5))
    else:
        framedrawAbs.setPose(transl(0,0,-22.3))
    framedrawAbs.setParentStatic(RL.Item('UR5 Base'))
    robot.setPoseFrame(framedrawAbs)

    # get the pixel reference to draw
    pixel_ref.Recolor([0,0,0])
    pixel_ref.Copy()
    
    return 0
    
def ImgEngrave(IMAGE_FILE, robot, isCurved):
    # select the file to draw
    #svgfile = path_stationfile + IMAGE_FILE
    # import the SVG file
    svgdata = svg_load(IMAGE_FILE)

    IMAGE_SIZE = Point(SIZE_BOARD[0]/2,SIZE_BOARD[1])   # size of the image in MM
    svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)
    size_img = svgdata.size_poly()  # returns the size of the current polygon

    #Our Path takes and svg image and- converts them to a set of path segments, which is made up of points.
    #Between each path a approach point is made.
    APPROACH = 10  # approach distance in MM for each path

    for path in svgdata:
    
        np = path.nPoints()
            
        # robot movement: approach to the first target
        p_0 = path.getPoint(0)
        if isCurved:
            p_0Z = calcZ_coordTri(p_0.y)
        else:
            p_0Z = 0
        target0 = transl(-p_0.x + OffsetToCenter[0], -p_0.y + OffsetToCenter[1], -p_0Z)*rotz(-pi/2)
        target0_app = target0*transl(0,0, APPROACH)
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
            p_i.x = p_i.x - OffsetToCenter[0]
            p_i.y = p_i.y - OffsetToCenter[1]
            v_i = path.getVector(i)
            
            if isCurved:
                p_iZ = calcZ_coordTri(p_i.y)
                pt_pose = point3D_2_pose(p_i, v_i)
            else:
                p_iZ = 0
                pt_pose = point2D_2_pose(p_i, v_i)
                
            target = transl(-p_i.x , -p_i.y , -p_iZ)*rotz(-pi/2)
            #AddTarget = RL.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
        pathEnd = target*transl(0,0,-APPROACH)
        robot.MoveJ(pathEnd)
    return 0

def StrEngrave(TEXT_FILE, robot, isCurved):
    # select the file to draw
    path_stationfile = RL.getParam('PATH_OPENSTATION')
    svgfile = path_stationfile + '/' + makeSVG(TEXT_FILE)
    # import the SVG file
    svgdata = svg_load(svgfile) 

    IMAGE_SIZE = Point(SIZE_BOARD[0]/16,SIZE_BOARD[1]/4)   # size of the image in MM
    svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)
    size_img = svgdata.size_poly()  # returns the size of the current polygon

    #Our Path takes and svg image and- converts them to a set of path segments, which is made up of points.
    #Between each path a approach point is made.
    APPROACH = 10  # approach distance in MM for each path

    for path in svgdata:
        
        np = path.nPoints()
        
        segment = path.getPath()._segments

        # robot movement: approach to the first target
        p_0 = path.getPoint(0)

        if isCurved:
            p_0Z = calcZ_coordTri(p_0.y)
        else:
            p_0Z = 0
        target0 = transl(p_0.x + OffsetToCenter[0], p_0.y, p_0Z)*rotz(-pi/2)
        target0_app = target0*transl(0,0,-APPROACH)
        framedraw = RL.Item('Frame Draw')
        framedrawAbs = RL.Item('Frame Draw Abs')
        #AddTarget0 = RL.AddTarget('Target0', framedrawAbs)
        #AddTarget0.setPose(target0)
        #AddTarget_app = RL.AddTarget('App', framedrawAbs)
        #AddTarget_app.setPose(target0_app)
        #robot.MoveL(target0_app)
        #robot.MoveL(target0)
        
        for i in range(np):
            p_i = path.getPoint(i)
            p_i.x = -p_i.x - OffsetToCenter[0]
            p_i.y = -p_i.y 
            v_i = path.getVector(i)
            
            #Check if the point is a turning point by checking angles before and after
            if abs(path.getVector(i).angle() - path.getVector(i-1).angle()) > pi/146 and abs(path.getVector(i).angle() - path.getVector(i+1).angle()) > pi/146:
               pixel_ref.Recolor([1,0,0]) #([0,0,0,1])
               pixel_ref.Copy()
               if abs(path.getVector(i-1).angle() - path.getVector(i-2).angle()) > pi/146 or abs(path.getVector(i+1).angle() - path.getVector(i+2).angle()) > pi/146:
                  pixel_ref.Recolor([0,0,1])
                  pixel_ref.Copy()
            else: 
                pixel_ref.Recolor([0,0,0]) #([0,0,0,1])
                pixel_ref.Copy()
             

            if isCurved:
                p_iZ = calcZ_coordTri(p_i.y)
                pt_pose = point3D_2_pose(p_i, v_i)
                
            else:
                p_iZ = 0
                pt_pose = point2D_2_pose(p_i, v_i)
                
            target = transl(-p_i.x, -p_i.y, -p_iZ)*rotz(-pi/2)
            #AddTarget = RL.AddTarget('Engraving Target', framedrawAbs)
            #AddTarget.setPose(target)
            # Move the robot to the next target
            #robot.MoveL(target)
            framedraw.Paste().setPose(pt_pose)
            #pixel.setName('%s'%path.getVector(i).angle())
        pathEnd = target*transl(0,0,-APPROACH)
        robot.MoveJ(pathEnd)
    return 0

def makeSVG(TEXT_FILE):
# creating a SVG surface
    with cairo.SVGSurface("StrEng.svg", 100, 100) as surface:
        Context = cairo.Context(surface) # creating a cairo context object for SVG surface # useing Context method	
        Context.set_source_rgb(1, 0, 0) # setting color of the context
        Context.set_font_size(50)# approximate text height
        Context.select_font_face("Purisa", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL) # Font Style
        Context.move_to(35, 45) # position for the text
        Context.text_path(TEXT_FILE) # displays the text
        Context.set_line_width(2)# Width of outline
        Context.stroke()# stroke out the color and width property
        return "StrEng.svg"

def point3D_2_pose(point, tangent):
    """Converts a 2D point to a 3D pose in the XY plane including rotation being tangent to the path"""
       #The curved cover is made with a radius of 92.5 mm, with a maximum distance of 4.5mm from the flat plane of the covers top   
       #This circle offset is the horisontal distance between the cirlce crossing the horisontal axis and the origin, when the circle is translated downwards so that the highest point is 4.5 above the origin
 
    return transl(point.x, point.y, calcZ_coordTri(point.y))*rotz(-tangent.angle())

def point2D_2_pose(point, tangent):
    """Converts a 2D point to a 3D pose in the XY plane including rotation being tangent to the path"""
    return transl(point.x, point.y, 0)*rotz(-tangent.angle())

def calcZ_coordTri(yCoord):                  #calculating the z-coordinate using the pythogorean theroem         
    zCoord = sqrt(92.5**2 - yCoord**2)-92.5                      
    return zCoord
