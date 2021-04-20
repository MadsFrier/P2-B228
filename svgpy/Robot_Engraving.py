from robolink import *    # API to communicate with RoboDK
from robodk import *      # robodk robotics toolbox

import sys 
import os
import re

PIXELS_AS_OBJECTS = False    # Set to True to generate PDF or HTML simulations that include the drawn path
TCP_KEEP_TANGENCY = False    # Set to True to keep the tangency along the path
SIZE_BOARD = [50, 100]     # Size of the image. The image will be scaled keeping its aspect ratio
MM_X_PIXEL = 1             # in mm. The path will be cut depending on the pixel size. If this value is changed it is recommended to scale the pixel object
IMAGE_FILE = 'World map.svg'             # Path of the SVG image, it can be relative to the current RDK station

#--------------------------------------------------------------------------------
# function definitions:

def point2D_2_pose(point, tangent):
    """Converts a 2D point to a 3D pose in the XY plane including rotation being tangent to the path"""
    return transl(point.x, point.y, 0)*rotz(tangent.angle())

def svg_draw_quick(svg_img, board, pix_ref):
    """Quickly shows the image result without checking the robot movements."""
    RDK.Render(False)
    count = 0
    for path in svg_img:
        count = count + 1
        # use the pixel reference to set the path color, set pixel width and copy as a reference
        pix_ref.Recolor(path.fill_color)
        if PIXELS_AS_OBJECTS:
            pix_ref.Copy()
        np = path.nPoints()
        print('drawing path %i/%i' % (count, len(svg_img)))
        for i in range(np):
            p_i = path.getPoint(i)
            v_i = path.getVector(i)

            # Reorient the pixel object along the path
            pt_pose = point2D_2_pose(p_i, v_i)
            
            # add the pixel geometry to the drawing board object, at the calculated pixel pose
            if PIXELS_AS_OBJECTS:
                board.Paste().setPose(pt_pose)
            else:
                board.AddGeometry(pix_ref, pt_pose)
            
    RDK.Render(True)

def svg_draw_robot(svg_img, board, pix_ref, item_frame, item_tool, robot):
    """Draws the image with the robot. It is slower that svg_draw_quick but it makes sure that the image can be drawn with the robot."""

    APPROACH = 2  # approach distance in MM for each path    
    home_joints = [-144.700431, -60.204293, 103.198570, -99.897231, 53.177811, 38.494197]
    
    robot.setPoseFrame(item_frame)
    robot.setPoseTool(item_tool)

    # get the target orientation depending on the tool orientation at home position
    #orient_frame2tool = invH(item_frame.Pose())*robot.SolveFK(home_joints)*item_tool.Pose()
    orient_frame2tool = roty(pi) #alternative:
    orient_frame2tool[0:3,3] = Mat([0,0,0])
    
    for path in svg_img:
        # use the pixel reference to set the path color, set pixel width and copy as a reference
        if PIXELS_AS_OBJECTS:
            pix_ref.Copy()
        np = path.nPoints()

        # robot movement: approach to the first target
        p_0 = path.getPoint(0)
        target0 = transl(p_0.x, p_0.y, 0)*orient_frame2tool
        target0_app = target0*transl(0,0,-APPROACH)
        robot.MoveL(target0_app)

        if TCP_KEEP_TANGENCY:
            joints_now = robot.Joints().tolist()
            joints_now[5] = -180
            robot.MoveJ(joints_now)
 
        for i in range(np):
            p_i = path.getPoint(i)
            v_i = path.getVector(i)       
            pt_pose = point2D_2_pose(p_i, v_i)
   
            if TCP_KEEP_TANGENCY:
                #moving the tool along the path (axis 6 may reach its limits)                
                target = pt_pose*orient_frame2tool 
            else:
                #keep the tool orientation constant
                target = transl(p_i.x, p_i.y, 0)*orient_frame2tool

            # Move the robot to the next target
            robot.MoveJ(target)

            # create a new pixel object with the calculated pixel pose
            if PIXELS_AS_OBJECTS:
                board.Paste().setPose(pt_pose)
            else:
                board.AddGeometry(pix_ref, pt_pose)

        target_app = target*transl(0,0,-APPROACH)
        robot.MoveL(target_app)
        
    robot.MoveL(home_joints)

#--------------------------------------------------------------------------------
# Program start
RDK = Robolink()
path_stationfile = RDK.getParam('PATH_OPENSTATION')

from svgpy.svg import *

# select the file to draw
svgfile = path_stationfile + '/Camillas tegning.svg'

# import the SVG file
svgdata = svg_load(svgfile)

IMAGE_SIZE = Point(SIZE_BOARD[0],SIZE_BOARD[1])   # size of the image in MM
svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)
size_img = svgdata.size_poly()  # returns the size of the current polygon

# get the robot, frame and tool objects
robot = RDK.ItemUserPick('', ITEM_TYPE_ROBOT)
framedraw = RDK.Item('Frame draw')
tooldraw = RDK.Item('Gripper')

# get the pixel reference to draw
pixel_ref = RDK.Item('Ellipse')

# delete previous image if any
image = RDK.Item('Board & image')
if image.Valid() and image.Type() == ITEM_TYPE_OBJECT: image.Delete()

# make a drawing board base on the object reference "Blackboard 250mm"
board_draw = RDK.Item('Top')
pixel_ref.Copy()

# quickly show the final result without checking the robot movements:
#svg_draw_quick(svgdata, board_draw, pixel_ref)

# draw the image with the robot:
svg_draw_robot(svgdata, board_draw, pixel_ref, framedraw, tooldraw, robot)
