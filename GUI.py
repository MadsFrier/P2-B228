from robolink import *
from robodk import *

import PySimpleGUI as sg
import os.path

sg.theme('DarkBlack')

bottomCoverColor = 0
fuseAmount = 0
topCoverType = 0
topCoverColor = 0
engraving = "No text"

selectColor = (
    'Black', 'White', 'Blue')
   
selectFuse = (
    '0', '1', '2')

selectTopCoverType = (
    'Flat', 'Curved')

width = max(map(len, selectColor))+1

layout = [
    [sg.Text("Select bottom cover color")], [sg.Combo(selectColor, size=(width, 5), enable_events=True, key='SBCC')],
    [sg.Text("Select amount of fuses")], [sg.Combo(selectFuse, size=(width, 5), enable_events=True, key='SAOF')],
    [sg.Text("Select top cover type")], [sg.Combo(selectTopCoverType, size=(width, 5), enable_events=True, key='STCT')],
    [sg.Text("Select top cover color")], [sg.Combo(selectColor, size=(width, 5), enable_events=True, key='STCC')],
    [sg.Text("Choose engraving")], [sg.Input(do_not_clear=True, enable_events=True, key='textbox')],
    [sg.Button("Place order", key='PO')],
]

window = sg.Window("Dummy Phone Order", layout, finalize=True)

while True:
    event, values = window.read()
    print(event, values)

    #Textbox
    if len(values["textbox"])>10:
        window.Element('textbox').Update(values['textbox'][:-1])

    #Bottom cover color
    if values['SBCC'] == "Black":
        bottomCoverColor = 1

    if values['SBCC'] == "White":
        bottomCoverColor = 2

    if values['SBCC'] == "Blue":
        bottomCoverColor = 3

    #Fuse amount
    if values['SAOF'] == "0":
        fuseAmount = 1

    if values['SAOF'] == "1":
        fuseAmount = 2

    if values['SAOF'] == "2":
        fuseAmount = 3

    #Top cover type
    if values['STCT'] == "Flat":
        topCoverType = 1

    if values['STCT'] == "Curved":
        topCoverType = 2

    #Top cover color
    if values['STCC'] == "Black":
        topCoverColor = 1

    if values['STCC'] == "White":
        topCoverColor = 2

    if values['STCC'] == "Blue":
        topCoverColor = 3
    
    if event == sg.WINDOW_CLOSED:
        break
    
    if event == 'PO':
        if bottomCoverColor == 0 or fuseAmount == 0 or topCoverType == 0 or topCoverColor == 0:
            sg.popup_error(f'Fill everything out!')
        else:
            #Place order
            break

window.close()

print(bottomCoverColor, fuseAmount, topCoverType, topCoverColor)

RL = Robolink()
 
robot = RL.Item('UR5e')

home = RL.Item('Target 1')

if bottomCoverColor == 1:
    assemble1 = RL.Item('Prog1')
    assemble1.RunProgram()

if bottomCoverColor == 2:
    assemble2 = RL.Item('Prog3')
    assemble2.RunProgram()