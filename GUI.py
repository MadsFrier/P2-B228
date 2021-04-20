from robolink import *
from robodk import *

import PySimpleGUI as sg
import os.path

sg.theme('DarkBlack')

selectColor = (
    'Black', 'White', 'Blue')

selectFuse = (
    '0', '1', '1*', '2')

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

class phone:

    def __init__(self, bottomCoverColor, fuseAmount, topCoverType, topCoverColor, engraving):

        self.bottomCoverColor = bottomCoverColor
        self.fuseAmount = fuseAmount
        self.topCoverType = topCoverType
        self.topCoverColor = topCoverColor
        self.engraving = engraving

while True:
    event, values = window.read()
    print(event, values)

    if len(values["textbox"])>10:
        window.Element('textbox').Update(values['textbox'][:-1])

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'PO':
        if values['SBCC'] is "" or values['SAOF'] is "" or values['STCT'] is "" or values['STCC'] is "":
            sg.popup_error(f'Fill everything out!')
        else:
            #Place order
            break

window.close()

RL = Robolink()

robot = RL.Item('UR5e')

home = RL.Item('Target 1')

phoneInfo = phone(values['SBCC'], values['SAOF'], values['STCT'], values['STCC'], values['textbox'])

#List of RDK Programs
x = RL.Item('RDKProg')

#Bottom Cover Color
if phoneInfo.bottomCoverColor == 'Black':
    assemble1 = RL.Item('Prog1')
    assemble1.RunProgram()
elif phoneInfo.bottomCoverColor == 'White':
    assemble2 = RL.Item('Prog2')
    assemble2.RunProgram()
elif phoneInfo.bottomCoverColor == 'Blue':
    assemble3 = RL.Item('Prog3')
    assemble3.RunProgram()

#fuse Amount
if phoneInfo.fuseAmount == '0':
    assemble1 = RL.Item('Prog1')
    assemble1.RunProgram()
elif phoneInfo.fuseAmount == '1':
    assemble2 = RL.Item('Prog2')
    assemble2.RunProgram()
elif phoneInfo.fuseAmount == '1*':
    assemble3 = RL.Item('Prog3')
    assemble3.RunProgram()
elif phoneInfo.fuseAmount == '2':
    assemble3 = RL.Item('Prog3')
    assemble3.RunProgram()