#!/usr/bin/env python3


#***************************************************************************
#*  Script to generate map after measuring lengths physically
#*
#*  Created:  May 30 16:30:00 2019
#*  Copyright  2019  Morian Sonnet [Morian.Sonnet@rwth-aachen.de]
#*
#****************************************************************************

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.
#

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QMessageBox
from PyQt5.QtGui import QPixmap, QImage, QDoubleValidator
import PyQt5.uic as uic

import PIL.ImageQt as ImageQt
from qlineeditfocus import  QLineEditFocus



import map_tool as mt
import map_tool_validator as mtv

import yaml


class MapTool:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = uic.loadUi("./window.ui")
        self.highlight = ''
        try:
            with open('values.yaml','r') as values_file:
                self.values = yaml.safe_load(values_file)
        except yaml.YAMLError as exc:
            print(exc)
            print("Delete or repair values.yaml")
            sys.exit(0)
        except IOError:
            #file does not exist, use default values
            self.values = { 'width' : 14,
                'hole_left_top' : 2.2,
                'hole_top' : 0.8,
                'height_left' : 7,
                'height_right' : 7,
                'hole_top_left' : 1.5,
                'hole_bottom_left' : 1,
                'hole_top_right' : 1.5,
                'hole_bottom_right' : 1,
                'middle_height' : 8,
                'middle_extent' : 4,
                'insert_top_left' : 2,
                'insert_top_right' : 2,
                'insert_left_height' : 1,
                'insert_right_height': 1,
                'insert_left_height_wall' : 1,
                'insert_right_height_wall': 1,
                'insert_left_width_wall' : 3,
                'insert_right_width_wall': 3,
                'insert_left_hole' : 1,
                'insert_right_hole' :1,
                'middle_hole_left' : 2}

        for name in self.values.keys():
            edit = self.window.findChild(QLineEditFocus,name)
            edit.setText('{:.2f}'.format(self.values[name]))
            edit.focusInSignal.connect(lambda name = name : self.focus(name))
            edit.editingFinished.connect(self.create_string)
            edit.setValidator(QDoubleValidator())



        self.window.createString.clicked.connect(self.dehighlight_reread)
        self.window.saveMap.clicked.connect(self.save_map)

        #self.window.mapString.soft_validator = mtv.MapStringValidator()
        self.window.mapString.editingFinished.connect(self.create_custom_string)
    
        self.window.show()

        self.create_string()


        sys.exit(self.app.exec_())

    def create_custom_string(self):
        print("hello")
        self.map_string = self.window.mapString.text()
        self.update()

    def show_command(self,command):
        self.image, self.origin = mt.paint_command(command)
        self.pixmap = ImageQt.toqpixmap(self.image)
        self.window.map.setPixmap(self.pixmap)

    def focus(self,highlight):
        self.highlight = highlight
        self.create_string()

    def dehighlight_reread(self):
        self.highlight = ''
        self.create_string()

    def dehighlight(self):
        self.highlight = ''
        self.map_string = self.map_string.replace('*','')
        self.window.mapString.setText(self.map_string)
        self.update()

    def create_string(self):
        def stringify(f):
            return "{:.2f}".format(f)
        # read all length settings into dictionary
        for name in self.values.keys():
            edit = self.window.findChild(QLineEditFocus,name)
            self.values[name] = float(edit.text())

        print(self.highlight)

        #create THA STRING # magic
        temp_string = ""
        #start with left insertion zone
        left_start_x = - self.values['middle_extent']/2-self.values['middle_hole_left']-self.values['insert_left_width_wall']
        left_start_y = self.values['middle_height'] - self.values['height_left'] - self.values['insert_left_height']
        temp_string = temp_string + "(" + stringify(left_start_x) + "," + stringify(left_start_y) + ")"
        #draw left insertion zone
        temp_string = temp_string + stringify(self.values['insert_left_width_wall']) + ("*;" if self.highlight == 'insert_left_width_wall' else ";") + stringify(self.values['insert_left_height_wall']) + ("*" if self.highlight == 'insert_left_height_wall' else "")
        #offset insert highest point to main wall
        y_offset = self.values['insert_left_height'] - self.values['insert_left_height_wall']
        temp_string = temp_string + "[" + stringify(y_offset) + "];"
        #insert hole left
        temp_string = temp_string + "[" + stringify(-self.values['insert_left_hole']) + ("*]" if self.highlight == 'insert_left_hole' else "]")
        temp_string = temp_string + stringify(-self.values['insert_top_left']) + ('*;' if self.highlight == 'insert_top_left' else ';')

#left side
        temp_string = temp_string + stringify(self.values['hole_bottom_left']) + ( '*' if (self.highlight == 'height_left' or self.highlight == 'hole_bottom_left') else '')
        temp_string = temp_string + '[' + stringify(self.values['height_left'] - self.values['hole_bottom_left'] - self.values['hole_top_left']) + ( '*]' if (self.highlight == 'height_left') else ']')
        temp_string = temp_string + stringify(self.values['hole_top_left']) + ( '*;' if (self.highlight == 'height_left' or self.highlight == 'hole_top_left') else ';')

#top side
        temp_string = temp_string + stringify(self.values['hole_left_top']) + ( '*' if (self.highlight == 'hole_left_top' or self.highlight == 'width') else '') 
        temp_string = temp_string + '[' + stringify(self.values['hole_top']) + ( '*]' if (self.highlight == 'hole_top' or self.highlight == 'width') else ']') 
        temp_string = temp_string +  stringify(self.values['width'] - self.values['hole_top'] - self.values['hole_left_top']) + ( '*;' if  self.highlight == 'width' else ';') 

#right side
        temp_string = temp_string + stringify(-self.values['hole_top_right']) + ( '*' if (self.highlight == 'height_right' or self.highlight == 'hole_top_right') else '')
        temp_string = temp_string + '[' + stringify(-self.values['height_right'] + self.values['hole_bottom_right'] + self.values['hole_top_right']) + ( '*]' if (self.highlight == 'height_right') else ']')
        temp_string = temp_string + stringify(-self.values['hole_bottom_right']) + ( '*;' if (self.highlight == 'height_right' or self.highlight == 'hole_bottom_right') else ';')

        temp_string = temp_string + stringify(-self.values['insert_top_right']) + ( '*' if self.highlight== 'insert_top_right' else '')
        temp_string = temp_string + '[' + stringify(-self.values['insert_right_hole']) + ( '*];' if self.highlight== 'insert_right_hole' else '];')

        y_offset = self.values['insert_right_height'] - self.values['insert_right_height_wall']
        temp_string = temp_string + "[" + stringify(-y_offset) + "]"

        temp_string = temp_string  + stringify(-self.values['insert_right_height_wall']) + ( '*;' if self.highlight== 'insert_right_height_wall' else ';')
        temp_string = temp_string  + stringify(self.values['insert_right_width_wall']) + ( '*;' if self.highlight== 'insert_right_width_wall' else ';')


#middle bottom line
        temp_string = temp_string + "(" + stringify(self.values['middle_extent']/2) + "," + stringify(0) + ")" + stringify(-self.values['middle_extent']) + ( '*;' if self.highlight == 'middle_extent' else ';' )
        if self.highlight == 'middle_hole_left' :
            temp_string = temp_string + ';[' + stringify(-self.values['middle_hole_left']) + '*];'


        if self.highlight == 'middle_height':
            temp_string = temp_string + "(0,0);[" + stringify(self.values['middle_height']) + '*];'

        if self.highlight == 'insert_left_height':
            temp_string = temp_string + "(" + stringify(left_start_x + self.values['insert_left_width_wall']/2) + ',' + stringify(left_start_y) + ');[' + stringify(self.values['insert_left_height']) + '*];' 

        if self.highlight == 'insert_right_height':
            right_start_x = -self.values['middle_extent']/2-self.values['middle_hole_left']-self.values['insert_left_hole']-self.values['insert_top_left']+self.values['width'] -self.values['insert_top_right'] -self.values['insert_right_hole']+self.values['insert_right_width_wall']
            right_start_y = self.values['middle_height'] - self.values['height_right'] - self.values['insert_right_height']
            temp_string = temp_string + "(" + stringify(right_start_x - self.values['insert_right_width_wall']/2) + ',' + stringify(right_start_y) + ');[' + stringify(self.values['insert_right_height']) + '*];' 

        self.map_string = temp_string
        self.window.mapString.setText(self.map_string)

        self.update()

    def update(self):
        self.show_command(self.map_string)

    def save_map(self):
        self.dehighlight()
        self.image.save('map.png')
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        msg.setText("The origin is at {}".format(self.origin) )
        msg.setWindowTitle("Origin position")
        msg.setStandardButtons(QMessageBox.Ok)

        msg.exec_()

        with open('map_creation.txt','w') as f:
            f.write("Origin:\n")
            f.write(str(self.origin))
            f.write("\nMap String:\n")
            f.write(self.map_string)
        with open('values.yaml','w') as f:
            yaml.dump(self.values, f, default_flow_style=False)
        




        



if __name__ == '__main__':
    MapTool()
