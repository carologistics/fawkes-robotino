#!/usr/bin/env python3
# ***************************************************************************
# *  Script to generate map after measuring lengths physically
# *
# *  Created:  May 30 16:30:00 2019
# *  Copyright  2019  Morian Sonnet [Morian.Sonnet@rwth-aachen.de]
# *
# ****************************************************************************
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

try:
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtGui import QDoubleValidator
    import PyQt5.uic as uic
except ImportError:
    print("PyQt5 import error, install 'python3-qt5' with dnf")
    sys.exit(1)

try:
    import PIL.ImageQt as ImageQt
except ImportError:
    print("PIL.ImageQt import error, install 'python3-pillow-qt' with dnf")
    sys.exit(1)

from qlineeditfocus import QLineEditFocus


import map_tool as mt

try:
    import yaml
except ImportError:
    print("yaml import error, install 'python3-pyyaml' with dnf")
    sys.exit(1)


class MapTool:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = uic.loadUi("./window.ui")
        self.highlight = ""
        try:
            with open("values.yaml", "r") as values_file:
                self.values = yaml.safe_load(values_file)
        except yaml.YAMLError as exc:
            print(exc)
            print("Delete or repair values.yaml")
            sys.exit(0)
        except IOError:
            # file does not exist, use default values
            self.values = {
                "width": 14,
                "hole_left_top": 2.2,
                "hole_top": 0.8,
                "height_left": 7,
                "height_right": 7,
                "hole_top_left": 1.5,
                "hole_bottom_left": 1,
                "hole_top_right": 1.5,
                "hole_bottom_right": 1,
                "middle_height": 8,
                "middle_extent": 4,
                "insert_top_left": 2,
                "insert_top_right": 2,
                "insert_left_height": 1,
                "insert_right_height": 1,
                "insert_left_height_wall": 1,
                "insert_right_height_wall": 1,
                "insert_left_width_wall": 3,
                "insert_right_width_wall": 3,
                "insert_left_hole": 1,
                "insert_right_hole": 1,
                "middle_hole_left": 2,
            }
            # To understand the meaning of the different names, click the corresponding
            # line edit in the GUI. The corresponding part is highlighted, the name is output
            # in the shell.

        for name in self.values.keys():  # initialize all edit elements in the GUI
            edit = self.window.findChild(QLineEditFocus, name)
            edit.setText("{:.2f}".format(self.values[name]))
            edit.focusInSignal.connect(lambda name=name: self.focus(name))
            edit.editingFinished.connect(self.create_string)
            edit.setValidator(QDoubleValidator())

        self.window.createString.clicked.connect(self.dehighlight_reread)
        self.window.saveMap.clicked.connect(self.save_map)

        self.window.mapString.editingFinished.connect(self.create_custom_string)

        self.window.show()

        self.create_string()

        sys.exit(self.app.exec_())

    # this is called after the command string was manually manipulated
    def create_custom_string(self):
        self.map_string = self.window.mapString.text()
        self.update()

    # paint the command string and display it
    def show_command(self, command):
        self.image, self.origin = mt.paint_command(command)
        self.pixmap = ImageQt.toqpixmap(self.image)
        self.window.map.setPixmap(self.pixmap)

    # focus on a specific line element
    def focus(self, highlight):
        self.highlight = highlight
        self.create_string()

    # remove highlights, but reread. This removes some dead code in the command string,
    # but also deletes all manual manipulations of the command string
    def dehighlight_reread(self):
        self.highlight = ""
        self.create_string()

    # just remote all highlights by eliminating all asterisks
    def dehighlight(self):
        self.highlight = ""
        self.map_string = self.map_string.replace("*", "")
        self.window.mapString.setText(self.map_string)
        self.update()

    # function to create the command string for the map, including highlighted element
    def create_string(self):
        def stringify(f):
            return "{:.2f}".format(f)

        # read all length settings into dictionary
        for name in self.values.keys():
            edit = self.window.findChild(QLineEditFocus, name)
            self.values[name] = float(edit.text())

        print(self.highlight)

        # The following code assembles the map string, by just concatenating a lot of strings.
        # All numbers are formated using the stringify function, which allows two digits behind the comma.
        # The general idea is to have one long line streak, which starts in the left insertion zone and goes in
        # S shape up to the top wall. It continues then in an 2 shape down to the right insertion zone again. Finally,
        # the middle wall is shown. At the end, depending on the highlighted section, a few more segments are drawn,
        # to resemble the highlighted vacancies. To make clearer which part of the code belongs to which part in the
        # final map, for almost each comment a number in parentheses is given.
        # Look into the file map_example.png to find the number an see where the following line of code refers to.
        temp_string = ""

        # First, the starting coordinate of the left insertion zone is calculated.
        left_start_x = (
            -self.values["middle_extent"] / 2 - self.values["middle_hole_left"] - self.values["insert_left_width_wall"]
        )
        left_start_y = self.values["middle_height"] - self.values["height_left"] - self.values["insert_left_height"]
        # Store the coordinate into the map string
        temp_string = temp_string + "(" + stringify(left_start_x) + "," + stringify(left_start_y) + ")"

        # Add the line to the right and highlight it, if necessary. (1)
        temp_string = (
            temp_string
            + stringify(self.values["insert_left_width_wall"])
            + ("*;" if self.highlight == "insert_left_width_wall" else ";")
        )
        # (2)
        temp_string = (
            temp_string
            + stringify(self.values["insert_left_height_wall"])
            + ("*" if self.highlight == "insert_left_height_wall" else "")
        )
        # Sometime there is a small offset between the right wall of the left insertion zone and its top wall.
        y_offset = self.values["insert_left_height"] - self.values["insert_left_height_wall"]
        temp_string = temp_string + "[" + stringify(y_offset) + "];"
        # Insert the vacancy where the robot will enter the field. (3)
        temp_string = (
            temp_string
            + "["
            + stringify(-self.values["insert_left_hole"])
            + ("*]" if self.highlight == "insert_left_hole" else "]")
        )
        # Top wall of the left insertion zone. (4)
        temp_string = (
            temp_string
            + stringify(-self.values["insert_top_left"])
            + ("*;" if self.highlight == "insert_top_left" else ";")
        )

        # Now the left side of the playing field. (5)
        temp_string = (
            temp_string
            + stringify(self.values["hole_bottom_left"])
            + ("*" if (self.highlight == "height_left" or self.highlight == "hole_bottom_left") else "")
        )
        # (6)
        temp_string = (
            temp_string
            + "["
            + stringify(self.values["height_left"] - self.values["hole_bottom_left"] - self.values["hole_top_left"])
            + ("*]" if (self.highlight == "height_left") else "]")
        )
        # (7)
        temp_string = (
            temp_string
            + stringify(self.values["hole_top_left"])
            + ("*;" if (self.highlight == "height_left" or self.highlight == "hole_top_left") else ";")
        )

        # Top wall of the field. (8)
        temp_string = (
            temp_string
            + stringify(self.values["hole_left_top"])
            + ("*" if (self.highlight == "hole_left_top" or self.highlight == "width") else "")
        )
        # Often the top wall cannot be closed, due to limited supply of walls during the RoboCup.
        # This vacancy is inserted now. (9)
        temp_string = (
            temp_string
            + "["
            + stringify(self.values["hole_top"])
            + ("*]" if (self.highlight == "hole_top" or self.highlight == "width") else "]")
        )
        # (10)
        temp_string = (
            temp_string
            + stringify(self.values["width"] - self.values["hole_top"] - self.values["hole_left_top"])
            + ("*;" if self.highlight == "width" else ";")
        )

        # The right section of the field. (11)
        temp_string = (
            temp_string
            + stringify(-self.values["hole_top_right"])
            + ("*" if (self.highlight == "height_right" or self.highlight == "hole_top_right") else "")
        )
        # (12)
        temp_string = (
            temp_string
            + "["
            + stringify(-self.values["height_right"] + self.values["hole_bottom_right"] + self.values["hole_top_right"])
            + ("*]" if (self.highlight == "height_right") else "]")
        )
        # (13)
        temp_string = (
            temp_string
            + stringify(-self.values["hole_bottom_right"])
            + ("*;" if (self.highlight == "height_right" or self.highlight == "hole_bottom_right") else ";")
        )

        # Right insertion zone. Just as the left insertion zone, just in reverse order. (14)
        temp_string = (
            temp_string
            + stringify(-self.values["insert_top_right"])
            + ("*" if self.highlight == "insert_top_right" else "")
        )
        # (15)
        temp_string = (
            temp_string
            + "["
            + stringify(-self.values["insert_right_hole"])
            + ("*];" if self.highlight == "insert_right_hole" else "];")
        )

        y_offset = self.values["insert_right_height"] - self.values["insert_right_height_wall"]
        temp_string = temp_string + "[" + stringify(-y_offset) + "]"

        # (16)
        temp_string = (
            temp_string
            + stringify(-self.values["insert_right_height_wall"])
            + ("*;" if self.highlight == "insert_right_height_wall" else ";")
        )
        # (17)
        temp_string = (
            temp_string
            + stringify(self.values["insert_right_width_wall"])
            + ("*;" if self.highlight == "insert_right_width_wall" else ";")
        )

        # The long line streak ended. Now draw the middle bottom wall.
        # As the middle of the middle bottom wall naturally deserves to be at (0,0),
        # computation of the starting coordinate is easy.
        temp_string = temp_string + "(" + stringify(self.values["middle_extent"] / 2) + "," + stringify(0) + ")"
        # (18)
        temp_string = (
            temp_string
            + stringify(-self.values["middle_extent"])
            + ("*;" if self.highlight == "middle_extent" else ";")
        )

        # now more optional highlighted vacancies follow.
        # the distance between the middle bottom wall (18) and the left insertion zone (2)
        if self.highlight == "middle_hole_left":
            temp_string = temp_string + ";[" + stringify(-self.values["middle_hole_left"]) + "*];"

        # the distance between the middle bottom wall and the top wall
        if self.highlight == "middle_height":
            temp_string = temp_string + "(0,0);[" + stringify(self.values["middle_height"]) + "*];"

        # the height of the left insertion zone, thus the distance between its bottom (1) and top wall (4).
        if self.highlight == "insert_left_height":
            temp_string = (
                temp_string
                + "("
                + stringify(left_start_x + self.values["insert_left_width_wall"] / 2)
                + ","
                + stringify(left_start_y)
                + ");["
                + stringify(self.values["insert_left_height"])
                + "*];"
            )

        # the height of the right insertion zone, thus the distance between its bottom (17) and top wall (14).
        # The computation of the starting point is slightly more complex here, as its x position depends on the complete
        # first line streak. The position of the left insertion zone is known relative to the origin,
        # however the position of the right insertion zone is only known relative to the top wall,
        # which is relative to the left insertion zone.
        if self.highlight == "insert_right_height":
            right_start_x = (
                -self.values["middle_extent"] / 2
                - self.values["middle_hole_left"]
                - self.values["insert_left_hole"]
                - self.values["insert_top_left"]
                + self.values["width"]
                - self.values["insert_top_right"]
                - self.values["insert_right_hole"]
                + self.values["insert_right_width_wall"]
            )
            right_start_y = (
                self.values["middle_height"] - self.values["height_right"] - self.values["insert_right_height"]
            )
            temp_string = (
                temp_string
                + "("
                + stringify(right_start_x - self.values["insert_right_width_wall"] / 2)
                + ","
                + stringify(right_start_y)
                + ");["
                + stringify(self.values["insert_right_height"])
                + "*];"
            )

        self.map_string = temp_string
        self.window.mapString.setText(self.map_string)

        self.update()

    def update(self):
        self.show_command(self.map_string)

    # function the save the map image and some more helping information
    # the origin is displayed in a additional message box
    def save_map(self):
        self.dehighlight()
        self.image.save("map.png")
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        msg.setText("The origin is at {}".format(self.origin))
        msg.setWindowTitle("Origin position")
        msg.setStandardButtons(QMessageBox.Ok)

        msg.exec_()

        with open("map_creation.txt", "w") as f:
            f.write("Origin:\n")
            f.write(str(self.origin))
            f.write("\nMap String:\n")
            f.write(self.map_string)
        with open("values.yaml", "w") as f:
            yaml.dump(self.values, f, default_flow_style=False)


if __name__ == "__main__":
    MapTool()
