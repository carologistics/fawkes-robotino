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
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QValidator
import inspect


# This class bases on the standard QLineEdit and extends it
# by the signal focusInSignal, which is emitted when the user
# focus (by e.g. clicking into the field) this lineEdit.
#
# This functionality is needed to highlight the corresponding line 
# in the map, after the user chooses the value he wants to edit.
class QLineEditFocus(QLineEdit):
    focusInSignal = pyqtSignal(name="FocusIn")

    def __init__(self, parent = None):
        super().__init__(parent)

        self.soft_validator = None


        super().textChanged.connect(self.turn_conditional_red)
        super().editingFinished.connect(self.turn_white)

    def focusInEvent(self,e):
        super().focusInEvent(e)
        self.focusInSignal.emit()

    def turn_conditional_red(self):
        if not self.soft_validator:
            return
        if  self.soft_validator.validate(super().text(),0)[0] == QValidator.Invalid:
            self.setStyleSheet("QLineEdit { background: rgb(255, 0, 0);}")
    def turn_white(self):
        self.setStyleSheet("QLineEdit { background: rgb(255, 255, 255);}")

