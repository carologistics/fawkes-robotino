#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  Contributor 2024 Abhirup Das<abhirup.das@rwth-aachen.de>
#
##########################################################################
#
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

from ultralytics import YOLO

def main():
    model = YOLO('yolov8n.pt')
    print("model loaded, ultra")
    output = model.train(data='data.yaml', epochs=20, batch = 32)
    print("model trained, ultra")
    new_model = YOLO('best.pt',task='detect')
    new_model.export(format="ncnn")


if __name__ == "__main__":
    main()