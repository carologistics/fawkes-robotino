#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  Copyright © RoboFlow
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

%pip install requests
import glob
from roboflow import Roboflow

# Initialize Roboflow client
rf = Roboflow(api_key="")

# Directory path and file extension for images
dir_name = "PATH/TO/IMAGES"
file_extension_type = ".jpg"

# Annotation file path and format (e.g., .coco.json)
annotation_filename = "PATH/TO/_annotations.coco.json"

# Get the upload project from Roboflow workspace
project = rf.workspace("carologistics").project("workpiece_detection-zsaw1")

# Upload images
image_glob = glob.glob(dir_name + '/*' + file_extension_type)
for image_path in image_glob:
    print(project.single_upload(
        image_path=image_path,
        annotation_path=annotation_filename,
        # optional parameters:
        # annotation_labelmap=labelmap_path,
        # split='train',
        # num_retry_uploads=0,
        # batch_name='batch_name',
        # tag_names=['tag1', 'tag2'],
        # is_prediction=False,
    ))
