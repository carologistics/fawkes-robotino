%pip install requests
import glob
from roboflow import Roboflow

# Initialize Roboflow client
rf = Roboflow(api_key="XkvHB2mdjqJZJ1qSpE8K")

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
