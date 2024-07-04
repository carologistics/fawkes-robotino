#Camera Calibration & Checkerboard Generation
https://markhedleyjones.com/projects/calibration-checkerboard-collection
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
k=XkvHB2mdjqJZJ1qSpE8K

#Roboflow Online Annotation Tool (Preferred)
https://app.roboflow.com/
##Use Matteo's Access 

#Label Studio Offline Annotation Tool
# Install the package
pip install -U label-studio
label-studio
##Easy Upload to Roboflow

#Runnning Yolo Train script
1. Run yolo_training.py script to train on images but adjust the directory names in the dataset.yaml 
   to the correct location of the train, validation and test sets in your local environment
2. Use Yolov8n.pt when you want to train from scratch. But prefer to use the already trained weights from previous training.
