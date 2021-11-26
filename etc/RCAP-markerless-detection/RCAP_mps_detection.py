import cv2
import os
import imgaug as ia
import imgaug.augmenters as iaa
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage

################################ config values ################################

#path to data directory to evaluate
data_path = 'evaluation_set/'

#path to directory to save the label.txt files to
#make sure the folder is missing or empty
save_path = 'output_directory/'

mps_classes = ['BS', 'CS', 'RS', 'DS']

output_image_width = 810
output_image_height = 1440
output_line_thickness = 4
output_test_size = 25

#waits for this time between detections (in ms)
#any key press skips this
#0 = wait for key press
wait_between_detections = 5000

CONFIDENCE_THRESHOLD = 0.95
NMS_THRESHOLD = 0.5

weights_file = "yolov4_7000.weights"
config_file = "yolov4.cfg"

############################### set up network ################################

net = cv2.dnn.readNet(weights_file, config_file)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(608, 608), scale=0.00392, swapRB=True)

################################# run network #################################

#create folder if needed
try:
    os.mkdir(save_path)
except OSError:
    print ("Creation of the folder %s failed" % save_path)
else:
    print ("Successfully created the folder %s " % save_path)

entries = os.listdir(data_path)
for file in entries:
    if(file.endswith(".JPG") or file.endswith(".jpg")):
        #get image and set up input
        frame = cv2.imread(data_path+file)
        frame_height = frame.shape[0]
        frame_width = frame.shape[1]
        blob = cv2.dnn.blobFromImage(frame, 1.0, size=(608, 608), mean=(0, 0, 0), swapRB=True, crop=False)

        #run network
        classes, scores, boxes = model.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

        #set up values for conversion
        file_name = file[:-4]
        box_array = []
        i = 0
        bb_frame = frame
        bb_frame = cv2.resize(bb_frame, (output_image_width, output_image_height))
        with open(save_path+file_name+".txt", 'a') as label_file:
            for box in boxes:
                #convert into yolo labels
                yolo_x = box[0] / frame_width
                yolo_y = box[1] / frame_height
                yolo_width = box[2]  / frame_width
                yolo_height = box[3] / frame_height
                #model.detect outputs the top left corner instead of the middle point
                yolo_x =  yolo_x + yolo_width/2
                yolo_y =  yolo_y + yolo_height/2

                #write yolo label into file (creates a new one if needed)
                yolo_label = str(classes[i])+" "+str(yolo_x)[:8]+" "+str(yolo_y)[:8]+" "+str(yolo_width)[:8]+" "+str(yolo_height)[:8]+"\n"
                label_file.write(yolo_label)
                
                #convert into imgaug bounding box                
                box_width = yolo_width * output_image_width
                box_height = yolo_height * output_image_height
                x_min = int(yolo_x * output_image_width - (box_width / 2))
                y_min = int(yolo_y * output_image_height - (box_height / 2))
                x_max = int(yolo_x * output_image_width + (box_width / 2))
                y_max = int(yolo_y * output_image_height + (box_height / 2))

                #safe and draw bounding box
                bb = BoundingBox(x_min, y_min, x_max, y_max, label=mps_classes[classes[i]])
                bb_frame = bb.draw_box_on_image(image=bb_frame, size=output_line_thickness)
                box_array.append(bb)
                i += 1
        #draw labels on image
        for box in box_array:
            bb_frame = box.draw_label_on_image(image=bb_frame, size=output_line_thickness, size_text=output_test_size)

        #show image with bounding boxes
        cv2.imshow("preview", bb_frame)
        cv2.waitKey(wait_between_detections)
