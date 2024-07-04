import cv2
import os, shutil
import tqdm
import zipfile
import yaml
import numpy as np
import requests
import imgaug as ia
import imgaug.augmenters as iaa
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage
from roboflow import Roboflow

#region Params
############################################## PARAMETERS ##############################################

#get images for augmentation from coco
if os.path.isdir('test2017'):
    pass
else:
    url = "http://images.cocodataset.org/zips/test2017.zip"
    zip_filename = 'test2017.zip'
    response = requests.get(url, stream=True)
    if response.status_code == 200:
        total_size = int(response.headers.get('content-length', 0))
        with open(zip_filename, 'wb') as file, tqdm(
            desc='Downloading',
            total=total_size,
            unit='B',
            unit_scale=True,
            unit_divisor=1024,
        ) as bar:
            for chunk in response.iter_content(chunk_size=1024):
                file.write(chunk)
                bar.update(len(chunk))
        print('File downloaded successfully.')

        with zipfile.ZipFile(zip_filename, 'r') as zip_file:
            zip_file.extractall('test2017')
        print('File extracted successfully.')

        os.remove(zip_filename)
        print('Original ZIP file deleted.')
    else:
        print('Failed to download file. Status code:', response.status_code)
    
#get data from roboflow
if os.path.isdir('workpiece_detection-2'):
    pass
else: 
    rf = Roboflow(api_key="XkvHB2mdjqJZJ1qSpE8K")
    project = rf.workspace("carologistics").project("workpiece_detection-zsaw1")
    version = project.version(2)
    dataset = version.download("yolov8")
    print ("Successfully downloaded from roboflow to workpiece_detection-2")

# path with all labeled data
data_path = 'workpiece_detection-2/'

# path for augmented data set
augmented_path = "augmented_set/"

# save path for training, validation and test set
training_path = f'{augmented_path}train/'
test_path = f'{augmented_path}test/'
validation_path = f'{augmented_path}valid/'

txt_path_train = f'{training_path}labels/'
img_path_train = f'{training_path}images/'
txt_path_valid = f'{validation_path}labels/'
img_path_valid = f'{validation_path}images/'
txt_path_test = f'{test_path}labels/'
img_path_test = f'{test_path}images/'


# path for images to morph with
rnd_image_path = "test2017/"

# rename image: new_img_name + img_count (starting from start_value)
new_img_name = 'rc_objects_'
start_value = 4818

# number of copies per image, rotated 90° after each
num_copy = 8

# id from which to start using the random images
rnd_id = 0

#create folder if needed
for folder in [img_path_train, txt_path_train, img_path_valid, txt_path_valid, img_path_test, txt_path_test]:
    try:
        os.makedirs(folder)
    except OSError:
        print ("Creation of the folder %s failed" % folder)
    else:
        print ("Successfully created the folder %s " % folder)

#copy data.yaml for training to Augmented_set
if os.path.isdir('workpiece_detection-2'):
    with open("workpiece_detection-2/data.yaml") as f:
        list_doc = yaml.safe_load(f)
        cwd = os.getcwd()
        list_doc['test'] = f"{cwd}/augmented_set/test/images"
        list_doc['train'] = f"{cwd}/augmented_set/train/images"
        list_doc['val'] = f"{cwd}/augmented_set/valid/images"
    f.close()

    with open("augmented_set/data.yaml", "w") as f:
        yaml.dump(list_doc, f)
        print ("Successfully created the file %s " % f.name)
    f.close()
else:
    pass

# augmentation sequence
seq = iaa.Sequential([
    iaa.CropAndPad(percent=(-0.15, 0.25)),
    iaa.Sometimes(0.5, iaa.BlendAlphaBoundingBoxes(None, 
        background=iaa.BlendAlphaRegularGrid(nb_rows=(7, 30), nb_cols=(6, 24),
            foreground=iaa.Multiply(0.0),
            alpha=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))),
    iaa.Sometimes(0.5, iaa.CoarseDropout(0.02, size_percent=0.5))
])
#endregion

#region Prep Data
############################################## PREPARE DATA ##############################################

# Setting into training, validation and test set
train_entries = os.listdir(f'{data_path}train/images/')
test_entries = os.listdir(f'{data_path}test/images/')
valid_entries = os.listdir(f'{data_path}valid/images/')
training_set = []
test_set = []
valid_set = []
for image_file in train_entries:
    if(image_file.endswith(".jpg")):
        training_set.append(image_file)

for image_file in test_entries:
    if(image_file.endswith(".jpg")):
        test_set.append(image_file)

for image_file in valid_entries:
    if(image_file.endswith(".jpg")):
        valid_set.append(image_file)


# rename them as new_img_name + img_count and save them in training_set, valid_set and test_set folder
img_count = start_value
for file in training_set:
    original_file_name = file[:-4]
    new_file_name = new_img_name+str(img_count)
    
    img = cv2.imread(f'{data_path}train/images/'+original_file_name+".jpg")
    cv2.imwrite(f'{training_path}images/'+new_file_name+".jpg", img)

    
    with open(training_path+'labels/'+new_file_name+".txt", 'w') as new_file:
        with open(f'{data_path}train/labels/'+original_file_name+".txt", 'r') as original_file:
            new_file.write(original_file.read())
    img_count += 1

print("Training set created")

for file in valid_set:
    original_file_name = file[:-4]
    new_file_name = new_img_name+str(img_count)
    
    img = cv2.imread(f'{data_path}valid/images/'+original_file_name+".jpg")
    cv2.imwrite(validation_path+'images/'+new_file_name+".jpg", img)
    
    with open(validation_path+'labels/'+new_file_name+".txt", 'w') as new_file:
        with open(f'{data_path}valid/labels/'+original_file_name+".txt", 'r') as original_file:
            new_file.write(original_file.read())
    img_count += 1

print("Validation set created")

for file in test_set:
    original_file_name = file[:-4]
    new_file_name = new_img_name+str(img_count)
    
    img = cv2.imread(f'{data_path}test/images/'+original_file_name+".jpg")
    cv2.imwrite(test_path+'images/'+new_file_name+".jpg", img)
    
    with open(test_path+'labels/'+new_file_name+".txt", 'w') as new_file:
        with open(f'{data_path}test/labels/'+original_file_name+".txt", 'r') as original_file:
            new_file.write(original_file.read())
    img_count += 1

print("Test set created")
#endregion

#region Augmentation
############################################## AUGMENTATION ##############################################

entries = os.listdir(f'{training_path}images/')
rnd_images = os.listdir(rnd_image_path)

for image_file in entries:
    if(not image_file.endswith(".jpg")):
        continue
    #load image
    img = cv2.imread(f'{training_path}images/'+image_file)
    img_height = img.shape[0]
    img_width = img.shape[1]

    #load label
    original_file_name = image_file[:-4]
    label_file = original_file_name + ".txt"
    label = open(f'{training_path}labels/'+label_file, "r")
    yolo_labels = label.readlines()
    label.close()
    
    #convert into imgaug bounding box
    box_array = []
    for box in yolo_labels:
        bb_values = [bb.strip() for bb in box.split()]
        class_number = int(bb_values[0])
        yolo_x = float(bb_values[1])
        yolo_y = float(bb_values[2])
        yolo_width = float(bb_values[3])
        yolo_height = float(bb_values[4])

        box_width = yolo_width * img_width
        box_height = yolo_height * img_height
        x_min = int(yolo_x * img_width - (box_width / 2))
        y_min = int(yolo_y * img_height - (box_height / 2))
        x_max = int(yolo_x * img_width + (box_width / 2))
        y_max = int(yolo_y * img_height + (box_height / 2))
        box_array.append(BoundingBox(x_min, y_min, x_max, y_max, label=class_number))
    bbs = BoundingBoxesOnImage(box_array, shape=img.shape)

    black_pixels = np.where(
        (img[:, :, 0] == 0) & 
        (img[:, :, 1] == 0) & 
        (img[:, :, 2] == 0)
    )
    img[black_pixels] = [1,1,1]
    
    for i in range(num_copy):
        
        ### augmentation ###
        
        #rotate by i * 90°
        rot_image = iaa.Rot90((i), keep_size=False)
        image_rot, bbs_rot = rot_image(image=img, bounding_boxes=bbs)
        
        #apply augmentation sequence
        image_aug, bbs_aug = seq(image=image_rot, bounding_boxes=bbs_rot)
        #clip bounding boxes to image and remove them if <= 50% are left
        bbs_aug = bbs_aug.remove_out_of_image_fraction(0.5).clip_out_of_image()

        #convert back and safe
        dheight = 1.0/image_aug.shape[0]
        dwidth = 1.0/image_aug.shape[1]
        file_name = original_file_name+"_"+str(i)
        with open(f'{training_path}labels/'+file_name+".txt", 'a') as label_file:
            for bb in bbs_aug:
                class_number = bb.label
                yolo_x = bb.center_x * dwidth
                yolo_y = bb.center_y * dheight
                yolo_width = bb.width * dwidth
                yolo_height = bb.height * dheight
                yolo_label = str(class_number)[:8]+" "+str(yolo_x)[:8]+" "+str(yolo_y)[:8]+" "+str(yolo_width)[:8]+" "+str(yolo_height)[:8]+"\n"

                label_file.write(yolo_label)
                
        # morph 2 images using black values [0,0,0] as mask
        rnd_img = cv2.imread(rnd_image_path+rnd_images[rnd_id])
        morph_image = cv2.resize(rnd_img, (image_aug.shape[1],image_aug.shape[0]))
        aug_pixels = np.where(
            (image_aug[:, :, 0] == 0) & 
            (image_aug[:, :, 1] == 0) & 
            (image_aug[:, :, 2] == 0)
        )

        # set those pixels to the random image
        image_aug[aug_pixels] = morph_image[aug_pixels]

        rnd_id += 1
        cv2.imwrite(f'{training_path}images/'+file_name+".jpg", image_aug)

print("Augmentation done")
#endregion

#region Delete original dataset
if os.path.isdir(data_path):
    shutil.rmtree(data_path)
    print('Deleted original dataset')
#endregion