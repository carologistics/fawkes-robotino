#! /usr/bin/bash
# While recording a model for later conveyor_pose_recognition, sometimes
# small mistakes in rotation or translation happen.
# Further, the origin position of different models must be exactly
# equal, to make use of common settings for different models.
#
# Apart from that, recorded models sometimes contain unwanted points of
# e.g. cables. Instead of filtering them out during recording, one can
# easily delete them during postprocessing the models.
#
# A simple tool to do above tasks is CloudCompare. As it cannot read
# natively the .pcd files, the below script first converts the pcd file
# into a readable file, opens CloudCompare and converts the file back
# after succesfully closing CloudCompare.
#
# Call it as ./editPcd.sh <fileToEdit>

if [ $# -ne 1 ]; then
  echo "Expect one argument: pcd file"
  exit 1
fi
echo "Save the file in the same file!"

path=$( realpath $0)
path=$(dirname $path)

cp $1 $1.orig
$path/pcd2txt.sh $1 $1.txt
CloudCompare $1.txt
$path/txt2pcd.sh $1.txt $1
