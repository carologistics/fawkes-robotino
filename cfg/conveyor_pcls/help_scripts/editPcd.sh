#! /usr/bin/bash
if [ $# -ne 1 ]; then
  echo "Expect one argument: pcd file"
  exit 1
fi
echo "Save the file in the same file!"

path=$( realpath $0)
path=$(dirname $path)

cp $1 $1.orig
$path/pcd2txt.sh $1 $1.txt
CloudCompare $1.txt || exit 1
$path/txt2pcd.sh $1.txt $1
