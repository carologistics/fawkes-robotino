#! /usr/bin/bash
if [ $# -ne 2 ]; then
  echo "Expect two arguments, 1. txt file, 2. pcd file"
  echo "The pcd file should still include the header"
  echo "The txt file should not include any header"
  exit 1
fi

sed "/^[^A-Z#]/d" -i $2
size=$(wc -l $1 | grep -o "[0-9]*")
echo $size
sed "s/POINTS .*/POINTS $size/" -i $2
sed "s/WIDTH .*/WIDTH $size/" -i $2
cat $1 >> $2

exit 0
