#! /usr/bin/bash

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.

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
