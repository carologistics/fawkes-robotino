#! /usr/bin/bash
if [ $# -ne 2 ]; then
  echo "Expect two arguments, 1. pcd file, 2. txt file"
  exit 1
fi

sed "/^[A-Z#]/d"  $1 > $2

exit 0
