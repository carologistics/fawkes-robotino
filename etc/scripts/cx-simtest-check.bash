#! /bin/bash
#
# cx-simtest-check.bash
# Copyright (C) 2019-2020 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.
#

# Check the logfiles passed to the script for the status of the CX simtests.
# Collect and print the failures and exit with status 1 if a failure occurred.
# Otherwise, exit with status 0.

set -eu

check_file () {
  tail -n +0 -f $file | while read LINE ; do
    if [[ "${LINE}" == *"SIMTEST: FAILED"* ]] ; then
      echo $file
      return
    elif [[ "${LINE}" == *"[EXCEPTION]"* ]] ; then
      echo $file
      return
    elif [[ "${LINE}" == *"SIMTEST: SUCCEEDED"* ]] ; then
      return
    fi
  done
}

for file in $* ; do
  if [ ! -f $file ] ; then
    echo "File $file does not exist!"
    exit 1
  fi
  failures+=($(check_file $file))
done

if [ ${#failures[@]} -gt 0 ] ; then
  for failure in ${failures[@]} ; do
    echo "Failure in $failure"
  done
  exit 1
fi
echo "All tests successful!"
exit 0
