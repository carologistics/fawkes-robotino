# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tim Wendt
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************


#!/bin/python3

import sys
import subprocess
import os
from threading import Thread, Lock

exit_flag = False
exit_code = 0
exit_lock = Lock()

def follow_file(file):
    global exit_flag, exit_code

    if not os.path.exists(file):
        with exit_lock:
            print(f"File {file} does not exist!")
            exit_flag = True
            exit_code = 1
        return

    command = ['tail', '-f', file]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    for line in iter(process.stdout.readline, ''):
        if "SIMTEST: Finished" in line:
            print(line, end='')
            with exit_lock:
                exit_flag = True
                exit_code = 2
            break
        elif "SIMTEST: End" in line:
            print(line, end='')
            with exit_lock:
                exit_flag = True
                exit_code = 3
            break
        elif "SIMTEST: FAILED" in line:
            print(line, end='')
            with exit_lock:
                exit_flag = True
                exit_code = 4
            break
        elif "[EXCEPTION]" in line:
            print(line, end='')
            with exit_lock:
                exit_flag = True
                exit_code = 5
            break
        elif "SIMTEST: SUCCEEDED" in line:
            print(line, end='')
            with exit_lock:
                exit_flag = True
                exit_code = 0
            break
    process.stdout.close()
    process.terminate()

if __name__ == '__main__':
    files = sys.argv[1:]
    threads = []
    for file in files:
        thread = Thread(target=follow_file, args=(file,))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    if exit_flag:
        sys.exit(exit_code)
