#!/usr/bin/env python
"""
Based on the control script in m61_test
"""

# KLUDGE bake-in the path to common configs folder
import sys
sys.path.append('../common-config')

import linuxcnc
from linuxcnc_control import  LinuxcncControl

from os import listdir
from os.path import isfile, join
import re
import glob


def find_test_nc_files(testpath='../nc_files', show=False):
    files = [join(testpath, f) for f in listdir(testpath) if isfile(join(testpath, f)) and re.search('.ngc$|.nc$|.TAP$', f) is not None]
    if show:
        for fname in files:
            print fname
    return files


if __name__ == '__main__':
    test_folders = glob.glob(join('nc_files', '*'))

    test_files = []

    for f in test_folders:
        test_files.extend(find_test_nc_files(f, True))

    e = LinuxcncControl()

    e.do_startup(need_home=True)

    error_list = []
    for f in test_files:
        e.log(f, 'info', 'loading program')
        e.set_mode(linuxcnc.MODE_AUTO)
        e.open_program(f)
        e.set_mode(linuxcnc.MODE_AUTO)
        e.run_full_program()
        if not e.wait_on_program(f):
            e.do_startup(need_home=False)

    # KLUDGE just in case any others creep in
    e.flush_errors()
    sys.exit(e.has_error())
