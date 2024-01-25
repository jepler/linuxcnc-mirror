#!/usr/bin/env python
"""
Based on the control script in m61_test
"""

import linuxcnc
from linuxcnc_control import  LinuxcncControl
import sys

from os import listdir
from os.path import isfile, join
import re
import glob


def find_test_nc_files(testpath='../nc_files', show=False):
    files = [join(testpath, f) for f in listdir(testpath) if isfile(join(testpath, f)) and re.search('.ngc$|.nc$|.TAP$', f) is not None]
    if show:
        print "In folder {0}, found {1} files:".format(testpath,len(files))
        for fname in files:
            print fname
    return files


if __name__ == '__main__':
    """ Run batch tests (user specifies which ones interactively) """
    run_sts = 0

    # raw_input("Press any key when the LinuxCNC GUI has loaded")
    # KLUDGE this lists all subfolders in the auto-test directory
    # this dir should be populated with symlinks to any folders of test files to
    # run
    test_folders = glob.glob(join('nc_files', '*'))

    test_files = []

    for f in test_folders:
        test_files.extend(find_test_nc_files(f, True))

    e = LinuxcncControl(continue_after_error=args.continue_after_error)

    e.do_startup(need_home=True)

    error_list = []
    for f in test_files:
        e.write_error_log()

        print "Loading program {0}".format(f)
        e.set_mode(linuxcnc.MODE_AUTO)
        e.open_program(f)
        e.set_mode(linuxcnc.MODE_AUTO)
        e.run_full_program()
        if not e.wait_on_program(f) and e.continue_after_error:
            e.do_startup(need_home=False)
        else:
            break

    # KLUDGE just in case any others creep in
    e.flush_errors()
    e.write_error_log()
    sys.exit(e.has_error())
