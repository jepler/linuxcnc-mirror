#!/usr/bin/env python
"""Copied from m61-test"""

import linuxcnc
import os
import sys
import re
from time import sleep


class LinuxcncError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class CommandTimeoutError(Exception):
    def __init__(self, message):
        # Call the base class constructor with the parameters it needs
        self.message = message

    def __str__(self):
        return self.message


class LinuxcncControl:
    """
    issue G-Code commands
    make sure important modes are saved and restored
    mode is saved only once, and can be restored only once

    usage example:
        e = emc_control()
        e.prepare_for_mdi()
            any internal sub using e.g("G0.....")
        e.finish_mdi()

    """

    def __init__(self, timeout=2, max_runtime=24 * 60 * 60, throw_lcnc_exceptions=False, continue_after_error=True):
        self.c = None
        self.e = None
        self.s = None
        self.cmd_timeout = timeout
        self.error_list = []
        self.raise_on_error = throw_lcnc_exceptions
        # Ignore blank errors by default
        self.err_ignore_mask = '^$'
        self.try_init(ignore_error=True)
        # Don't allow a test to run for more than
        self.max_runtime = max_runtime
        self.continue_after_error = continue_after_error
        self.has_error_ = False

    def try_init(self, ignore_error=False):
        try:
            self.c = linuxcnc.command()
            self.e = linuxcnc.error_channel()
            self.s = linuxcnc.stat()
            self.s.poll()
        except linuxcnc.error as e:
            if ignore_error:
                return False

        return True

    def wait_for_backend(self, timeout_sec):
        k = 0
        print('Waiting for LinuxCNC...')
        while k < 25:
            k += 1
            sleep(1)
            if self.try_init(ignore_error=True):
                return True

        return False

    def await_done(self, timeout=None):
        sts = self.c.wait_complete(timeout if timeout is not None else self.cmd_timeout)
        if sts == -1:
            raise CommandTimeoutError("command exceeded timeout of {}".format(self.cmd_timeout))
        return sts

    def running(self, do_poll=True):
        """
        check if interpreter is running.
        If so, cant switch to MDI mode.
        """
        if do_poll:
            self.s.poll()
        return (self.s.task_mode == linuxcnc.MODE_AUTO and
                self.s.interp_state != linuxcnc.INTERP_IDLE)

    def set_mode(self, m):
        """
        set EMC mode if possible, else throw LinuxcncError
        """
        self.s.poll()
        if self.s.task_mode == m :
            return 1
        if self.running(do_poll=False):
            raise LinuxcncError("interpreter running - cant change mode")
        self.c.mode(m)
        return self.await_done()


    def set_state(self, m):
        """
        set EMC mode if possible, else throw LinuxcncError
        """
        self.s.poll()
        if self.s.task_mode == m :
            return 1
        self.c.state(m)
        return self.await_done()

    def do_home(self, axismask):
        self.s.poll()
        self.c.home(axismask)
        return self.await_done()

    def ok_for_mdi(self):
        """
        check wether ok to run MDI commands.
        """
        self.s.poll()
        return not self.s.estop and self.s.enabled and self.s.homed

    def prepare_for_mdi(self):
        """
        check wether ok to run MDI commands.
        throw  LinuxcncError if told so.
        return current mode
        """

        self.s.poll()
        if self.s.estop:
            raise LinuxcncError("machine in ESTOP")

        if not self.s.enabled:
            raise LinuxcncError("machine not enabled")

        if not self.s.homed:
            raise LinuxcncError("machine not homed")

        if self.running():
            raise LinuxcncError("interpreter not idle")

        return self.set_mode(linuxcnc.MODE_MDI)

    g_raise_except = True

    def g(self, code, wait=False):
        """
        issue G-Code as MDI command.
        wait for completion if reqested
        """

        self.c.mdi(code)
        if wait:
            try:
                while self.c.wait_complete(self.cmd_timeout) == -1:
                    pass
                return True
            except KeyboardInterrupt:
                self.error_list.append("warning: interrupted by keyboard in c.wait_complete(self.timeout)")
                return False

        self.poll_and_log_error(code)
        return False

    def get_current_tool(self):
        self.s.poll()
        return self.s.tool_in_spindle

    def active_codes(self):
        self.s.poll()
        return self.s.gcodes

    def get_current_system(self):
        g = self.active_codes()
        for i in g:
                if i >= 540 and i <= 590:
                        return i/10 - 53
                elif i >= 590 and i <= 593:
                        return i - 584
        return 1

    def open_program(self, filename):
        """Open an nc file"""
        self.s.poll()
        self.set_mode(linuxcnc.MODE_AUTO)
        full_file = os.path.join(os.getcwd(), filename)
        self.c.program_open(full_file)
        return self.await_done()

    def run_full_program(self):
        """Start a loaded program"""
        self.s.poll()
        self.c.auto(linuxcnc.AUTO_RUN, 0)
        # WARNING: does not await completion of the command (use wait_on_program for this)

    def set_feed_scale(self, scale):
        """Assign a feed scale"""

        self.s.poll()
        self.c.feedrate(scale)
        return self.await_done()

    def wait_on_program(self, f):
        iter_count = 0
        update_rate = 50.  # Hz
        max_iter = int(update_rate * self.max_runtime)
        while iter_count < max_iter:
            self.s.poll()
            self.flush_errors(f)
            if self.s.task_state != linuxcnc.STATE_ON:
                self.error_list.append('{}: error: motion disabled'.format(f))
                return False

            if self.s.exec_state == linuxcnc.EXEC_ERROR or self.s.state == linuxcnc.RCS_ERROR:
                self.error_list.append('{}: warning: unhandled linuxcnc error, exec_state = {}, rcs_state = {}, aborting test'.format(f, self.s.exec_state, self.s.state))
                return False

            if self.s.exec_state == linuxcnc.EXEC_DONE and self.s.state == linuxcnc.RCS_DONE:
                return True

            sleep(1. / update_rate)
        else:
            self.error_list.append('{}: error: exceeded max allowed run time of {} seconds'.format(f, self.max_runtime))
            return False

    def run_until_done(self, f):
        self.set_mode(linuxcnc.MODE_AUTO)
        self.open_program(f)
        self.set_mode(linuxcnc.MODE_AUTO)
        self.run_full_program()
        return self.wait_on_program(f)

    def check_rcs_error(self):
        self.s.poll()
        if self.s.state == linuxcnc.RCS_ERROR:
            return True
        return False

    def set_error_ignore_pattern(self, pattern):
        self.err_ignore_mask = pattern

    def poll_and_log_error(self, f):
        error = self.e.poll()
        if error is None:
            return False

        err_type, msg_text = error
        err_type_str = "error" if err_type in (linuxcnc.NML_ERROR, linuxcnc.OPERATOR_ERROR) else "info"
        err_str = "{}: {}: {}".format(f, err_type_str, msg_text)
        self.error_list.append(err_str)
        if self.raise_on_error:
            raise LinuxcncError(err_str)
        return True

    def flush_errors(self, filename=""):
        while self.poll_and_log_error(filename):
            sleep(0.001)
            continue

    def has_error(self):
        self.has_error_ = self.has_error_ or self.find_new_error()
        return self.has_error_

    def find_new_error(self):
        for msg in self.error_list:
            if re.search(self.err_ignore_mask, msg):
                continue
            if re.search('error: ', msg):
                return True
        else:
            return False

    def write_error_log(self):
        sys.stderr.writelines(self.error_list)
        # Cache the error state since we flush the error list to std err
        self.has_error_ = self.has_error()
        self.error_list = []

    def do_startup(self, need_home=False):
        self.set_mode(linuxcnc.MODE_MANUAL)
        self.set_state(linuxcnc.STATE_ESTOP_RESET)
        self.set_state(linuxcnc.STATE_ON)
        if need_home:
            self.do_home(-1)
            sleep(0.1)
        self.set_mode(linuxcnc.MODE_AUTO)


