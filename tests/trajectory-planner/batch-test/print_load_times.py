#!/usr/bin/env python
import datetime
import re

if __name__ == '__main__':
    with open('batch-test-err.log') as err_log:
        t_old = datetime.datetime(1970, 1, 1)
        for l in err_log.readlines():
            dt_str = l.split('|')[0].rstrip()
            try:
                t = datetime.datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S.%f")
                msg = l.split('|')[1].rstrip()
            except ValueError:
                continue

            if re.search('loading', l):
                # print dt before recalculating (since dt here is for the next program
                dt = (t-t_old).total_seconds()
                print("{} seconds | {}".format(dt, msg))
                t_old = t
