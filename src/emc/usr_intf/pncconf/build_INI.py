import os
import time

class INI:
    def __init__(self,app):
        # access to:
        self.d = app.d  # collected data
        global SIG
        SIG = app._p    # private data (signals names)
        global _PD
        _PD = app._p    # private data
        self.a = app    # The parent, pncconf

    def write_inifile(self, base):
        filename = os.path.join(base, self.d.machinename + ".ini")
        file = open(filename, "w")
        print(_("# Generated by PNCconf at %s") % time.asctime(), file=file)
        print(_("# Using LinuxCNC version:  %s") % self.d.linuxcnc_version, file=file)
        print(_("# If you make changes to this file, they will be"), file=file)
        print(_("# overwritten when you run PNCconf again"), file=file)

        print(file=file)
        print("[EMC]", file=file)
        print("MACHINE = %s" % self.d.machinename, file=file)
        print("DEBUG = 0", file=file)

        # the joints_axes conversion script named 'update_ini'
        # will try to update for joints_axes if no VERSION is set
        print("VERSION = 1.1", file=file)

        # write the qtplasmac section
        if self.d.frontend == _PD._QTPLASMAC:
            self.write_qtplasmac_section(file)

        print(file=file)
        print("[DISPLAY]", file=file)
        if self.d.frontend == _PD._AXIS:
            print("DISPLAY = axis", file=file)
        elif self.d.frontend == _PD._TKLINUXCNC:
            print("DISPLAY = tklinuxcnc", file=file)
        elif self.d.frontend == _PD._GMOCCAPY:
            print("DISPLAY = gmoccapy", file=file)
        elif self.d.frontend == _PD._TOUCHY:
            print("DISPLAY = touchy", file=file)
        elif self.d.frontend == _PD._QTDRAGON:
            print("DISPLAY = qtvcp qtdragon", file=file)
            print("PREFERENCE_FILE_PATH = WORKINGFOLDER/qtdragon.pref", file=file)
        # qtplasmac has multiple screens
        elif self.d.frontend == _PD._QTPLASMAC:
            if self.d.qtplasmacscreen == 2:
                screen = "qtplasmac_9x16"
            elif self.d.qtplasmacscreen == 1:
                screen = "qtplasmac_4x3"
            else:
                screen = "qtplasmac"
            print("DISPLAY = qtvcp {}".format(screen), file=file)
        if self.d.gladevcp:
            theme = self.d.gladevcptheme
            if theme == "Follow System Theme":theme = ""
            else: theme = " -t "+theme
            if self.d.frontend in(_PD._AXIS, _PD._TOUCHY):
                if self.d.centerembededgvcp:
                    print("EMBED_TAB_NAME = GladeVCP", file=file)
                    print("EMBED_TAB_COMMAND = halcmd loadusr -Wn gladevcp gladevcp -c gladevcp%s -H gvcp_call_list.hal -x {XID} gvcp-panel.ui"%(theme), file=file)
                elif self.d.sideembededgvcp:
                    print("GLADEVCP =%s -H gvcp_call_list.hal gvcp-panel.ui"%(theme), file=file)
            elif self.d.frontend == _PD._GMOCCAPY:
                if self.d.centerembededgvcp:
                    print("EMBED_TAB_NAME = Center_panel", file=file)
                    print("EMBED_TAB_LOCATION = ntb_preview", file=file)
                elif self.d.sideembededgvcp:
                    print("EMBED_TAB_NAME = Right Panel", file=file)
                    print("EMBED_TAB_LOCATION = box_right", file=file)
                else:
                    print("EMBED_TAB_NAME = User Panel", file=file)
                    print("EMBED_TAB_LOCATION = ntb_user_tabs", file=file)
                print("EMBED_TAB_COMMAND = gladevcp -c gladevcp %s -H gvcp_call_list.hal -x {XID} gvcp-panel.ui"%(theme), file=file)

        if self.d.position_offset == 1: temp ="RELATIVE"
        else: temp = "MACHINE"
        print("POSITION_OFFSET = %s"% temp, file=file)
        if self.d.position_feedback == 1: temp ="ACTUAL"
        else: temp = "COMMANDED"
        print("POSITION_FEEDBACK = %s"% temp, file=file)
        print("MAX_FEED_OVERRIDE = %f"% self.d.max_feed_override, file=file)

        if self.d.frontend == _PD._QTDRAGON:
            print("MAX_SPINDLE_0_OVERRIDE = %f"% self.d.max_spindle_override, file=file)
            print("MIN_SPINDLE_0_OVERRIDE = %f"% self.d.min_spindle_override, file=file)
            print("DEFAULT_SPINDLE_0_SPEED = 500", file=file)
            print("MIN_SPINDLE_0_SPEED = 100", file=file)
            print("MAX_SPINDLE_0_SPEED = 2500", file=file)
        # qtplasmac doesn't use spindle override
        elif self.d.frontend != _PD._QTPLASMAC:
            print("MAX_SPINDLE_OVERRIDE = %f"% self.d.max_spindle_override, file=file)
            print("MIN_SPINDLE_OVERRIDE = %f"% self.d.min_spindle_override, file=file)

        print("INTRO_GRAPHIC = linuxcnc.gif", file=file)
        print("INTRO_TIME = 5", file=file)
        print("PROGRAM_PREFIX = %s" % \
                                    os.path.expanduser("~/linuxcnc/nc_files"), file=file)
        if self.d.pyvcp:
            print("PYVCP = pyvcp-panel.xml", file=file)
        # these are for AXIS GUI and QtPlasmaC
        if self.d.units == _PD._METRIC:
            if self.d.frontend == _PD._QTPLASMAC:
                print("INCREMENTS = %s"% self.d.increments_metric_qtplasmac, file=file)
            else:
                print("INCREMENTS = %s"% self.d.increments_metric, file=file)
        else:
            if self.d.frontend == _PD._QTPLASMAC:
                print("INCREMENTS = %s"% self.d.increments_imperial_qtplasmac, file=file)
            else:
                print("INCREMENTS = %s"% self.d.increments_imperial, file=file)
        if self.d.axes == 2:
            print("LATHE = 1", file=file)
        print("POSITION_FEEDBACK = %s"% temp, file=file)
        print("DEFAULT_LINEAR_VELOCITY = %f"% self.d.default_linear_velocity, file=file)
        print("MAX_LINEAR_VELOCITY = %f"% self.d.max_linear_velocity, file=file)
        print("MIN_LINEAR_VELOCITY = %f"% self.d.min_linear_velocity, file=file)
        print("DEFAULT_ANGULAR_VELOCITY = %f"% self.d.default_angular_velocity, file=file)
        print("MAX_ANGULAR_VELOCITY = %f"% self.d.max_angular_velocity, file=file)
        print("MIN_ANGULAR_VELOCITY = %f"% self.d.min_angular_velocity, file=file)
        # qtplasmac has internal editor
        if self.d.frontend != _PD._QTPLASMAC:
            print("EDITOR = %s"% self.d.editor, file=file)
        print("GEOMETRY = %s"% self.d.geometry, file=file)
        print("CYCLE_TIME = 100", file=file)

        # set up MDI macro buttons
        if self.d.frontend == _PD._QTDRAGON:
            print(file=file)
            print("[MDI_COMMAND_LIST]", file=file)
            print("MDI_COMMAND = G0 Z0;X0 Y0", file=file)
            print("MDI_COMMAND = G53 G0 Z0;G53 G0 X0 Y0", file=file)

        print(file=file)
        print("[FILTER]", file=file)
        # qtplasmac has a different filter section
        if self.d.frontend == _PD._QTPLASMAC:
            print("PROGRAM_EXTENSION = .ngc,.nc,.tap GCode File (*.ngc, *.nc, *.tap)", file=file)
            print("ngc = ./qtplasmac/qtplasmac_gcode.py", file=file)
            print("nc  = ./qtplasmac/qtplasmac_gcode.py", file=file)
            print("tap = ./qtplasmac/qtplasmac_gcode.py", file=file)
        else:
            print("PROGRAM_EXTENSION = .png,.gif,.jpg Greyscale Depth Image", file=file)
            print("PROGRAM_EXTENSION = .py Python Script", file=file)
            print("png = image-to-gcode", file=file)
            print("gif = image-to-gcode", file=file)
            print("jpg = image-to-gcode", file=file)
            print("py = python", file=file)

        print(file=file)
        print("[TASK]", file=file)
        print("TASK = milltask", file=file)
        print("CYCLE_TIME = 0.010", file=file)

        print(file=file)
        print("[RS274NGC]", file=file)
        print("PARAMETER_FILE = linuxcnc.var", file=file)
        # qtplasmac has extra rs274ngc variables
        if self.d.frontend == _PD._QTPLASMAC:
            if self.d.units == _PD._METRIC:
                units = "metric"
            else:
                units = "imperial"
            print("RS274NGC_STARTUP_CODE = o<{}_startup> call".format(units), file=file)
            print("SUBROUTINE_PATH = ./:./qtplasmac:../../nc_files/subroutines", file=file)
            print("USER_M_PATH = ./:./qtplasmac", file=file)
            print("", file=file)

        #base_period = self.d.ideal_period()

        print(file=file)
        print("[EMCMOT]", file=file)
        print("EMCMOT = motmod", file=file)
        print("COMM_TIMEOUT = 1.0", file=file)
        #print("BASE_PERIOD = %d" % self.d.baseperiod, file=file)
        print("SERVO_PERIOD = %d" % self.d.servoperiod, file=file)
        print(file=file)
        print("[HMOT]", file=file)
        if not self.d.useinisubstitution:
            print(_("# **** This is for info only ****"), file=file)
        print("CARD0=hm2_%s.0"% self.d.mesa0_currentfirmwaredata[_PD._BOARDNAME], file=file)
        if self.d.number_mesa == 2:
            for boardnum in range(0,int(self.d.number_mesa)):
                if boardnum == 1 and (self.d.mesa0_currentfirmwaredata[_PD._BOARDNAME] == self.d.mesa1_currentfirmwaredata[_PD._BOARDNAME]):
                    halnum = 1
                else:
                    halnum = 0
            print(file, "CARD1=hm2_%s.%d"% (self.d.mesa1_currentfirmwaredata[_PD._BOARDNAME], halnum), file=file)
        if self.d._substitution_list:
            print(_("# These are to ease setting custom component's parameters in a custom HAL file"), file=file)
            print(file=file)
            for i,temp in enumerate(self.d._substitution_list):
                a,b = self.d._substitution_list[i]
                if a =="":
                    print(file=file)
                else:
                    print("%s=%s"%(a,b), file=file)
        print(file=file)
        print("[HAL]", file=file)
        print("HALUI = halui", file=file)
        print("HALFILE = %s.hal" % self.d.machinename, file=file)
        # qtplasmac requires the qtplasmac_comp file to connect the plasmac component
        if self.d.frontend == _PD._QTPLASMAC:
            print("HALFILE = qtplasmac_comp.hal", file=file)
        print("HALFILE = custom.hal", file=file)

        if self.d.pyvcp and self.d.pyvcphaltype == 1 and self.d.pyvcpconnect:
           print("POSTGUI_HALFILE = pyvcp_options.hal", file=file)
        if self.d.serial_vfd:
            if self.d.gs2_vfd:
                print("POSTGUI_HALFILE = gs2_vfd.hal", file=file)
            if self.d.mitsub_vfd:
                print("POSTGUI_HALFILE = mitsub_vfd.hal", file=file)
        if self.d.toolchangeprompt:
            if self.d.frontend == _PD._QTDRAGON:
                print("POSTGUI_HALFILE = qtvcp_postgui.hal", file=file)
            elif self.d.frontend == _PD._GMOCCAPY:
                print("POSTGUI_HALFILE = gmoccapy_postgui.hal", file=file)
        print("POSTGUI_HALFILE = custom_postgui.hal", file=file)
        print("SHUTDOWN = shutdown.hal", file=file)
        print(file=file)
        print("[HALUI]", file=file)
        if self.d.halui == True:
            for i in range(0,15):
                cmd =self.d["halui_cmd" + str(i)]
                if cmd =="": break
                print("MDI_COMMAND = %s"% cmd, file=file)

        # Build axis/joints info

        # add X axis
        num_joints = 1; coords = "X"; tandemflag = False
        tandemjoint = self.a.tandem_check('x')
        print('tandem:',tandemjoint)
        if tandemjoint:
            #add tandem to X
            tandemflag = True
            num_joints += 1
            coords += 'X'

        if self.d.axes in (0,1): # xyz or xyyza
            # add Y axis
            num_joints += 1
            coords += 'Y'
            tandemjoint = self.a.tandem_check('y')
            if tandemjoint:
                #add tandem to Y
                tandemflag = True
                num_joints += 1
                coords += 'Y'

        # add Z axis
        num_joints += 1
        coords += 'Z'
        tandemjoint = self.a.tandem_check('z')
        if tandemjoint:
            tandemflag = True
            num_joints += 1
            coords += 'Z'

        if self.d.axes == 1: # for xyza
            # add A axis
            num_joints += 1
            coords += 'A'
            tandemjoint = self.a.tandem_check('a')
            if tandemjoint:
                # add tandem to A
                tandemflag = True
                num_joints += 1
                coords += 'A'
        if not self.d.axes in (0,1,2):
            print("___________________unknown self.d.axes",self.d.axes)
            return
        print(file=file)
        print("[KINS]", file=file)
        # trivial kinematics: no. of joints == no.of axes)
        # with trivkins, axes do not have to be consecutive
        print("JOINTS = %d"%num_joints, file=file)
        if tandemflag and self.d.frontend != _PD._QTPLASMAC:
            print("KINEMATICS = trivkins coordinates=%s kinstype=BOTH"%coords.replace(" ",""), file=file)
        else:
            print("KINEMATICS = trivkins coordinates=%s"%coords.replace(" ",""), file=file)
        print(file=file)
        print("[TRAJ]", file=file)
        # qtplasmac requires 3 spindles
        if self.d.frontend == _PD._QTPLASMAC:
            print("SPINDLES = 3", file=file)
        print("COORDINATES = ",coords, file=file)
        if self.d.axes == 1:
            print("MAX_ANGULAR_VELOCITY = %.2f" % self.d.amaxvel, file=file)
            defvel = min(60, self.d.amaxvel/10.)
            print("DEFAULT_ANGULAR_VELOCITY = %.2f" % defvel, file=file)
        if self.d.units == _PD._METRIC:
            print("LINEAR_UNITS = mm", file=file)
        else:
            print("LINEAR_UNITS = inch", file=file)
        print("ANGULAR_UNITS = degree", file=file)
        if self.d.axes == 2:
            maxvel = max(self.d.xmaxvel, self.d.zmaxvel)
        else:
            maxvel = max(self.d.xmaxvel, self.d.ymaxvel, self.d.zmaxvel)
        hypotvel = (self.d.xmaxvel**2 + self.d.ymaxvel**2 + self.d.zmaxvel**2) **.5
        defvel = min(maxvel, max(.1, maxvel/10.))
        print("DEFAULT_LINEAR_VELOCITY = %.2f" % defvel, file=file)
        print("MAX_LINEAR_VELOCITY = %.2f" % maxvel, file=file)
        if self.d.restore_joint_position:
            print("POSITION_FILE = position.txt", file=file)
        if not self.d.require_homing:
            print("NO_FORCE_HOMING = 1", file=file)
        print(file=file)
        print("[EMCIO]", file=file)
        print("EMCIO = io", file=file)
        print("CYCLE_TIME = 0.100", file=file)
        print("TOOL_TABLE = tool.tbl", file=file)
        # qtplasmac doesn't require these
        if self.d.frontend != _PD._QTPLASMAC:
            if self.d.allow_spindle_on_toolchange:
                print("TOOL_CHANGE_WITH_SPINDLE_ON = 1", file=file)
            if self.d.raise_z_on_toolchange:
                print("TOOL_CHANGE_QUILL_UP = 1", file=file)
            if self.d.random_toolchanger:
                print("RANDOM_TOOLCHANGER = 1", file=file)

        all_homes = bool(self.a.home_sig("x") and self.a.home_sig("z"))
        if self.d.axes in (0,1): all_homes = bool(all_homes and self.a.home_sig("y"))
        # A axis usually doesn't have home switches
        #if self.d.axes == 1: all_homes = all_homes and self.a.home_sig("a")

        ##############################################################
        # build axis/joint info
        ##############################################################
        # self.d.axes:
        # 0 = xyz
        # 1 = xz
        # 2 = xyza
        # todo: simplify hardcoding for trivkins sequential joint no.s

        jnum = 0
        # Always X AXIS
        self.write_one_axis(file, 'x')
        tandemjoint = self.a.tandem_check('x')
        self.write_one_joint(file, 0, "x", "LINEAR", all_homes, tandemjoint)
        if tandemjoint:
            jnum += 1
            self.write_one_joint(file, jnum, "x", "LINEAR", all_homes, True, ismain = False)
        jnum += 1
        print("#******************************************", file=file)

        # Maybe add Y AXIS
        if self.d.axes in(0,1): # xyz or xyza
            self.write_one_axis(file, 'y')
            tandemjoint = self.a.tandem_check('y')
            self.write_one_joint(file, jnum, "y", "LINEAR", all_homes, tandemjoint)
            if tandemjoint:
                jnum += 1
                self.write_one_joint(file, jnum, "y", "LINEAR", all_homes, True, ismain = False)
            jnum += 1
            print("#******************************************", file=file)

        # Always add Z AXIS
        self.write_one_axis(file, 'z')
        tandemjoint = self.a.tandem_check('z')
        self.write_one_joint(file, jnum, "z", "LINEAR", all_homes, tandemjoint)
        if tandemjoint:
            jnum += 1
            self.write_one_joint(file, jnum, "z", "LINEAR", all_homes, True, ismain = False)
        jnum += 1
        print("#******************************************", file=file)

        # Maybe add A AXIS
        if self.d.axes == 1: # xyza
            self.write_one_axis(file, 'a')
            tandemjoint = self.a.tandem_check('a')
            self.write_one_joint(file, jnum, "a", "ANGULAR", all_homes, tandemjoint)
            if tandemjoint:
                jnum += 1
                self.write_one_joint(file, jnum, "a", "ANGULAR", all_homes, True, ismain = False)
            jnum += 1
            print("#******************************************", file=file)

        # usually add SPINDLE except for qtplasmac
        if self.d.include_spindle and self.d.frontend != _PD._QTPLASMAC:
            self.write_one_joint(file, 9, "s", "null", all_homes, False)
        file.close()
        self.d.add_md5sum(filename)

    def write_one_joint(self, file, num, letter, type, all_homes, tandemflag, ismain = True):
        def get(s): return self.d[letter + s]
        pwmgen = self.a.pwmgen_sig(letter)
        tppwmgen = self.a.tppwmgen_sig(letter)
        stepgen = self.a.stepgen_sig(letter)
        encoder = self.a.encoder_sig(letter)
        resolver = self.a.resolver_sig(letter)
        potoutput = self.a.potoutput_sig(letter)

        closedloop = False
        if stepgen and (encoder or resolver): closedloop = True
        if (encoder or resolver) and (pwmgen or tppwmgen) : closedloop = True
        if closedloop and letter == "s": closedloop = False
        #print("INI ",letter + " is closedloop? "+ str(closedloop),encoder,pwmgen,tppwmgen,stepgen)

        print(file=file)
        if letter == 's':
            print("[SPINDLE_%d]" % 0, file=file)
        else:
            print("[JOINT_%d]" % num, file=file)
            print("TYPE = %s" % type, file=file)
            print("HOME = %s" % get("homepos"), file=file)
            print("FERROR = %s"% get("maxferror"), file=file)
            print("MIN_FERROR = %s" % get("minferror"), file=file)
        if not letter == "s" or (letter == "s" and stepgen):
            print("MAX_VELOCITY = %s" % get("maxvel"), file=file)
            print("MAX_ACCELERATION = %s" % get("maxacc"), file=file)
            if stepgen:
                print(_("# The values below should be 25% larger than MAX_VELOCITY and MAX_ACCELERATION"), file=file)
                print(_("# If using BACKLASH compensation STEPGEN_MAXACCEL should be 100% larger."), file=file)
                if get("usecomp") or get("usebacklash"):
                    factor = 2.0
                else:
                    factor = 1.25
                print("STEPGEN_MAXVEL = %.2f" % (float(get("maxvel")) * factor), file=file)
                print("STEPGEN_MAXACCEL = %.2f" % (float(get("maxacc")) * factor), file=file)

        print("P = %s" % get("P"), file=file)
        print("I = %s" % get("I"), file=file)
        print("D = %s" % get("D"), file=file)
        print("FF0 = %s" % get("FF0"), file=file)
        print("FF1 = %s" % get("FF1"), file=file)
        print("FF2 = %s" % get("FF2"), file=file)
        print("BIAS = %s"% get("bias"), file=file)
        print("DEADBAND = %s"% get("deadband"), file=file)
        print("MAX_OUTPUT = %s" % get("maxoutput"), file=file)
        if encoder or resolver:
            if get("invertencoder"):
                temp = -1
            else: temp = 1
            if encoder:
                print("ENCODER_SCALE = %s" % (get("encoderscale") * temp), file=file)
            else:
                print("RESOLVER_SCALE = %s" % (get("encoderscale") * temp), file=file)
        if pwmgen or potoutput:
            if get("invertmotor"):
                temp = -1
            else: temp = 1

            if letter == 's':
                if self.d.susenegativevoltage:
                    min_voltage_factor = -1
                else:
                    min_voltage_factor = 0 # dont output negative voltages
                max_voltage_factor = 10.0/self.d.soutputmaxvoltage # voltagelimit
                if self.d.suseoutputrange2:
                    scale = 1 # we scale the multiple ranges in the HAL file
                else:
                    scale = self.d.soutputscale # we scale to the max RPM
                print("OUTPUT_SCALE = %s"% ( int(max_voltage_factor * scale * temp) ), file=file)
                print("OUTPUT_MIN_LIMIT = %s"% ( int(min_voltage_factor * scale) ), file=file)
                print("OUTPUT_MAX_LIMIT = %s"% ( int(scale) ), file=file)
                print(file=file)
            else:
                print("OUTPUT_SCALE = %s" % (get("outputscale") * temp), file=file)
                pwmpinname = self.a.make_pinname(pwmgen)
                if (pwmgen and "analog" in pwmpinname) or potoutput:
                    print("OUTPUT_MIN_LIMIT = %s"% (get("outputminlimit")), file=file)
                    print("OUTPUT_MAX_LIMIT = %s"% (get("outputmaxlimit")), file=file)

        if stepgen:
            print(_("# these are in nanoseconds"), file=file)
            print("DIRSETUP   = %d"% int(get("dirsetup")), file=file)
            print("DIRHOLD    = %d"% int(get("dirhold")), file=file)
            print("STEPLEN    = %d"% int(get("steptime")), file=file)
            print("STEPSPACE  = %d"% int(get("stepspace")), file=file)
            if get("invertmotor"):
                temp = -1
            else: temp = 1
            print("STEP_SCALE = %s"% (get("stepscale") * temp), file=file)
        if letter == 's':return
        if get("usecomp"):
            print("COMP_FILE = %s" % get("compfilename"), file=file)
            print("COMP_FILE_TYPE = %s" % get("comptype"), file=file)
        if get("usebacklash"):
            print("BACKLASH = %s" % get("backlash"), file=file)
        minlim, maxlim = self.find_limits(letter)
        print("MIN_LIMIT = %s" % minlim, file=file)
        print("MAX_LIMIT = %s" % maxlim, file=file)
        thisaxishome = set(("all-limit-home", "all-home", "home-" + letter, "min-home-" + letter, "max-home-" + letter, "both-home-" + letter))
        ignore = set(("min-home-" + letter, "max-home-" + letter, "both-home-" + letter, "all-limit-home"))
        share = set(("all-limit-home", "all-home"))
        homes = False
        for i in thisaxishome:
            if self.a.findsignal(i): homes = True
        # set homing speeds and directions
        # search direction : True = positive direction
        # latch direction :  True = opposite direction
        if homes:
            searchvel = abs(get("homesearchvel"))
            latchvel = abs(get("homelatchvel"))
            #print(get("searchdir"))
            if get("searchdir") == 0:
                 searchvel = -searchvel
                 if get("latchdir") == 0:
                    latchvel = -latchvel
            else:
                if get("latchdir") == 1:
                    latchvel = -latchvel
            if ismain:
                print("HOME_OFFSET = %f" % get("homesw"), file=file)
            else:
                print("HOME_OFFSET = %f" % get("hometandemsw"), file=file)
            print("HOME_SEARCH_VEL = %f" % searchvel, file=file)
            print("HOME_LATCH_VEL = %f" % latchvel, file=file)
            print("HOME_FINAL_VEL = %f" % get("homefinalvel"), file=file)
            if get("usehomeindex"):useindex = "YES"
            else: useindex = "NO"
            print("HOME_USE_INDEX = %s" % useindex, file=file)
            for i in ignore:
                if self.a.findsignal(i):
                    print("HOME_IGNORE_LIMITS = YES", file=file)
                    break
            for i in share:
                if self.a.findsignal(i):
                    print("HOME_IS_SHARED = 1", file=file)
                    break
        else:
            print("HOME_OFFSET = %s" % get("homepos"), file=file)

        # if  user doesn't request manual individual homing, add the sequence number:
        if not self.d.individual_homing:
            seqnum = int(get("homesequence"))
            # if a tandem joint we wish to finish the home sequence together
            if tandemflag: wait ='-'
            else: wait = ''
            print("HOME_SEQUENCE = %s%d" % (wait,seqnum), file=file)

    def write_one_axis(self, file, letter):
        # For KINEMATICS_IDENTITY:
        #     use axis MIN,MAX values identical corresponding joint values
        def get(s): return self.d[letter + s]
        minlim, maxlim = self.find_limits(letter)
        if not letter == "s":
            axis_letter = letter.upper()
            print(file=file)
            print("#******************************************", file=file)
            print("[AXIS_%s]" % axis_letter, file=file)
            # qtplasmac requires double vel & acc to use eoffsets correctly
            if self.d.frontend == _PD._QTPLASMAC:
                print(_("# MAX_VEL & MAX_ACC need to be twice the corresponding joint value"), file=file)
                print("MAX_VELOCITY = %s" % (get("maxvel") * 2), file=file)
                print("MAX_ACCELERATION = %s" % (get("maxacc") * 2), file=file)
                print("OFFSET_AV_RATIO = 0.5", file=file)
            else:
                print("MAX_VELOCITY = %s" % get("maxvel"), file=file)
                print("MAX_ACCELERATION = %s" % get("maxacc"), file=file)
            print("MIN_LIMIT = %s" % minlim, file=file)
            print("MAX_LIMIT = %s" % maxlim, file=file)

    # write the qtplasmac section
    def write_qtplasmac_section(self, file):
        print(file=file)
        print("[QTPLASMAC]", file=file)
        print(_("# set the operating mode (default is 0)"), file=file)
        print("MODE = {}".format(self.d.qtplasmacmode), file=file)
        print(_("# set the estop type (0=indicator, 1=hidden, 2=button)"), file=file)
        print("ESTOP_TYPE = {}".format(self.d.qtplasmacestop), file=file)
        print(_("# set the dro position ('top' or 'bottom')"), file=file)
        dro = 'top' if self.d.qtplasmacdro else 'bottom'
        print("DRO_POSITION = {}".format(dro), file=file)
        print(_("# error message flash (0=no, 1=yes)"), file=file)
        print("FLASH_ERROR = {}".format(self.d.qtplasmacerror), file=file)
        print(_("# hide buttons (0=no, 1=yes)"), file=file)
        print("HIDE_RUN = {}".format(self.d.qtplasmacstart), file=file)
        print("HIDE_PAUSE = {}".format(self.d.qtplasmacpause), file=file)
        print("HIDE_ABORT = {}".format(self.d.qtplasmacstop), file=file)
        print(_("# laser touchoff"), file=file)
        print("#LASER_TOUCHOFF = X0.0 Y0.0", file=file)
        print(_("# camera touchoff"), file=file)
        print("#CAMERA_TOUCHOFF = X0.0 Y0.0 ", file=file)
        print(_("# powermax communications"), file=file)
        if self.d.qtplasmacpmx:
            print("PM_PORT = {}".format(self.d.qtplasmacpmx), file=file)
        else:
            print("#PM_PORT = /dev/ttyUSB0", file=file)
        print(_("# user buttons"), file=file)
        for ub in range(1, 21):
            if self.d.qtplasmac_bnames[ub-1]:
                print("BUTTON_{}_NAME = {}".format(ub ,self.d.qtplasmac_bnames[ub-1]), file=file)
                print("BUTTON_{}_CODE = {}".format(ub ,self.d.qtplasmac_bcodes[ub-1]), file=file)

    # linuxcnc doesn't like having home right on an end of travel,
    # so extend the travel limit by up to .001in or .01mm
    def find_limits(self, letter):
        def get(s): return self.d[letter + s]
        minlim = -abs(get("minlim"))
        maxlim = get("maxlim")
        home = get("homepos")
        if self.d.units == _PD._METRIC: extend = .01
        else: extend = .001
        minlim = min(minlim, home) - extend
        maxlim = max(maxlim, home) + extend
        return (minlim, maxlim)

# BOILER CODE
    def __getitem__(self, item):
        return getattr(self, item)
    def __setitem__(self, item, value):
        return setattr(self, item, value)
