# stdglue - canned prolog and epilog functions for the remappable builtin codes (T,M6,M61,S,F)
#
# we dont use argspec to avoid the generic error message of the argspec prolog and give more
# concise ones here

# cycle_prolog,cycle_epilog: generic code-independent support glue for oword sub cycles
#
# these are provided as starting point - for more concise error message you would better
# write a prolog specific for the code
#
# Usage:
#REMAP=G84.3  modalgroup=1 argspec=xyzqp prolog=cycle_prolog ngc=g843 epilog=cycle_epilog

import emccanon
from interpreter import *
from emccanon import MESSAGE
throw_exceptions = 1

# REMAP=S   prolog=setspeed_prolog  ngc=setspeed epilog=setspeed_epilog
# exposed parameter: #<speed>

def setspeed_prolog(self,**words):
    try:
        c = self.blocks[self.remap_level]
        if not c.s_flag:
            self.set_errormsg("S requires a value") 
            return INTERP_ERROR
        self.params["speed"] = c.s_number
    except Exception as e:
        self.set_errormsg("S/setspeed_prolog: %s)" % (e))
        return INTERP_ERROR
    return INTERP_OK

def setspeed_epilog(self,**words):
    try:
        if not self.value_returned:
            r = self.blocks[self.remap_level].executing_remap
            self.set_errormsg("the %s remap procedure %s did not return a value"
                             % (r.name,r.remap_ngc if r.remap_ngc else r.remap_py))
            return INTERP_ERROR
        if self.return_value < -TOLERANCE_EQUAL: # 'less than 0 within interp's precision'
            self.set_errormsg("S: remap procedure returned %f" % (self.return_value)) 
            return INTERP_ERROR
        if self.blocks[self.remap_level].builtin_used:
            pass
            #print "---------- S builtin recursion, nothing to do"
        else:
            self.speed = self.params["speed"]
            emccanon.enqueue_SET_SPINDLE_SPEED(self.speed)
        return INTERP_OK
    except Exception as e:
        self.set_errormsg("S/setspeed_epilog: %s)" % (e))
        return INTERP_ERROR
    return INTERP_OK    

# REMAP=F   prolog=setfeed_prolog  ngc=setfeed epilog=setfeed_epilog
# exposed parameter: #<feed>

def setfeed_prolog(self,**words):
    try:
        c = self.blocks[self.remap_level]
        if not c.f_flag:
            self.set_errormsg("F requires a value") 
            return INTERP_ERROR
        self.params["feed"] = c.f_number
    except Exception as e:
        self.set_errormsg("F/setfeed_prolog: %s)" % (e))
        return INTERP_ERROR
    return INTERP_OK    

def setfeed_epilog(self,**words):
    try:
        if not self.value_returned:
            r = self.blocks[self.remap_level].executing_remap
            self.set_errormsg("the %s remap procedure %s did not return a value"
                             % (r.name,r.remap_ngc if r.remap_ngc else r.remap_py))
            return INTERP_ERROR
        if self.blocks[self.remap_level].builtin_used:
            pass
            #print "---------- F builtin recursion, nothing to do"
        else:
            self.feed_rate = self.params["feed"]
            emccanon.enqueue_SET_FEED_RATE(self.feed_rate)
        return INTERP_OK
    except Exception as e:
        self.set_errormsg("F/setfeed_epilog: %s)" % (e))
        return INTERP_ERROR
    return INTERP_OK    

# REMAP=T   prolog=prepare_prolog ngc=prepare epilog=prepare_epilog
# exposed parameters: #<tool> #<pocket>

def prepare_prolog(self,**words):
    try:
        cblock = self.blocks[self.remap_level]
        if not cblock.t_flag:
            self.set_errormsg("T requires a tool number")
            return INTERP_ERROR
        tool  = cblock.t_number
        if tool:
            (status, pocket) = self.find_tool_pocket(tool)
            if status != INTERP_OK:
                self.set_errormsg("T%d: pocket not found" % (tool))
                return status
        else:
            pocket = -1 # this is a T0 - tool unload
        self.params["tool"] = tool
        self.params["pocket"] = pocket
        return INTERP_OK
    except Exception as e:
        self.set_errormsg("T%d/prepare_prolog: %s" % (int(words['t']), e))
        return INTERP_ERROR

def prepare_epilog(self, **words):
    try:
        if not self.value_returned:
            r = self.blocks[self.remap_level].executing_remap
            self.set_errormsg("the %s remap procedure %s did not return a value"
                             % (r.name,r.remap_ngc if r.remap_ngc else r.remap_py))
            return INTERP_ERROR
        if self.blocks[self.remap_level].builtin_used:
            #print "---------- T builtin recursion, nothing to do"
            return INTERP_OK
        else:
            if self.return_value > 0:
                self.selected_tool = int(self.params["tool"])
                self.selected_pocket = int(self.params["pocket"])
                emccanon.SELECT_TOOL(self.selected_tool)
                return INTERP_OK
            else:
                self.set_errormsg("T%d: aborted (return code %.1f)" % (int(self.params["tool"]),self.return_value))
                return INTERP_ERROR
    except Exception as e:
        self.set_errormsg("T%d/prepare_epilog: %s" % (tool,e))
        return INTERP_ERROR       

# REMAP=M6  modalgroup=6 prolog=change_prolog ngc=change epilog=change_epilog
# exposed parameters:
#    #<tool_in_spindle>
#    #<selected_tool>
#    #<current_pocket>
#    #<selected_pocket>

def change_prolog(self, **words):
    try:
        # this is relevant only when using iocontrol-v2.
        if self.params[5600] > 0.0:
            if self.params[5601] < 0.0:
                self.set_errormsg("Toolchanger hard fault %d" % (int(self.params[5601])))
                return INTERP_ERROR
            print("change_prolog: Toolchanger soft fault %d" % int(self.params[5601]))

        if self.selected_pocket < 0:
            self.set_errormsg("M6: no tool prepared")
            return INTERP_ERROR
        if self.cutter_comp_side:
            self.set_errormsg("Cannot change tools with cutter radius compensation on")
            return INTERP_ERROR
        self.params["tool_in_spindle"] = self.current_tool
        self.params["selected_tool"] = self.selected_tool
        self.params["current_pocket"] = self.current_pocket
        self.params["selected_pocket"] = self.selected_pocket
        return INTERP_OK
    except Exception as e:
        self.set_errormsg("M6/change_prolog: %s" % (e))
        return INTERP_ERROR

def change_epilog(self, **words):
    try:
        if not self.value_returned:
            r = self.blocks[self.remap_level].executing_remap
            self.set_errormsg("the %s remap procedure %s did not return a value"
                             % (r.name,r.remap_ngc if r.remap_ngc else r.remap_py))
            yield INTERP_ERROR
        # this is relevant only when using iocontrol-v2.
        if self.params[5600] > 0.0:
            if self.params[5601] < 0.0:
                self.set_errormsg("Toolchanger hard fault %d" % (int(self.params[5601])))
                yield INTERP_ERROR
            print("change_epilog: Toolchanger soft fault %d" % int(self.params[5601]))

        if self.blocks[self.remap_level].builtin_used:
            #print "---------- M6 builtin recursion, nothing to do"
            yield INTERP_OK
        else:
            if self.return_value > 0.0:
                # commit change
                self.selected_pocket =  int(self.params["selected_pocket"])
                emccanon.CHANGE_TOOL(self.selected_pocket)
                self.current_pocket = self.selected_pocket
                self.selected_pocket = -1
                self.selected_tool = -1
                # cause a sync()
                self.set_tool_parameters()
                self.toolchange_flag = True
                yield INTERP_EXECUTE_FINISH
            else:
                self.set_errormsg("M6 aborted (return code %.1f)" % (self.return_value))
                yield INTERP_ERROR
    except Exception as e:
        self.set_errormsg("M6/change_epilog: %s" % (e))
        yield INTERP_ERROR

# REMAP=M61  modalgroup=6 prolog=settool_prolog ngc=settool epilog=settool_epilog
# exposed parameters: #<tool> #<pocket>

def settool_prolog(self,**words):
    try:
        c = self.blocks[self.remap_level]
        if not c.q_flag:
            self.set_errormsg("M61 requires a Q parameter") 
            return INTERP_ERROR
        tool = int(c.q_number)
        if tool < -TOLERANCE_EQUAL: # 'less than 0 within interp's precision'
            self.set_errormsg("M61: Q value < 0") 
            return INTERP_ERROR
        (status,pocket) = self.find_tool_pocket(tool)
        if status != INTERP_OK:
            self.set_errormsg("M61 failed: requested tool %d not in table" % (tool))
            return status
        self.params["tool"] = tool
        self.params["pocket"] = pocket
        return INTERP_OK
    except Exception as e:
        self.set_errormsg("M61/settool_prolog: %s)" % (e))
        return INTERP_ERROR

def settool_epilog(self,**words):
    try:
        if not self.value_returned:
            r = self.blocks[self.remap_level].executing_remap
            self.set_errormsg("the %s remap procedure %s did not return a value"
                             % (r.name,r.remap_ngc if r.remap_ngc else r.remap_py))
            return INTERP_ERROR

        if self.blocks[self.remap_level].builtin_used:
            #print "---------- M61 builtin recursion, nothing to do"
            return INTERP_OK
        else:
            if self.return_value > 0.0:
                self.current_tool = int(self.params["tool"])
                self.current_pocket = int(self.params["pocket"])
                emccanon.CHANGE_TOOL_NUMBER(self.current_pocket)
                # cause a sync()
                self.tool_change_flag = True
                self.set_tool_parameters()
            else:
                self.set_errormsg("M61 aborted (return code %.1f)" % (self.return_value))
                return INTERP_ERROR
    except Exception as e:
        self.set_errormsg("M61/settool_epilog: %s)" % (e))
        return INTERP_ERROR

# educational alternative: M61 remapped to an all-Python handler
# demo - this really does the same thing as the builtin (non-remapped) M61
#
# REMAP=M61 modalgroup=6 python=set_tool_number

def set_tool_number(self, **words):
    try:
        c = self.blocks[self.remap_level]
        if c.q_flag:
            toolno = int(c.q_number)
        else:
            self.set_errormsg("M61 requires a Q parameter")
            return status 
        (status,pocket) = self.find_tool_pocket(toolno)
        if status != INTERP_OK:
            self.set_errormsg("M61 failed: requested tool %d not in table" % (toolno))
            return status
        if words['q'] > -TOLERANCE_EQUAL: # 'greater equal 0 within interp's precision'
            self.current_pocket = pocket
            self.current_tool = toolno
            emccanon.CHANGE_TOOL_NUMBER(pocket)
            # cause a sync()
            self.tool_change_flag = True
            self.set_tool_parameters()
            return INTERP_OK
        else:
            self.set_errormsg("M61 failed: Q=%4" % (toolno))
            return INTERP_ERROR
    except Exception as e:
        self.set_errormsg("M61/set_tool_number: %s" % (e))
        return INTERP_ERROR

_uvw = ("u","v","w","a","b","c")
_xyz = ("x","y","z","a","b","c")
# given a plane, return  sticky words, incompatible axis words and plane name
# sticky[0] is also the movement axis
_compat = {
    emccanon.CANON_PLANE_XY : (("z","r"),_uvw,"XY"),
    emccanon.CANON_PLANE_YZ : (("x","r"),_uvw,"YZ"),
    emccanon.CANON_PLANE_XZ : (("y","r"),_uvw,"XZ"),
    emccanon.CANON_PLANE_UV : (("w","r"),_xyz,"UV"),
    emccanon.CANON_PLANE_VW : (("u","r"),_xyz,"VW"),
    emccanon.CANON_PLANE_UW : (("v","r"),_xyz,"UW")}           

# extract and pass parameters from current block, merged with extra parameters on a continuation line
# keep tjose parameters across invocations
# export the parameters into the oword procedure
def cycle_prolog(self,**words):
    # self.sticky_params is assumed to have been initialized by the
    # init_stgdlue() method below
    global _compat
    try:    
        # determine whether this is the first or a subsequent call
        c = self.blocks[self.remap_level]
        r = c.executing_remap
        if c.g_modes[1] == r.motion_code:
            # first call - clear the sticky dict
            self.sticky_params[r.name] = dict()

        self.params["motion_code"] = c.g_modes[1]

        (sw,incompat,plane_name) =_compat[self.plane]
        for (word,value) in list(words.items()):
            # inject current parameters
            self.params[word] = value
            # record sticky words
            if word in sw:
                if self.debugmask & 0x00080000: print("%s: record sticky %s = %.4f" % (r.name,word,value))
                self.sticky_params[r.name][word] = value
            if word in incompat:
                return "%s: Cannot put a %s in a canned cycle in the %s plane" % (r.name, word.upper(), plane_name)

        # inject sticky parameters which were not in words:
        for (key,value) in list(self.sticky_params[r.name].items()):
            if not key in words:
                if self.debugmask & 0x00080000: print("%s: inject sticky %s = %.4f" % (r.name,key,value))
                self.params[key] = value

        if not "r" in self.sticky_params[r.name]:
            return "%s: cycle requires R word" % (r.name)
        else:
            if self.sticky_params[r.name] <= 0.0:
                return "%s: R word must be > 0 if used (%.4f)" % (r.name, words["r"])

        if "l" in words:
            # checked in interpreter during block parsing
            # if l <= 0 or l not near an int
            self.params["l"] = words["l"]
            
        if "p" in words:
            p = words["p"]
            if p < 0.0:
                return "%s: P word must be >= 0 if used (%.4f)" % (r.name, p)
            self.params["p"] = p

        if self.feed_rate == 0.0:
            return "%s: feed rate must be > 0" % (r.name)
        if self.feed_mode == INVERSE_TIME:
            return "%s: Cannot use inverse time feed with canned cycles" % (r.name)
        if self.cutter_comp_side:
            return "%s: Cannot use canned cycles with cutter compensation on" % (r.name)
        return INTERP_OK

    except Exception as e:
        raise
        return "cycle_prolog failed: %s" % (e)

# make sure the next line has the same motion code, unless overriden by a
# new G code
def cycle_epilog(self,**words):
    try:
        c = self.blocks[self.remap_level]
        self.motion_mode = c.executing_remap.motion_code # retain the current motion mode
        return INTERP_OK
    except Exception as e:
        return "cycle_epilog failed: %s" % (e)

# this should be called from TOPLEVEL __init__()
def init_stdglue(self):
    self.sticky_params = dict()

#####################################
# pure python remaps
#####################################

# REMAP=M6 python=ignore_m6
#
# m5 silently ignored
#
def ignore_m6(self,**words):
    try:
        return INTERP_OK
    except Exception as e:
        return "Ignore M6 failed: %s" % (e)

# REMAP=T python=index_lathe_tool_with_wear
#
# uses T101 for tool 1, wear 1 no M6 needed
# tool offsets for tool 1 and tool 10001 are added together.
#
def index_lathe_tool_with_wear(self,**words):
    # only run this if we are really moving the machine
    # skip this if running task for the screen
    if not self.task:
        return INTERP_OK
    try:
        # check there is a tool number from the Gcode
        cblock = self.blocks[self.remap_level]
        if not cblock.t_flag:
            self.set_errormsg("T requires a tool number")
            return INTERP_ERROR
        tool_raw = int(cblock.t_number)

        # interpet the raw tool number into tool and wear number
        # If it's less then 100 someone forgot to add the wear #, so we added it automatically
        # separate out tool number (tool) and wear number (wear), add 10000 to wear number
        if tool_raw <100:
            tool_raw=tool_raw*100
        tool = int(tool_raw/100)
        wear = 10000 + tool_raw % 100

        # uncomment for debugging
        #print'***tool#',cblock.t_number,'toolraw:',tool_raw,'tool split:',tool,'wear split',wear
        if tool:
            # check for tool number entry in tool file
            (status, pocket) = self.find_tool_pocket(tool)
            if status != INTERP_OK:
                self.set_errormsg("T%d: tool entry not found" % (tool))
                return status
        else:
            tool = -1
            pocket = -1
            wear = -1
        self.params["tool"] = tool
        self.params["pocket"] = pocket
        self.params["wear"] =  wear

        # index tool immediately to tool number
        self.selected_tool = int(self.params["tool"])
        self.selected_pocket = int(self.params["pocket"])
        emccanon.SELECT_TOOL(self.selected_tool)
        if self.selected_pocket < 0:
            self.set_errormsg("T0 not valid")
            return INTERP_ERROR
        if self.cutter_comp_side:
            self.set_errormsg("Cannot change tools with cutter radius compensation on")
            return INTERP_ERROR
        self.params["tool_in_spindle"] = self.current_tool
        self.params["selected_tool"] = self.selected_tool
        self.params["current_pocket"] = self.current_pocket
        self.params["selected_pocket"] = self.selected_pocket

        # change tool
        try:
            self.selected_pocket =  int(self.params["selected_pocket"])
            emccanon.CHANGE_TOOL(self.selected_pocket)
            self.current_pocket = self.selected_pocket
            self.selected_pocket = -1
            self.selected_tool = -1
            # cause a sync()
            self.set_tool_parameters()
            self.toolchange_flag = True
        except:
            self.set_errormsg("T change aborted (return code %.1f)" % (self.return_value))
            return INTERP_ERROR

        # add tool offset
        self.execute("g43 h%d"% tool)
        # if the wear offset is specified, add it's offset
        if wear>10000:
            self.execute("g43.2 h%d"% wear)
        return INTERP_OK

    except Exception as e:
        print(e)
        self.set_errormsg("T%d index_lathe_tool_with_wear: %s" % (int(words['t']), e))
        return INTERP_ERROR


# REMAP=M6 modalgroup=10 python=tool_probe_m6
#
# auto tool probe on m6
# move to Z max
# move to tool change position for toolchange
# backup offset and position
# wait for acknowledge of tool change
# move to tool setter probe position
# probe tool on tool setter
# move back to tool change position
# set tool offsets
# move to Z max
# goto position backup
# based on Versaprobe remap
#
# param _backup_offset holds the work piece height
#
# required INI settings
# (Abs coordinates/ machine based units)
#
# will follow these directives:
#{EMCIO]
#TOOL_CHANGE_POSITION = 0 0 0
#TOOL_CHANGE_WITH_SPINDLE_ON = 0
#TOOL_CHANGE_QUILL_UP = 1

# exemple with imperial unit
#[TOOLSENSOR]
#X = -5.00
#Y = -1
#Z = -1

#SETTER_HEIGHT = 1.6
#MAXPROBE =  -3
#TS_LATCH = 0.012
#REVERSE_LATCH = 0.08
#SEARCH_VEL = 12
#PROBE_VEL = 0.4


# What about checking machine is homed before allow this?
# TODO ver's use a configurable setterheight that can be set manually or using probe

def tool_probe_m6(self, **words):

    # only run this if we are really moving the machine
    # skip this if running task for the screen
    if not self.task:
        yield INTERP_OK

    IMPERIAL_BASED = not(bool(self.params['_metric_machine']))

    # Saving G90 G91 at startup
    if bool(self.params["_absolute"]) == 1:
        print ("Absolute G90")
        AbsoluteFlag = True
    else:
        print ("Incremental G91")
        AbsoluteFlag = False

    # cancel tool offset
    self.execute("G49")

    # record current position; probably should record every axis
    X = emccanon.GET_EXTERNAL_POSITION_X()
    Y = emccanon.GET_EXTERNAL_POSITION_Y()
#    Z = emccanon.GET_EXTERNAL_POSITION_Z()
    print ("X ",X)
    print ("Y ",Y)
#    print ("Z ",Z)
    if self.current_pocket == 0:                                      # if current tool is 0 do not restore Z
        Z = 0
        print ("Z if TOOL 0 ",Z)
    else:                                                             # if current tool is GT 0 try restore Z with all offset including new tool offset
        Z = self.params[5422]
        print ("Z if TOOL 1 ",Z)


    # turn off all spindles if required
    if not self.tool_change_with_spindle_on:
        for s in range(0,self.num_spindles):
            emccanon.STOP_SPINDLE_TURNING(s)

    try:
        # we need to be in machine based units
        # if we aren't - switch
        # remember so we can switch back later
        switchUnitsFlag = False
        if bool(self.params["_imperial"]) != IMPERIAL_BASED:
            print ("not right Units: {}".format(bool(self.params["_imperial"])))
            if IMPERIAL_BASED:
                print ("switched Units to imperial")
                self.execute("G20")
            else:
                print ("switched Units to metric")
                self.execute("G21")
            switchUnitsFlag = True

        # Force absolute for G53 move
        self.execute("G90")

        # Z up first if required
        if self.tool_change_quill_up:
            self.execute("G53 G0 Z0")

        self.params["tool_in_spindle"] = self.current_tool
        self.params["selected_tool"] = self.selected_tool
        self.params["current_pocket"] = self.current_pocket
        self.params["selected_pocket"] = self.selected_pocket

        # change tool where ever we are
        # user sets toolchange position prior to toolchange
        # we will return here after

        try:
            self.selected_pocket =  int(self.params["selected_pocket"])
            emccanon.CHANGE_TOOL(self.selected_pocket)
            self.current_pocket = self.selected_pocket
            self.selected_pocket = -1
            self.selected_tool = -1
            # cause a sync()
            self.set_tool_parameters()
            self.toolchange_flag = True
        except InterpreterException as e:
            # if we switched units for tool change - switch back
            if switchUnitsFlag:
                if IMPERIAL_BASED:
                    self.execute("G21")
                    print ("switched Units back to metric")
                else:
                    self.execute("G20")
                    print ("switched Units back to imperial")
            if AbsoluteFlag:
                 self.execute("G90")
            else:
                 self.execute("G91")
            self.set_errormsg("tool_probe_m6 remap error: %s" % (e))
            yield INTERP_ERROR

        yield INTERP_EXECUTE_FINISH

        try:

# TODO allow the autoprobe section to be a configurable option from some interface (restoring manual mode)

            # move to tool setter position (from INI)
            self.execute("G90")
            self.execute("G53 G0 X#<_ini[TOOLSENSOR]X> Y#<_ini[TOOLSENSOR]Y>")
            self.execute("G53 G0 Z#<_ini[TOOLSENSOR]Z>")

            # backup G5x offset for correct tool measurement
            if self.params["_coord_system"] == 540:
                self.params["_backup_offset"] = self.params[5223]    
            elif self.params["_coord_system"] == 550:
                self.params["_backup_offset"] = self.params[5243]
            elif self.params["_coord_system"] == 560:
                self.params["_backup_offset"] = self.params[5263]
            elif self.params["_coord_system"] == 570:
                self.params["_backup_offset"] = self.params[5283]
            elif self.params["_coord_system"] == 580:
                self.params["_backup_offset"] = self.params[5303]
            elif self.params["_coord_system"] == 590:
                self.params["_backup_offset"] = self.params[5323]
            elif self.params["_coord_system"] == 591:
                self.params["_backup_offset"] = self.params[5343]
            elif self.params["_coord_system"] == 592:
                self.params["_backup_offset"] = self.params[5363]
            elif self.params["_coord_system"] == 593:
                self.params["_backup_offset"] = self.params[5383]
            print ("backup_offset", self.params["_backup_offset"])

            # reset G5x offset for correct tool measurement
            self.execute("G10 L2 P0 Z0")

            # set incremental mode
            self.execute("G91")

            # Search probe
            self.execute("G38.2 Z#<_ini[TOOLSENSOR]MAXPROBE> F#<_ini[TOOLSENSOR]SEARCH_VEL>")
            # Wait for results
            yield INTERP_EXECUTE_FINISH

            # FIXME if there is an error it never comes back   # A possible solution is G38.3 and check if the var 5070 is 1 ???
            # which leaves linuxcnc in g91 state
            if self.params[5070] == 0 or self.return_value > 0.0:
                # if we switched units for tool change - switch back
                if switchUnitsFlag:
                    if IMPERIAL_BASED:
                        self.execute("G21")
                        print ("switched Units back to metric")
                    else:
                        self.execute("G20")
                        print ("switched Units back to imperial")
                if AbsoluteFlag:
                    self.execute("G90")
                else:
                    self.execute("G91")
                # restore G5x offset if something fail
                self.execute("G10 L2 P0 Z{}".format(self.params["_backup_offset"]))
                self.set_errormsg("tool_probe_m6 remap error:")
                yield INTERP_ERROR

            if self.params["_ini[TOOLSENSOR]SETTER_WITH_SPRING"] == 1: # DO NOT WORK FINE WITHOUT SPRING MOUNTED PROBE AND SETTER
                     print("------------G38.4 used WITH SPRING SETTER-------------")
                     # Spring mounted latch probe
                     self.execute("G38.4 Z#<_ini[TOOLSENSOR]REVERSE_LATCH> F[#<_ini[TOOLSENSOR]SEARCH_VEL>*0.5]")
                     # Wait for results
                     yield INTERP_EXECUTE_FINISH

                     # FIXME if there is an error it never comes back   # A possible solution is G38.5 and check if the var 5070 is 1 ???
                     # which leaves linuxcnc in g91 state
                     if self.params[5070] == 0 or self.return_value > 0.0:
                     # if we switched units for tool change - switch back
                         if switchUnitsFlag:
                             if IMPERIAL_BASED:
                                 self.execute("G21")
                                 print ("switched Units back to metric")
                             else:
                                 self.execute("G20")
                                 print ("switched Units back to imperial")
                         if AbsoluteFlag:
                              self.execute("G90")
                         else:
                              self.execute("G91")
                         # restore G5x offset if something fail
                         self.execute("G10 L2 P0 Z{}".format(self.params["_backup_offset"]))
                         self.set_errormsg("tool_probe_m6 remap error:")
                         yield INTERP_ERROR

            # Final Latch probe
            self.execute("G1 Z#<_ini[TOOLSENSOR]TS_LATCH> F[#<_ini[TOOLSENSOR]SEARCH_VEL>*0.5]")

            # Final probe
            self.execute("G38.2 Z-[#<_ini[TOOLSENSOR]TS_LATCH>*2] F#<_ini[TOOLSENSOR]PROBE_VEL>")
            # Wait for final results
            yield INTERP_EXECUTE_FINISH

            # FIXME if there is an error it never comes back   # A possible solution is G38.3 and check if the var 5070 is 1 ???
            # which leaves linuxcnc in g91 state
            if self.params[5070] == 0 or self.return_value > 0.0:
                # if we switched units for tool change - switch back
                if switchUnitsFlag:
                    if IMPERIAL_BASED:
                        self.execute("G21")
                        print ("switched Units back to metric")
                    else:
                        self.execute("G20")
                        print ("switched Units back to imperial")
                if AbsoluteFlag:
                    self.execute("G90")
                else:
                    self.execute("G91")
                # restore G5x offset if something fail
                self.execute("G10 L2 P0 Z{}".format(self.params["_backup_offset"]))
                self.set_errormsg("tool_probe_m6 remap error:")
                yield INTERP_ERROR

            # Force absolute for G53 move
            self.execute("G90")

            # adjust tool offset from calculations
            proberesult = self.params[5063]
            probeheight = self.params["_ini[TOOLSENSOR]SETTER_HEIGHT"]
            workheight = self.params["_backup_offset"]                  # load G5x backup_offset after correct tool measurement


            adj = proberesult - probeheight                             # IMO better to save tool offset without G5x offset and to do this separatly
            self.execute("G10 L1 P#<selected_tool> Z{}".format(adj))

            adj = workheight
            self.execute("G10 L2 P0 Z{}".format(adj))                    # restore G5x offset after correct tool measurement

            # apply tool offset
            self.execute("G43")
            # Wait for acceptance
            yield INTERP_EXECUTE_FINISH

            # Z up first if required
            if self.tool_change_quill_up:
                self.execute("G53 G0 Z0")

            # return to recorded positon
            self.execute("G53 G0 X{:.5f} Y{:.5f}".format(X,Y))
            
#            Zcalc = Z + self.params["_backup_offset"]
#            print("Zcalc", Zcalc)
#            self.execute("G53 G0 Z{:.5f}".format(Zcalc))

            # Attempt for wait to position before call self.execute("G1 Z{:.5f}".format(Z)) # Value are ok but this code seem to be not executed
            yield INTERP_EXECUTE_FINISH

            if Z != 0 and AbsoluteFlag:
                  print("******************RESTORING Z", Z)
                  self.execute("F100")
                  self.execute("G1 Z{:.5f}".format(Z))                   # Value are ok but this code seem to be not executed
                  print("******************Why not move to ", Z)
            elif AbsoluteFlag == 0:
                  print("******************NO Z RESTORING IF STARTUP MODE IS INCREMENTAL")
            elif Z ==0:
                  print("******************NO Z RESTORING IF LAST TOOL WAS 0")
            else:
                  print("******************NO Z RESTORING DUE TO SOMETHING WRONG")

            # set back the saved absolute state if needed
            if AbsoluteFlag:
                   self.execute("G90")
            else:
                   self.execute("G91")

                    
            # if we switched units for tool change - switch back
            if switchUnitsFlag:
                if IMPERIAL_BASED:
                    self.execute("G21")
                    print ("switched Units back to metric")
                else:
                    self.execute("G20")
                    print ("switched Units back to imperial")


        except InterpreterException as e:
            # if we switched units for tool change - switch back
            if switchUnitsFlag:
                if IMPERIAL_BASED:
                    self.execute("G21")
                    print ("switched Units back to metric")
                else:
                    self.execute("G20")
                    print ("switched Units back to imperial")
            if AbsoluteFlag:
                self.execute("G90")
            else:
                self.execute("G91")
            # restore G5x offset if something fail
            self.execute("G10 L2 P0 Z{}".format(self.params["_backup_offset"]))
            msg = "%d: '%s' - %s" % (e.line_number,e.line_text, e.error_message)
            print (msg)
            yield INTERP_ERROR

    except Exception as e:
        print (e)
        self.set_errormsg("tool_probe_m6 remap error: %s" % (e))
        yield INTERP_ERROR



