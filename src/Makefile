ifeq ($(origin KERNELRELEASE), undefined)
MAKEFLAGS += --warn-undefined-variables
endif

EXTRAFLAGS :=
EXTRA_DEBUG ?=

# see http://www.cmcrossroads.com/ask-mr-make/6535-tracing-rule-execution-in-gnu-make
# to trace make execution of make in more detail:
# make VV=1
ifeq ("$(origin VV)", "command line")
    OLD_SHELL := $(SHELL)
    SHELL = $(warning Building $@$(if $<, (from $<))$(if $?, ($? newer)))$(OLD_SHELL)
endif

# Delete the default suffix rules
.SUFFIXES:
.PHONY: default userspace modules clean modclean depclean install python pythonclean cscope cscopeclean

# A "trivial build" is one which should not include dependency information
# either because it should be usable before dependency information can be
# generated or when it is invalid (clean, docclean) or when running as root
# when the user must guarantee in advance that everything is built
# (setuid, install)
ifeq ($(MAKECMDGOALS),)
TRIVIAL_BUILD=no
else
ifeq ($(filter-out docclean clean setuid install tags swish,$(MAKECMDGOALS)),)
TRIVIAL_BUILD=yes
else
TRIVIAL_BUILD=no
endif
endif


# Beautify output
# ---------------------------------------------------------------------------
#
# A simple variant is to prefix commands with $(Q) - that's useful
# for commands that shall be hidden in non-verbose mode.
#
#	$(Q)ln $@ :<
#
# If BUILD_VERBOSE equals 0 then the above command will be hidden.
# If BUILD_VERBOSE equals 1 then the above command is displayed.

ifeq ("$(origin V)", "command line")
  BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
  BUILD_VERBOSE = 0
endif

ifeq ($(BUILD_VERBOSE),1)
  Q =
else
  Q = @
endif

ifeq "$(findstring s,$(filter-out --%, $(MAKEFLAGS)))" ""
ECHO=@echo
VECHO=echo
else
ECHO=@true
VECHO=true
endif

ifeq ($(BASEPWD),)
BASEPWD := $(shell pwd)
export BASEPWD
include Makefile.inc
ifeq ($(origin PYTHONPATH),undefined)
PYTHONPATH:=$(EMC2_HOME)/lib/python
else
PYTHONPATH:=$(EMC2_HOME)/lib/python:$(PYTHONPATH)
endif
export PYTHONPATH
else
include $(BASEPWD)/Makefile.inc
endif
ifeq ($(RTPREFIX),)
$(error Makefile.inc must specify RTPREFIX and other variables)
endif

cc-option = $(shell if $(CC) $(CFLAGS) $(1) -S -o /dev/null -xc /dev/null \
	     > /dev/null 2>&1; then echo "$(1)"; else echo "$(2)"; fi ;)

ifeq ($(origin KERNELRELEASE),undefined)
# When KERNELRELEASE is not defined, this is the userspace build.
# The "modules" target is the gateway to the kernel module build.
default: userspace modules
ifeq ($(RUN_IN_PLACE),yes)
ifneq ($(BUILD_SYS),uspace)
	@if [ -f ../bin/linuxcnc_module_helper ]; then if ! [ `stat -c %u ../bin/linuxcnc_module_helper` -eq 0 -a -u ../bin/linuxcnc_module_helper ]; then $(VECHO) "You now need to run 'sudo make setuid' in order to run in place."; fi; fi
else
	@if [ -f ../bin/rtapi_app ]; then if ! [ `stat -c %u ../bin/rtapi_app` -eq 0 -a -u ../bin/rtapi_app ]; then $(VECHO) "You now need to run 'sudo make setuid' in order to run in place with access to hardware."; fi; fi
endif
endif


# Print 'entering' all the time
MAKEFLAGS += w

# Create the variables with := so that subsequent += alterations keep it
# as a "substitute at assignment time" variable
TARGETS :=
PYTARGETS :=
GENERATED_MANPAGES :=

# Submakefiles from each of these directories will be included if they exist
SUBDIRS := \
    libnml/linklist libnml/cms libnml/rcs libnml/inifile libnml/os_intf \
    libnml/nml libnml/buffer libnml/posemath libnml \
    \
    rtapi/examples/timer rtapi/examples/semaphore rtapi/examples/shmem \
    rtapi/examples/extint rtapi/examples/fifo rtapi/examples rtapi \
    \
    hal/components hal/drivers hal/user_comps/devices hal/user_comps/mb2hal \
    hal/user_comps hal/user_comps/vismach hal/user_comps/vfs11_vfd hal/classicladder hal/utils hal \
    hal/user_comps/vfdb_vfd hal/user_comps/wj200_vfd \
    hal/user_comps/huanyang-vfd \
    \
    emc/usr_intf/axis emc/usr_intf/touchy emc/usr_intf/stepconf emc/usr_intf/pncconf \
    emc/usr_intf/gremlin \ emc/usr_intf/gscreen \
    emc/usr_intf/gmoccapy \
    emc/usr_intf emc/nml_intf emc/task emc/iotask emc/kinematics emc/tp emc/canterp \
    emc/motion emc/ini emc/rs274ngc emc/sai emc emc/pythonplugin \
    emc/motion-logger \
    \
    module_helper \
    \
    po \
    \
    ../docs/src ../docs/src/source-highlight \



ULAPISRCS := rtapi/$(RTPREFIX)_ulapi.c

# Each item in INCLUDES is transformed into a -I directive later on
# The top directory is always included
INCLUDES := .

USERSRCS :=
PROGRAMS :=

# When used like $(call TOxxx, ...) these turn a list of source files
# into the corresponding list of object files, dependency files,
# or both.  When a source file has to be compiled with special flags,
# TOOBJSDEPS is used.  Confusingly, TOOBJSDEPS includes preprocessed source
# file names, but this is what allows 'make src.i' to produce proper
# preprocessed source when src.c needs a customized compile flag.
# See Submakefile.skel for an example.
TOOBJS = $(patsubst %.cc,objects/%.o,$(patsubst %.c,objects/%.o,$(1)))
TODEPS = $(patsubst %.cc,objects/%.d,$(patsubst %.c,objects/%.d,$(1)))
TOOBJSDEPS = $(call TOOBJS,$(1)) $(call TODEPS, $(1)) $(patsubst %.cc,%.ii,$(patsubst %.c,%.i,$(1)))

SUBMAKEFILES := $(patsubst %,%/Submakefile,$(SUBDIRS))
-include $(wildcard $(SUBMAKEFILES))

# This checks that all the things listed in USERSRCS are either C files
# or C++ files
ASSERT_EMPTY = $(if $(1), $(error "Should be empty but is not: $(1)"))
$(call ASSERT_EMPTY,$(filter-out %.c %.cc, $(USERSRCS)))

ifeq ($(BUILD_PYTHON),yes)
$(call TOOBJS,$(PYSRCS)) : EXTRAFLAGS += -fPIC -fno-strict-aliasing
USERSRCS += $(PYSRCS)
endif

# Find the list of object files for each type of source file
CUSERSRCS := $(filter %.c,$(USERSRCS))
CXXUSERSRCS := $(filter %.cc,$(USERSRCS))
CUSEROBJS := $(call TOOBJS,$(CUSERSRCS))
CXXUSEROBJS += $(call TOOBJS,$(CXXUSERSRCS))

ifeq ($(TRIVIAL_BUILD),no)
# Find the dependency filenames, then include them all
DEPS := $(sort $(patsubst %.o,%.d,$(CUSEROBJS) $(CXXUSEROBJS)))
READ_DEPS = $(wildcard $(DEPS))
$(shell $(VECHO) 1>&2 Reading $(words $(READ_DEPS))/$(words $(DEPS)) dependency files)
-include $(READ_DEPS)
UNREAD_DEPS = $(filter-out $(READ_DEPS), $(DEPS))
$(shell $(VECHO) 1>&2 Done reading dependencies)
endif

# Each directory in $(INCLUDES) is passed as a -I directory when compiling.
INCLUDE := $(patsubst %,-I%, $(INCLUDES)) -I$(RTDIR)/include
ifeq ($(BUILD_PYTHON),yes)
INCLUDE += -I$(INCLUDEPY)
endif

# Compilation options.	Perhaps some of these should come from Makefile.inc? (CXXFLAGS now does)
INTEGER_OVERFLOW_FLAGS := -fwrapv
OPT := -Os $(INTEGER_OVERFLOW_FLAGS)
DEBUG := $(DEBUG) -g -Wall
CFLAGS := $(INCLUDE) $(OPT) $(DEBUG) $(EXTRA_DEBUG) -DULAPI -std=gnu99 -fgnu89-inline
CXXFLAGS := $(INCLUDE) $(CXXFLAGS)   $(EXTRA_DEBUG) -DULAPI $(DEBUG) $(OPT)

ifeq ($(RUN_IN_PLACE),yes)
LDFLAGS := -L$(LIB_DIR) -Wl,-rpath,$(LIB_DIR)
else
LDFLAGS := -Wl,-rpath-link,../lib
endif

# Rules to make .o (object) files
$(sort $(CUSEROBJS)) : objects/%.o: %.c
	$(ECHO) Compiling $<
	@mkdir -p $(dir $@)
	@rm -f $@
	$(Q)$(CC) -c $(CFLAGS) $(EXTRAFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@

$(sort $(CXXUSEROBJS)) : objects/%.o: %.cc
	$(ECHO) Compiling $<
	@mkdir -p $(dir $@)
	@rm -f $@
	$(Q)$(CXX) -c $(CXXFLAGS) $(EXTRAFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@

# Rules to make .i (preprocessed) files
$(sort $(patsubst %.c,%.i,$(CUSERSRCS))): %.i: %.c
	$(ECHO) Preprocessing $< to $@
	$(Q)$(CC) -dD $(CFLAGS) $(EXTRAFLAGS) -E $< -o $@

$(sort $(patsubst %.cc,%.ii,$(CXXUSERSRCS))): %.ii: %.cc
	$(ECHO) Preprocessing $< to $@
	$(Q)$(CXX) -dD $(CXXFLAGS) $(EXTRAFLAGS) -E $< -o $@

ifeq ($(TRIVIAL_BUILD),no)
configure: configure.in
	AUTOGEN_TARGET=configure ./autogen.sh

config.h.in: configure.in
	AUTOGEN_TARGET=config.h.in ./autogen.sh

config.status: configure
	if [ -f config.status ]; then ./config.status --recheck; else \
	    echo 1>&2 "*** linuxcnc is not configured.	Run './configure' with appropriate flags."; \
	    exit 1; \
	fi
endif

Makefile: config.h
config.h: config.h.in config.status
	@./config.status -q --header=$@

INFILES = \
        ../scripts/linuxcnc ../scripts/realtime \
	../scripts/haltcl ../scripts/rtapi.conf Makefile.inc Makefile.modinc \
	../tcl/linuxcnc.tcl ../scripts/halrun ../scripts/rip-environment \
	../scripts/linuxcncmkdesktop \
	 ../lib/python/nf.py \
	../share/desktop-directories/cnc.directory ../share/menus/CNC.menu \
	../share/applications/linuxcnc.desktop \
	../share/applications/linuxcnc-latency.desktop \
	../scripts/linuxcnc_var \

$(INFILES): %: %.in config.status
	@./config.status --file=$@

default: $(INFILES)

# For each file to be copied to ../include, its location in the source tree
# is listed here.  Note that due to $(INCLUDE), defined above, the include
# files in the source tree are the ones used when building linuxcnc.  The copy
# in ../include is used when building external components of linuxcnc.
HEADERS := \
    config.h \
    emc/ini/emcIniFile.hh \
    emc/ini/iniaxis.hh \
    emc/ini/initool.hh \
    emc/ini/initraj.hh \
    emc/ini/inihal.hh \
    emc/kinematics/cubic.h \
    emc/kinematics/kinematics.h \
    emc/kinematics/genhexkins.h \
    emc/kinematics/genserkins.h \
    emc/kinematics/pumakins.h \
    emc/tp/tc.h \
    emc/tp/tc_types.h \
    emc/tp/tcq.h \
    emc/tp/tp.h \
    emc/tp/tp_types.h \
    emc/tp/spherical_arc.h \
    emc/tp/blendmath.h \
    emc/motion/emcmotcfg.h \
    emc/motion/emcmotglb.h \
    emc/motion/motion.h \
    emc/motion/usrmotintf.h \
    emc/nml_intf/canon.hh \
    emc/nml_intf/canon_position.hh \
    emc/nml_intf/emctool.h \
    emc/nml_intf/emc.hh \
    emc/nml_intf/emc_nml.hh \
    emc/nml_intf/emccfg.h \
    emc/nml_intf/emcglb.h \
    emc/nml_intf/emcpos.h \
    emc/nml_intf/interp_return.hh \
    emc/nml_intf/interpl.hh \
    emc/nml_intf/motion_types.h \
    emc/nml_intf/debugflags.h \
    emc/rs274ngc/interp_internal.hh \
    emc/rs274ngc/interp_base.hh \
    emc/rs274ngc/rs274ngc.hh \
    hal/hal.h \
    hal/hal_parport.h \
    libnml/buffer/locmem.hh \
    libnml/buffer/memsem.hh \
    libnml/buffer/phantom.hh \
    libnml/buffer/physmem.hh \
    libnml/buffer/recvn.h \
    libnml/buffer/rem_msg.hh \
    libnml/buffer/sendn.h \
    libnml/buffer/shmem.hh \
    libnml/buffer/tcpmem.hh \
    libnml/cms/cms.hh \
    libnml/cms/cms_aup.hh \
    libnml/cms/cms_cfg.hh \
    libnml/cms/cms_dup.hh \
    libnml/cms/cms_srv.hh \
    libnml/cms/cms_up.hh \
    libnml/cms/cms_user.hh \
    libnml/cms/cms_xup.hh \
    libnml/cms/cmsdiag.hh \
    libnml/cms/tcp_opts.hh \
    libnml/cms/tcp_srv.hh \
    libnml/inifile/inifile.h \
    libnml/inifile/inifile.hh \
    libnml/linklist/linklist.hh \
    libnml/nml/cmd_msg.hh \
    libnml/nml/nml.hh \
    libnml/nml/nml_mod.hh \
    libnml/nml/nml_oi.hh \
    libnml/nml/nml_srv.hh \
    libnml/nml/nml_type.hh \
    libnml/nml/nmldiag.hh \
    libnml/nml/nmlmsg.hh \
    libnml/nml/stat_msg.hh \
    libnml/os_intf/_sem.h \
    libnml/os_intf/sem.hh \
    libnml/os_intf/_shm.h \
    libnml/os_intf/shm.hh \
    libnml/os_intf/_timer.h \
    libnml/os_intf/timer.hh \
    libnml/posemath/posemath.h \
    libnml/posemath/gotypes.h \
    libnml/posemath/gomath.h \
    libnml/posemath/sincos.h \
    libnml/rcs/rcs.hh \
    libnml/rcs/rcs_exit.hh \
    libnml/rcs/rcs_print.hh \
    libnml/rcs/rcsversion.h \
    rtapi/rtapi.h \
    rtapi/rtapi_app.h \
    rtapi/rtapi_atomic.h \
    rtapi/rtapi_bitops.h \
    rtapi/rtapi_bool.h \
    rtapi/rtapi_byteorder.h \
    rtapi/rtapi_device.h \
    rtapi/rtapi_firmware.h \
    rtapi/rtapi_gfp.h \
    rtapi/rtapi_io.h \
    rtapi/rtapi_limits.h \
    rtapi/rtapi_list.h \
    rtapi/rtapi_math.h \
    rtapi/rtapi_math_i386.h \
    rtapi/rtapi_math64.h \
    rtapi/rtapi_parport.h \
    rtapi/rtapi_pci.h \
    rtapi/rtapi_slab.h \
    rtapi/rtapi_stdint.h \
    rtapi/rtapi_common.h \
    rtapi/rtapi_ctype.h \
    rtapi/rtapi_errno.h \
    rtapi/rtapi_string.h

# Make each include file a target
TARGETS += $(patsubst %,../include/%,$(foreach h,$(HEADERS),$(notdir $h)))

# Add converting of %.po files
TARGETS += $(patsubst po/%.po, ../share/locale/%/LC_MESSAGES/linuxcnc.mo, $(wildcard po/*.po))
TARGETS += $(patsubst po/gmoccapy/%.po, ../share/locale/%/LC_MESSAGES/gmoccapy.mo, $(wildcard po/gmoccapy/*.po))
TARGETS += $(patsubst po/%.po, objects/%.msg, $(wildcard po/*.po))

# tooledit as standalone app:
TARGETS += ../bin/tooledit
../bin/tooledit: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/tooledit.tcl")>>$@
	$(Q)(echo "standalone_tooledit")>>$@
	$(Q)chmod +x $@

# ngcgui as standalone app:
TARGETS += ../bin/ngcgui
../bin/ngcgui: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/ngcgui.tcl")>>$@
	$(Q)(echo "::ngcgui::standalone_ngcgui")>>$@
	$(Q)chmod +x $@

# pyngcgui as standalone app:
TARGETS += ../bin/pyngcgui
../bin/pyngcgui: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(PYTHON)")>|$@
	$(Q)(echo "import pyngcgui")>>$@
	$(Q)(echo "pyngcgui.standalone_pyngcgui()")>>$@
	$(Q)chmod +x $@

# gremlin_view as standalone app:
TARGETS += ../bin/gremlin_view
../bin/gremlin_view: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(PYTHON)")>|$@
	$(Q)(echo "import gremlin_view")>>$@
	$(Q)(echo "gremlin_view.standalone_gremlin_view()")>>$@
	$(Q)chmod +x $@

# halshow as standalone app:
TARGETS += ../bin/halshow
../bin/halshow: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/bin/halshow.tcl")>>$@
	$(Q)chmod +x $@

# And make userspace depend on $(TARGETS)
userspace: $(TARGETS)

ifeq ($(BUILD_PYTHON),yes)
pythonclean:
	rm -f $(PYTARGETS)
	find ../lib/python -name '*.so' -exec rm {} +
python: $(PYTARGETS)
userspace: python
clean: docclean pythonclean cscopeclean
endif

# This is the gateway into the crazy world of "kbuild", the linux 2.6 system
# for building kernel modules.	Other kernel module build styles need to be
# accomodated here.
ifeq ($(BUILD_SYS),kbuild)

modules:
	MAKEFLAGS="$(filter-out --warn-undefined-variables,$(MAKEFLAGS))" \
	$(PYTHON) modsilent.py $(MAKE) KBUILD_EXTRA_SYMBOLS=$(moduledir)/Module.symvers -C $(KERNELDIR) SUBDIRS=`pwd` CC=$(CC) V=0 modules
	-cp Module.symvers *$(MODULE_EXT) ../rtlib/
endif

# These rules clean things up.	'modclean' cleans files generated by 'modules'
# (except that it doesn't remove the modules that were copied to rtlib)
# 'clean' cleans everything but dependency files, and 'depclean' cleans them
# too.
modclean:
	find -name '.*.cmd' -or -name '*.ko' -or -name '*.mod.c' -or -name '*.mod.o' | xargs rm -f
	-rm -rf .tmp_versions
	find . -name .tmp_versions |xargs rm -rf
	-rm -f ../rtlib/*.ko
	-rm -f ../rtlib/*.so

depclean:
	-rm -rf depends

clean: depclean modclean
	find . -name '*.o' |xargs rm -f
	-rm -rf objects
	-rm -f $(TARGETS)
	-rm -f $(GENERATED_MANPAGES)
	-rm -f ../rtlib/*.$(MODULE_EXT)
	-rm -f hal/components/conv_*.comp

# So that nothing is built as root, this rule does not depend on the touched
# files (Note that files in depends/ might be rebuilt, and there's little that
# can be done about it)
setuid:
ifeq ($(BUILD_SYS),uspace)
	chown root ../bin/rtapi_app
	chmod 4750 ../bin/rtapi_app
else
	chown root ../bin/linuxcnc_module_helper
	chmod 4750 ../bin/linuxcnc_module_helper
	chown root ../bin/pci_write
	chmod 4750 ../bin/pci_write
	chown root ../bin/pci_read
	chmod 4750 ../bin/pci_read
endif

# These rules allows a header file from this directory to be installed into
# ../include.  A pair of rules like these will exist in the Submakefile
# of each file that contains headers.
$(patsubst %,../include/%,$(wildcard *.h)): ../include/%.h: %.h
	$(Q)-cp $^ $@
$(patsubst %,../include/%,$(wildcard *.hh)): ../include/%.hh: %.hh
	$(Q)-cp $^ $@

DIR=install -d -m 0755 -o root
FILE=install -m 0644 -o root
TREE=cp -dR
CONFIGFILE=install -m 0644
EXE=install -m 0755 -o root
SETUID=install -m 4755 -o root
GLOB=$(wildcard $(1))

ifeq ($(RUN_IN_PLACE),yes)
define ERROR_MESSAGE
You configured run-in-place, but are trying to install.
For an installable version, run configure without --enable-run-in-place
and rebuild
endef
install:
	$(error $(ERROR_MESSAGE))

MENUS = ../share/menus/CNC.menu \
    ../share/desktop-directories/cnc.directory \
    ../share/applications/linuxcnc.desktop \
    ../share/applications/linuxcnc-latency.desktop \

install-menus install-menu: $(MENUS)
	mkdir -p $(HOME)/.config/menus/applications-merged
	cp $< $(HOME)/.config/menus/applications-merged
else

DOCS=NEWS README AUTHORS

DOCS_HELP=$(call GLOB,../docs/help/*)
NC_FILES=$(filter-out %/butterfly.ngc,$(call GLOB,../nc_files/*))
TCL=$(call GLOB,../tcl/*.tcl)
TCL_BIN=$(call GLOB,../tcl/bin/*.tcl) ../tcl/bin/popimage

install-test:
	@if type -path dpkg-query > /dev/null 2>&1 ; then  \
		if dpkg-query -S $(DESTDIR)/usr/bin/linuxcnc > /dev/null 2>&1 || dpkg-query -S $(DESTDIR)/usr/bin/emc > /dev/null 2>&1 ; then \
			echo "*** Error: Package version installed in $(DESTDIR)/usr"; \
			echo "Use './configure --enable-run-in-place' or uninstall the linuxcnc package"; \
			echo "before installing."; \
			exit 1; \
		fi \
	fi

install: install-test install-kernel-dep install-kernel-indep
	$(ECHO) "Installed in $(DESTDIR) with prefix $(prefix)"

install-dirs:
	$(DIR) $(DESTDIR)$(initd_dir) $(DESTDIR)$(EMC2_RTLIB_DIR) \
		$(DESTDIR)$(sysconfdir)/linuxcnc $(DESTDIR)$(bindir) \
		$(DESTDIR)$(libdir) $(DESTDIR)$(includedir)/linuxcnc \
		$(DESTDIR)$(docsdir) $(DESTDIR)$(ncfilesdir) \
		$(DESTDIR)/etc/X11/app-defaults $(DESTDIR)$(tcldir)/bin \
		$(DESTDIR)$(tcldir)/scripts \
		$(DESTDIR)$(mandir)/man1 \
		$(DESTDIR)$(mandir)/man3 \
		$(DESTDIR)$(mandir)/man9 \
		$(DESTDIR)$(tcldir)/msgs \
		$(DESTDIR)$(localedir)/de/LC_MESSAGES \
		$(DESTDIR)$(datadir)/axis/images \
		$(DESTDIR)$(datadir)/axis/tcl \
		$(DESTDIR)$(datadir)/gscreen/images \
		$(DESTDIR)$(datadir)/gscreen/skins \
		$(DESTDIR)$(datadir)/gmoccapy/images \
		$(DESTDIR)$(datadir)/glade3/catalogs \
		$(DESTDIR)$(datadir)/glade3/pixmaps \
		$(DESTDIR)$(datadir)/gtksourceview-2.0/language-specs \
		$(DESTDIR)$(datadir)/linuxcnc/stepconf \
		$(DESTDIR)$(datadir)/linuxcnc/pncconf \
		$(DESTDIR)$(datadir)/linuxcnc/pncconf/pncconf-help \
		$(DESTDIR)$(datadir)/linuxcnc/hallib

install-kernel-indep: install-dirs
	$(FILE) ../docs/man/man1/*.1 $(DESTDIR)$(mandir)/man1
	$(FILE) $(filter-out %/skeleton.3hal, $(wildcard ../docs/man/man3/*.3hal)) $(DESTDIR)$(mandir)/man3
	$(FILE) $(filter-out %/skeleton.3rtapi, $(wildcard ../docs/man/man3/*.3rtapi)) $(DESTDIR)$(mandir)/man3
	$(FILE) $(filter-out %/skeleton.9, $(wildcard ../docs/man/man9/*.9)) $(DESTDIR)$(mandir)/man9
	$(FILE) objects/*.msg $(DESTDIR)$(tcldir)/msgs
	$(EXE) ../scripts/realtime $(DESTDIR)$(initd_dir)
	$(EXE) ../scripts/halrun $(DESTDIR)$(bindir)
	$(EXE) ../scripts/haltcl $(DESTDIR)$(bindir)
	$(EXE) ../scripts/simulate_probe $(DESTDIR)$(bindir)
	$(EXE) ../scripts/sim_pin $(DESTDIR)$(bindir)
	$(FILE) ../docs/UPDATING $(DESTDIR)$(docsdir)/UPDATING
	$(FILE) ../*.png ../*.gif $(DESTDIR)$(datadir)/linuxcnc
	$(FILE) ../lib/hallib/*   $(DESTDIR)$(datadir)/linuxcnc/hallib

	# install all the sample configs, including subdirs (tar is required on debian systems, and common on others)
	$(DIR) $(DESTDIR)$(sampleconfsdir)
	((cd ../configs && tar --exclude CVS --exclude .cvsignore --exclude .gitignore -cf - .) | (cd $(DESTDIR)$(sampleconfsdir) && tar -xf -))

	$(EXE) $(filter-out ../bin/rtapi_app ../bin/linuxcnc_module_helper ../bin/pci_write ../bin/pci_read ../bin/test_rtapi_vsnprintf, $(filter ../bin/%,$(TARGETS))) $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc_info $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc_var $(DESTDIR)$(bindir)
	$(EXE) ../scripts/monitor-xhc-hb04 $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-test $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-plot $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-histogram $(DESTDIR)$(bindir)
	$(EXE) ../scripts/moveoff_gui $(DESTDIR)$(bindir)
	$(EXE) ../scripts/hal-histogram $(DESTDIR)$(bindir)
	$(EXE) ../scripts/xhc-hb04-accels $(DESTDIR)$(bindir)
	$(EXE) ../scripts/pyvcp_demo $(DESTDIR)$(bindir)
	$(EXE) ../scripts/gladevcp_demo $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcncmkdesktop $(DESTDIR)$(bindir)
	$(FILE) $(filter ../lib/%.a ../lib/%.so.0,$(TARGETS)) $(DESTDIR)$(libdir)
	cp --no-dereference $(filter ../lib/%.so, $(TARGETS)) $(DESTDIR)$(libdir)
	-ldconfig $(DESTDIR)$(libdir)
	$(FILE) $(filter %.h %.hh,$(TARGETS)) $(DESTDIR)$(includedir)/linuxcnc/
	$(FILE) $(addprefix ../docs/,$(DOCS)) $(DESTDIR)$(docsdir)
	$(FILE) $(DOCS_HELP) $(DESTDIR)$(docsdir)
	$(TREE) $(NC_FILES) $(DESTDIR)$(ncfilesdir)
	$(EXE) ../nc_files/M101 $(DESTDIR)$(ncfilesdir)
	$(FILE) ../tcl/TkLinuxCNC $(DESTDIR)/etc/X11/app-defaults
	$(FILE) ../app-defaults/XEmc $(DESTDIR)/etc/X11/app-defaults
	$(FILE) Makefile.modinc $(DESTDIR)$(datadir)/linuxcnc
	$(EXE) $(TCL) $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/hal.so $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/linuxcnc.so $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/pkgIndex.tcl $(DESTDIR)$(tcldir)
	$(EXE) $(TCL_BIN) $(DESTDIR)$(tcldir)/bin
	$(FILE) ../tcl/scripts/balloon.tcl ../tcl/scripts/emchelp.tcl $(DESTDIR)$(tcldir)/scripts
	$(EXE) ../tcl/scripts/Set_Coordinates.tcl $(DESTDIR)$(tcldir)/scripts
	$(FILE) ../share/linuxcnc/*.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/stepconf/*.glade $(DESTDIR)$(prefix)/share/linuxcnc/stepconf
	$(FILE) ../share/linuxcnc/touchy.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/gscreen.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/gscreen2.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/gmoccapy/gmoccapy.glade $(DESTDIR)$(prefix)/share/gmoccapy
	$(FILE) ../share/linuxcnc/pncconf/*.glade $(DESTDIR)$(prefix)/share/linuxcnc/pncconf
	$(FILE) ../share/linuxcnc/gremlin_view.ui $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/popupkeyboard.ui $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../configs/common/linuxcnc.nml $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../src/emc/usr_intf/pncconf/pncconf-help/*.txt $(DESTDIR)$(prefix)/share/linuxcnc/pncconf/pncconf-help
	$(FILE) ../src/emc/usr_intf/pncconf/pncconf-help/*.png $(DESTDIR)$(prefix)/share/linuxcnc/pncconf/pncconf-help

	$(FILE) ../lib/python/gladevcp/hal_python.xml $(DESTDIR)$(datadir)/glade3/catalogs/
	$(FILE) ../lib/python/gladevcp/widget*.png  $(DESTDIR)$(datadir)/glade3/pixmaps/

	$(FILE) ../share/gtksourceview-2.0/language-specs/*.lang  $(DESTDIR)$(datadir)/gtksourceview-2.0/language-specs/

ifeq ($(BUILD_PYTHON),yes)
install-kernel-indep: install-python
install-python: install-dirs
	$(DIR) $(DESTDIR)$(SITEPY) $(DESTDIR)$(SITEPY)/rs274
	$(DIR) $(DESTDIR)$(SITEPY)/touchy
	$(DIR) $(DESTDIR)$(SITEPY)/gscreen
	$(DIR) $(DESTDIR)$(SITEPY)/gmoccapy
	$(DIR) $(DESTDIR)$(SITEPY)/gladevcp
	$(DIR) $(DESTDIR)$(SITEPY)/stepconf
	$(DIR) $(DESTDIR)$(SITEPY)/pncconf
	$(FILE) ../lib/python/*.py ../lib/python/*.so $(DESTDIR)$(SITEPY)
	$(FILE) ../lib/python/rs274/*.py $(DESTDIR)$(SITEPY)/rs274
	$(FILE) ../lib/python/touchy/*.py $(DESTDIR)$(SITEPY)/touchy
	$(FILE) ../lib/python/gscreen/*.py $(DESTDIR)$(SITEPY)/gscreen
	$(FILE) ../lib/python/gmoccapy/*.py $(DESTDIR)$(SITEPY)/gmoccapy
	$(FILE) ../lib/python/gladevcp/*.py $(DESTDIR)$(SITEPY)/gladevcp
	$(FILE) ../lib/python/gladevcp/*.glade $(DESTDIR)$(SITEPY)/gladevcp
	$(FILE) ../lib/python/stepconf/*.py $(DESTDIR)$(SITEPY)/stepconf
	$(FILE) ../lib/python/pncconf/*.py $(DESTDIR)$(SITEPY)/pncconf
	$(EXE) ../bin/stepconf $(DESTDIR)$(bindir)
	$(EXE) ../bin/pncconf $(DESTDIR)$(bindir)
	$(EXE) ../bin/hal_input $(DESTDIR)$(bindir)
	$(EXE) ../bin/mitsub_vfd $(DESTDIR)$(bindir)
	$(EXE) ../bin/pyvcp $(DESTDIR)$(bindir)
	$(EXE) ../bin/gladevcp $(DESTDIR)$(bindir)
	$(EXE) ../bin/axis $(DESTDIR)$(bindir)
	$(EXE) ../bin/axis-remote $(DESTDIR)$(bindir)
	$(EXE) ../bin/debuglevel $(DESTDIR)$(bindir)
	$(EXE) ../bin/linuxcnctop $(DESTDIR)$(bindir)
	$(EXE) ../bin/mdi $(DESTDIR)$(bindir)
	$(EXE) ../bin/hal_manualtoolchange $(DESTDIR)$(bindir)
	$(EXE) ../bin/image-to-gcode $(DESTDIR)$(bindir)
	$(EXE) ../bin/touchy $(DESTDIR)$(bindir)
	$(EXE) ../bin/gscreen $(DESTDIR)$(bindir)
	$(EXE) ../bin/gmoccapy $(DESTDIR)$(bindir)
	$(EXE) $(patsubst %.py,../bin/%,$(VISMACH_PY)) $(DESTDIR)$(bindir)
	$(FILE) ../share/linuxcnc/linuxcnc-wizard.gif $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) emc/usr_intf/axis/etc/axis_light_background $(DESTDIR)$(docsdir)
	$(FILE) emc/usr_intf/axis/README $(DESTDIR)$(docsdir)/README.axis
	$(FILE) ../share/axis/images/*.png ../share/axis/images/*.gif ../share/axis/images/*.xbm ../share/axis/images/*.ngc $(DESTDIR)$(datadir)/axis/images
	$(FILE) ../share/axis/tcl/*.tcl $(DESTDIR)$(datadir)/axis/tcl
	$(FILE) ../share/gscreen/images/*.gif $(DESTDIR)$(datadir)/gscreen/images
	$(TREE) ../share/gscreen/skins/* $(DESTDIR)$(datadir)/gscreen/skins
	$(FILE) ../share/gmoccapy/images/*.png ../share/gmoccapy/images/*.gif ../share/gmoccapy/images/*.svg $(DESTDIR)$(datadir)/gmoccapy/images
endif

install-kernel-dep:
	$(DIR)	$(DESTDIR)$(moduledir)/linuxcnc \
		$(DESTDIR)$(bindir) \
		$(DESTDIR)$(sysconfdir)/linuxcnc
	$(FILE) ../rtlib/*$(MODULE_EXT) $(DESTDIR)$(EMC2_RTLIB_DIR)
ifneq "$(BUILD_SYS)" "uspace"
	$(FILE) Module.symvers $(DESTDIR)$(EMC2_RTLIB_DIR)
	$(SETUID) ../bin/linuxcnc_module_helper $(DESTDIR)$(bindir)
	$(SETUID) ../bin/pci_write $(DESTDIR)$(bindir)
	$(SETUID) ../bin/pci_read $(DESTDIR)$(bindir)
else
	$(SETUID) ../bin/rtapi_app $(DESTDIR)$(bindir)
endif
	$(FILE) ../scripts/rtapi.conf $(DESTDIR)$(sysconfdir)/linuxcnc
endif # RUN_IN_PLACE

CONF=../configs
COMMON=$(CONF)/common
CONFILES=$(addsuffix /$(1), $(filter-out $(COMMON) $(CONF),\
                                         ${shell find ${CONF} -type d -print}))

endif # userspace

ifneq ($(KERNELRELEASE),)
include $(BASEPWD)/hal/components/Submakefile
endif

# KERNELRELEASE is nonempty, therefore we are building modules using the
# "kbuild" system.  $(BASEPWD) is used here, instead of relative paths, because
# that's what kbuild seems to require

EXTRA_CFLAGS := $(filter-out -ffast-math,$(RTFLAGS)) -D__MODULE__ -I$(BASEPWD) -I$(BASEPWD)/libnml/linklist \
	-I$(BASEPWD)/libnml/cms -I$(BASEPWD)/libnml/rcs -I$(BASEPWD)/libnml/inifile \
	-I$(BASEPWD)/libnml/os_intf -I$(BASEPWD)/libnml/nml -I$(BASEPWD)/libnml/buffer \
	-I$(BASEPWD)/libnml/posemath -I$(BASEPWD)/rtapi -I$(BASEPWD)/hal \
	-I$(BASEPWD)/emc/nml_intf -I$(BASEPWD)/emc/kinematics -I$(BASEPWD)/emc/tp -I$(BASEPWD)/emc/motion \
	-DSEQUENTIAL_SUPPORT -DHAL_SUPPORT -DDYNAMIC_PLCSIZE -DRT_SUPPORT -DOLD_TIMERS_MONOS_SUPPORT -DMODBUS_IO_MASTER \
	-fno-fast-math $(call cc-option,-mieee-fp) -fno-unsafe-math-optimizations \
	-Wframe-larger-than=2560 -Wno-declaration-after-statement \
	$(INTEGER_OVERFLOW_FLAGS)
ifneq ($(KERNELRELEASE),)
ifeq ($(RTARCH):$(RTAI):$(filter $(RTFLAGS),-msse),x86_64:3:)
EXTRA_CFLAGS += -msse
endif
endif

ifeq "$(USE_STUBS)" "1"
MATHSTUB := rtapi/mathstubs.o
endif

ifdef SEQUENTIAL_SUPPORT
EXTRA_CFLAGS += -DSEQUENTIAL_SUPPORT
endif

# For each module, there's an addition to obj-m or obj-$(CONFIG_foo)
# plus a definition of foo-objs, which contains the full path to the
# object file(s) that the module contains.  Unfortunately, this setup pollutes
# the source directory with object files and other temporaries, but I can't
# find a way around it.

# Subdirectory:  rtapi
ifneq ($(BUILD_SYS),uspace)
obj-$(CONFIG_RTAPI) += rtapi.o
rtapi-objs := rtapi/$(RTPREFIX)_rtapi.o
endif

# Subdirectory: rtapi/examples (unneeded?)

# Subdirectory: hal/components
obj-$(CONFIG_BOSS_PLC) += boss_plc.o
boss_plc-objs := hal/components/boss_plc.o $(MATHSTUB)
obj-$(CONFIG_DEBOUNCE) += debounce.o
debounce-objs := hal/components/debounce.o $(MATHSTUB)
obj-$(CONFIG_ENCODER) += encoder.o
encoder-objs := hal/components/encoder.o $(MATHSTUB)
obj-$(CONFIG_COUNTER) += counter.o
counter-objs := hal/components/counter.o $(MATHSTUB)
obj-$(CONFIG_ENCODER_RATIO) += encoder_ratio.o
encoder_ratio-objs := hal/components/encoder_ratio.o $(MATHSTUB)
obj-$(CONFIG_STEPGEN) += stepgen.o
stepgen-objs := hal/components/stepgen.o $(MATHSTUB)
obj-$(CONFIG_LCD) += lcd.o
lcd-objs := hal/components/lcd.o $(MATHSTUB)
obj-$(CONFIG_MATRIX_KB) += matrix_kb.o
matrix_kb-objs := hal/components/matrix_kb.o $(MATHSTUB)
obj-$(CONFIG_MUX_GENERIC) += mux_generic.o
mux_generic-objs := hal/components/mux_generic.o $(MATHSTUB)
obj-$(CONFIG_PWMGEN) += pwmgen.o
pwmgen-objs := hal/components/pwmgen.o $(MATHSTUB)
obj-$(CONFIG_SIGGEN) += siggen.o
siggen-objs := hal/components/siggen.o $(MATHSTUB)
obj-$(CONFIG_PID) += pid.o
pid-objs := hal/components/pid.o $(MATHSTUB)
obj-$(CONFIG_AT_PID) += at_pid.o
at_pid-objs := hal/components/at_pid.o $(MATHSTUB)
obj-$(CONFIG_PID) += threads.o
threads-objs := hal/components/threads.o $(MATHSTUB)
obj-$(CONFIG_SUPPLY) += supply.o
supply-objs := hal/components/supply.o $(MATHSTUB)
obj-$(CONFIG_SIM_ENCODER) += sim_encoder.o
sim_encoder-objs := hal/components/sim_encoder.o $(MATHSTUB)
obj-$(CONFIG_WEIGHTED_SUM) += weighted_sum.o
weighted_sum-objs := hal/components/weighted_sum.o $(MATHSTUB)
obj-$(CONFIG_WATCHDOG) += watchdog.o
watchdog-objs := hal/components/watchdog.o $(MATHSTUB)
obj-$(CONFIG_MODMATH) += modmath.o
modmath-objs := hal/components/modmath.o $(MATHSTUB)
obj-$(CONFIG_STREAMER) += streamer.o
streamer-objs := hal/components/streamer.o $(MATHSTUB)
obj-$(CONFIG_SAMPLER) += sampler.o
sampler-objs := hal/components/sampler.o $(MATHSTUB)

# Subdirectory: hal/drivers
obj-$(CONFIG_HAL_PARPORT) += hal_parport.o
hal_parport-objs := hal/drivers/hal_parport.o $(MATHSTUB)
ifneq ($(BUILD_SYS),uspace)
obj-$(CONFIG_PCI_8255) += pci_8255.o
pci_8255-objs := hal/drivers/pci_8255.o
obj-$(CONFIG_HAL_TIRO) += hal_tiro.o
hal_tiro-objs := hal/drivers/hal_tiro.o $(MATHSTUB)
obj-$(CONFIG_HAL_STG) += hal_stg.o
hal_stg-objs := hal/drivers/hal_stg.o $(MATHSTUB)
obj-$(CONFIG_HAL_VTI) += hal_vti.o
hal_vti-objs := hal/drivers/hal_vti.o $(MATHSTUB)
obj-$(CONFIG_HAL_EVOREG) += hal_evoreg.o
hal_evoreg-objs := hal/drivers/hal_evoreg.o $(MATHSTUB)
obj-$(CONFIG_HAL_MOTENC) += hal_motenc.o
hal_motenc-objs := hal/drivers/hal_motenc.o $(MATHSTUB)
obj-$(CONFIG_HAL_AX521H) += hal_ax5214h.o
hal_ax5214h-objs := hal/drivers/hal_ax5214h.o $(MATHSTUB)
obj-$(CONFIG_HAL_SPEAKER) += hal_speaker.o
hal_speaker-objs := hal/drivers/hal_speaker.o $(MATHSTUB)
obj-$(CONFIG_HAL_SKELETON) += hal_skeleton.o
hal_skeleton-objs := hal/drivers/hal_skeleton.o $(MATHSTUB)
obj-$(CONFIG_OPTO_AC5) += opto_ac5.o
opto_ac5-objs := hal/drivers/opto_ac5.o $(MATHSTUB)
endif
obj-$(CONFIG_HAL_GM) += hal_gm.o
hal_gm-objs := hal/drivers/hal_gm.o $(MATHSTUB)
obj-$(CONFIG_HAL_PPMC) += hal_ppmc.o
hal_ppmc-objs := hal/drivers/hal_ppmc.o $(MATHSTUB)

obj-$(CONFIG_HOSTMOT2) += hostmot2.o hm2_test.o hm2_pci.o hm2_7i43.o hm2_7i90.o setsserial.o
ifeq ($(BUILD_SYS),uspace)
obj-$(CONFIG_HOSTMOT2) += hm2_eth.o hm2_spi.o
endif
hostmot2-objs :=			  \
    hal/drivers/mesa-hostmot2/hostmot2.o  \
    hal/drivers/mesa-hostmot2/ioport.o	  \
    hal/drivers/mesa-hostmot2/encoder.o   \
    hal/drivers/mesa-hostmot2/abs_encoder.o\
	hal/drivers/mesa-hostmot2/resolver.o  \
    hal/drivers/mesa-hostmot2/pwmgen.o	  \
    hal/drivers/mesa-hostmot2/tp_pwmgen.o \
	hal/drivers/mesa-hostmot2/sserial.o   \
    hal/drivers/mesa-hostmot2/stepgen.o   \
	hal/drivers/mesa-hostmot2/bspi.o	  \
	hal/drivers/mesa-hostmot2/uart.o	  \
    hal/drivers/mesa-hostmot2/watchdog.o  \
    hal/drivers/mesa-hostmot2/pins.o	  \
    hal/drivers/mesa-hostmot2/dpll.o  \
    hal/drivers/mesa-hostmot2/led.o	  \
    hal/drivers/mesa-hostmot2/tram.o	  \
    hal/drivers/mesa-hostmot2/raw.o	  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_7i90-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_7i90.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_7i43-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_7i43.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_pci-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_pci.o   \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_eth-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_eth.o   \
    $(MATHSTUB)
hm2_spi-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_spi.o   \
    $(MATHSTUB)
hm2_test-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_test.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
setsserial-objs :=			  \
    hal/drivers/mesa-hostmot2/setsserial.o  \
    $(MATHSTUB)

obj-$(CONFIG_CLASSICLADDER_RT) += classicladder_rt.o
classicladder_rt-objs := hal/classicladder/module_hal.o $(MATHSTUB)
classicladder_rt-objs += hal/classicladder/arithm_eval.o
classicladder_rt-objs += hal/classicladder/arrays.o
classicladder_rt-objs += hal/classicladder/calc.o
classicladder_rt-objs += hal/classicladder/calc_sequential.o
classicladder_rt-objs += hal/classicladder/manager.o
classicladder_rt-objs += hal/classicladder/symbols.o
classicladder_rt-objs += hal/classicladder/vars_access.o

ifdef SEQUENTIAL_SUPPORT
classicladder_rt-objs += hal/classicladder/calc_sequential_rt.o
endif

obj-m += scope_rt.o
scope_rt-objs := hal/utils/scope_rt.o $(MATHSTUB)

obj-m += hal_lib.o
hal_lib-objs := hal/hal_lib.o $(MATHSTUB)

obj-m += trivkins.o
trivkins-objs := emc/kinematics/trivkins.o

obj-m += 5axiskins.o
5axiskins-objs := emc/kinematics/5axiskins.o

obj-m += maxkins.o
maxkins-objs := emc/kinematics/maxkins.o

obj-m += gantrykins.o
gantrykins-objs := emc/kinematics/gantrykins.o

obj-m += rotatekins.o
rotatekins-objs := emc/kinematics/rotatekins.o

obj-m += tripodkins.o
tripodkins-objs := emc/kinematics/tripodkins.o

obj-m += genhexkins.o
genhexkins-objs := emc/kinematics/genhexkins.o
genhexkins-objs += libnml/posemath/_posemath.o
genhexkins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-m += genserkins.o
genserkins-objs := emc/kinematics/genserkins.o
genserkins-objs += libnml/posemath/gomath.o
genserkins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-m += pumakins.o
pumakins-objs := emc/kinematics/pumakins.o
pumakins-objs += libnml/posemath/_posemath.o
pumakins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-m += scarakins.o
scarakins-objs := emc/kinematics/scarakins.o
scarakins-objs += libnml/posemath/_posemath.o
scarakins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-$(CONFIG_MOTMOD) += motmod.o
motmod-objs := emc/kinematics/cubic.o
motmod-objs += emc/tp/tc.o
motmod-objs += emc/tp/tcq.o
motmod-objs += emc/tp/tp.o
motmod-objs += emc/tp/spherical_arc.o
motmod-objs += emc/tp/blendmath.o
motmod-objs += emc/tp/math_util.o
motmod-objs += emc/tp/joint_util.o
motmod-objs += emc/motion/motion.o
motmod-objs += emc/motion/command.o
motmod-objs += emc/motion/control.o
motmod-objs += emc/motion/homing.o
motmod-objs += emc/motion/emcmotglb.o
motmod-objs += emc/motion/emcmotutil.o
motmod-objs += emc/motion/stashf.o
motmod-objs += emc/motion/dbuf.o
motmod-objs += emc/nml_intf/emcpose.o
motmod-objs += libnml/posemath/_posemath.o
motmod-objs += libnml/posemath/sincos.o $(MATHSTUB)

TORTOBJS = $(foreach file,$($(patsubst %.o,%,$(1))-objs), objects/rt$(file))
ifeq ($(BUILD_SYS),uspace)
EXTRA_CFLAGS += -fPIC -Os
RTOBJS := $(sort $(foreach mod,$(obj-m),$(call TORTOBJS,$(mod))))

RTDEPS := $(sort $(RTOBJS:.o=.d))

modules: $(patsubst %.o,../rtlib/%.so,$(obj-m))
../rtlib/%.so:
	$(ECHO) Linking $@
	$(Q)ld -d -r -o objects/$*.tmp $^
	$(Q)objcopy -j .rtapi_export -O binary objects/$*.tmp objects/$*.sym
	$(Q)(echo '{ global : '; tr -s '\0' < objects/$*.sym | xargs -r0 printf '%s;\n' | grep .; echo 'local : * ; };') > objects/$*.ver
	$(Q)$(CC) -shared -Bsymbolic $(LDFLAGS) -Wl,--version-script,objects/$*.ver -o $@ $^ -lm

RTFLAGS += -fno-strict-aliasing -fwrapv

# Rules to make .o (object) files
$(sort $(RTOBJS)) : objects/rt%.o : %.c
	$(ECHO) Compiling realtime $<
	@rm -f $@
	@mkdir -p $(dir $@)
	$(Q)$(CC) -c $(OPT) $(DEBUG) $(EXTRA_DEBUG) -DRTAPI $(EXTRA_CFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@
endif

ifeq ($(BUILD_SYS),normal)
modules: $(patsubst %,../rtlib/%,$(obj-m))
RTOBJS := $(sort $(foreach mod,$(obj-m),$(call TORTOBJS,$(mod))))
RTDEPS := $(sort $(patsubst objects/%.o,depends/%.d, $(RTOBJS)))

# Rules to make .o (object) files
$(sort $(RTOBJS)) : objects/rt%.o : %.c
	$(ECHO) Compiling realtime $<
	@rm -f $@
	@mkdir -p $(dir $@)
	$(Q)$(CC) -c -DRTAPI -nostdinc -isystem $(shell $(CC) -print-file-name=include) -I$(KERNELDIR)/include $(EXTRA_CFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@

../rtlib/%.o:
	$(ECHO) Linking $@
	$(Q)ld -r -static -S $(LDFLAGS) -o $@ $^ $(EXTRALINK) $(MATHLIB)
endif

ifneq "$(filter normal uspace,$(BUILD_SYS))" ""
ifneq "$(BUILD_SYS)" "uspace"
../rtlib/rtapi$(MODULE_EXT): $(addprefix objects/rt,$(rtapi-objs))
endif
../rtlib/classicladder_rt$(MODULE_EXT): $(addprefix objects/rt,$(classicladder_rt-objs))
../rtlib/boss_plc$(MODULE_EXT): $(addprefix objects/rt,$(boss_plc-objs))
../rtlib/debounce$(MODULE_EXT): $(addprefix objects/rt,$(debounce-objs))
../rtlib/encoder$(MODULE_EXT): $(addprefix objects/rt,$(encoder-objs))
../rtlib/counter$(MODULE_EXT): $(addprefix objects/rt,$(counter-objs))
../rtlib/encoder_ratio$(MODULE_EXT): $(addprefix objects/rt,$(encoder_ratio-objs))
../rtlib/stepgen$(MODULE_EXT): $(addprefix objects/rt,$(stepgen-objs))
../rtlib/lcd$(MODULE_EXT): $(addprefix objects/rt,$(lcd-objs))
../rtlib/matrix_kb$(MODULE_EXT): $(addprefix objects/rt,$(matrix_kb-objs))
../rtlib/mux_generic$(MODULE_EXT): $(addprefix objects/rt,$(mux_generic-objs))
../rtlib/pwmgen$(MODULE_EXT): $(addprefix objects/rt,$(pwmgen-objs))
../rtlib/siggen$(MODULE_EXT): $(addprefix objects/rt,$(siggen-objs))
../rtlib/at_pid$(MODULE_EXT): $(addprefix objects/rt,$(at_pid-objs))
../rtlib/pid$(MODULE_EXT): $(addprefix objects/rt,$(pid-objs))
../rtlib/threads$(MODULE_EXT): $(addprefix objects/rt,$(threads-objs))
../rtlib/supply$(MODULE_EXT): $(addprefix objects/rt,$(supply-objs))
../rtlib/sim_encoder$(MODULE_EXT): $(addprefix objects/rt,$(sim_encoder-objs))
../rtlib/weighted_sum$(MODULE_EXT): $(addprefix objects/rt,$(weighted_sum-objs))
../rtlib/watchdog$(MODULE_EXT): $(addprefix objects/rt,$(watchdog-objs))
../rtlib/modmath$(MODULE_EXT): $(addprefix objects/rt,$(modmath-objs))
../rtlib/streamer$(MODULE_EXT): $(addprefix objects/rt,$(streamer-objs))
../rtlib/sampler$(MODULE_EXT): $(addprefix objects/rt,$(sampler-objs))
../rtlib/hal_parport$(MODULE_EXT): $(addprefix objects/rt,$(hal_parport-objs))
#../rtlib/uparport$(MODULE_EXT): $(addprefix objects/rt,$(uparport-objs))
../rtlib/pci_8255$(MODULE_EXT): $(addprefix objects/rt,$(pci_8255-objs))
../rtlib/hal_tiro$(MODULE_EXT): $(addprefix objects/rt,$(hal_tiro-objs))
../rtlib/hal_stg$(MODULE_EXT): $(addprefix objects/rt,$(hal_stg-objs))
../rtlib/hal_vti$(MODULE_EXT): $(addprefix objects/rt,$(hal_vti-objs))
../rtlib/hal_evoreg$(MODULE_EXT): $(addprefix objects/rt,$(hal_evoreg-objs))
../rtlib/hal_motenc$(MODULE_EXT): $(addprefix objects/rt,$(hal_motenc-objs))
../rtlib/hal_ax5214h$(MODULE_EXT): $(addprefix objects/rt,$(hal_ax5214h-objs))
../rtlib/hal_ppmc$(MODULE_EXT): $(addprefix objects/rt,$(hal_ppmc-objs))
../rtlib/hal_skeleton$(MODULE_EXT): $(addprefix objects/rt,$(hal_skeleton-objs))
../rtlib/hal_speaker$(MODULE_EXT): $(addprefix objects/rt,$(hal_speaker-objs))
../rtlib/opto_ac5$(MODULE_EXT): $(addprefix objects/rt,$(opto_ac5-objs))
../rtlib/scope_rt$(MODULE_EXT): $(addprefix objects/rt,$(scope_rt-objs))
../rtlib/hal_lib$(MODULE_EXT): $(addprefix objects/rt,$(hal_lib-objs))
../rtlib/motmod$(MODULE_EXT): $(addprefix objects/rt,$(motmod-objs))
../rtlib/trivkins$(MODULE_EXT): $(addprefix objects/rt,$(trivkins-objs))
../rtlib/5axiskins$(MODULE_EXT): $(addprefix objects/rt,$(5axiskins-objs))
../rtlib/maxkins$(MODULE_EXT): $(addprefix objects/rt,$(maxkins-objs))
../rtlib/gantrykins$(MODULE_EXT): $(addprefix objects/rt,$(gantrykins-objs))
../rtlib/rotatekins$(MODULE_EXT): $(addprefix objects/rt,$(rotatekins-objs))
../rtlib/tripodkins$(MODULE_EXT): $(addprefix objects/rt,$(tripodkins-objs))
../rtlib/genhexkins$(MODULE_EXT): $(addprefix objects/rt,$(genhexkins-objs))
../rtlib/genserkins$(MODULE_EXT): $(addprefix objects/rt,$(genserkins-objs))
../rtlib/pumakins$(MODULE_EXT): $(addprefix objects/rt,$(pumakins-objs))
../rtlib/scarakins$(MODULE_EXT): $(addprefix objects/rt,$(scarakins-objs))
../rtlib/hal_gm$(MODULE_EXT): $(addprefix objects/rt,$(hal_gm-objs))
../rtlib/hostmot2$(MODULE_EXT): $(addprefix objects/rt,$(hostmot2-objs))
../rtlib/hm2_test$(MODULE_EXT): $(addprefix objects/rt,$(hm2_test-objs))
../rtlib/hm2_pci$(MODULE_EXT): $(addprefix objects/rt,$(hm2_pci-objs))
../rtlib/hm2_7i43$(MODULE_EXT): $(addprefix objects/rt,$(hm2_7i43-objs))
../rtlib/hm2_7i90$(MODULE_EXT): $(addprefix objects/rt,$(hm2_7i90-objs))
../rtlib/setsserial$(MODULE_EXT): $(addprefix objects/rt,$(setsserial-objs))
../rtlib/hal_parport$(MODULE_EXT): $(addprefix objects/rt,$(hal_parport-objs))
../rtlib/hal_ppmc$(MODULE_EXT): $(addprefix objects/rt,$(hal_ppmc-objs))
../rtlib/hm2_eth$(MODULE_EXT): $(addprefix objects/rt,$(hm2_eth-objs))
../rtlib/hm2_spi$(MODULE_EXT): $(addprefix objects/rt,$(hm2_spi-objs))

ifeq ($(TRIVIAL_BUILD),no)
READ_RTDEPS = $(wildcard $(RTDEPS))
$(shell $(VECHO) 1>&2 Reading $(words $(READ_RTDEPS))/$(words $(RTDEPS)) realtime dependency files)
-include $(READ_RTDEPS)
$(shell $(VECHO) 1>&2 Done reading realtime dependencies)
endif
endif

# Phony so that it is always rebuilt when requested, not because it
# shouldn't exist as a file
.PHONY: tags
tags:
	ctags-exuberant \
		--extra=+fq \
		--exclude=depends --exclude=objects --exclude=.mod.c \
		'--langmap=make:+(Submakefile),make:+(Makefile.inc),c:+.comp' \
		-I EXPORT_SYMBOL+,RTAPI_MP_INT+,RTAPI_MP_LONG+,RTAPI_MP_STRING+ \
		-I RTAPI_MP_ARRAY_INT+,RTAPI_MP_ARRAY_LONG+,RTAPI_MP_ARRAY_STRING+ \
		-I MODULE_AUTHOR+,MODULE_DESCRIPTION+,MODULE_LICENSE+ \
		-R . ../tcl ../scripts ../share/axis/tcl
	rm -f TAGS
	find . -type f -name '*.[ch]' -printf '%P\0' |xargs -0 etags -l c --append
	find . -type f -name '*.cc' -printf '%P\0' |xargs -0 etags -l c++ --append
	find . -type f -name '*.hh' -printf '%P\0' |xargs -0 etags -l c++ --append

etags:
	etags	--extra=+fq \
		--exclude=depends --exclude=objects --exclude=.mod.c \
		'--langmap=make:+(Submakefile),make:+(Makefile.inc),c:+.comp' \
		-I EXPORT_SYMBOL+,RTAPI_MP_INT+,RTAPI_MP_LONG+,RTAPI_MP_STRING+ \
		-I RTAPI_MP_ARRAY_INT+,RTAPI_MP_ARRAY_LONG+,RTAPI_MP_ARRAY_STRING+ \
		-I MODULE_AUTHOR+,MODULE_DESCRIPTION+,MODULE_LICENSE+ \
		-R . ../tcl ../scripts ../share/axis/tcl
	find . -type f -name '*.[ch]' |xargs etags --language-force=C --append
	find . -type f -name '*.cc' |xargs etags --language-force=C++ --append
	find . -type f -name '*.hh' |xargs etags --language-force=C++ --append

.PHONY: swish
swish:
	swish-e -c .swish_config -v 0 -i $(BASEPWD) \
		$(dir $(BASEPWD))tcl \
		$(dir $(BASEPWD))share/axis/tcl \
		$(dir $(BASEPWD))lib/python \
		$(dir $(BASEPWD))scripts \
		$(dir $(BASEPWD))configs \
		$(dir $(BASEPWD))docs/src \
		$(dir $(BASEPWD))docs/man/man1

# When you depend on objects/var-ZZZ you are depending on the contents of the
# variable ZZZ, which is assumed to depend on a Makefile, a Submakefile, or
# Makefile.inc
objects/var-%: Makefile $(wildcard $(SUBMAKEFILES)) Makefile.inc
	@mkdir -p $(dir $@)
	@echo $($*) > $@.tmp
	@sh move-if-change $@.tmp $@

../lib/%.so: ../lib/%.so.0
	ln -sf $(notdir $<) $@

cscope:
	cscope -Rb

cscopeclean:
	bash -c 'for f in `find ./ -name "cscope.*out"`;do rm $$f;done'
# vim:ts=8:sts=8:sw=8:noet:

