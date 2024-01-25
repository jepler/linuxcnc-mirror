/********************************************************************
* Description: initraj.cc
*   INI file initialization for trajectory level
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <stdio.h>		// NULL
#include <stdlib.h>		// atol()
#include <string.h>		// strlen()
#include <ctype.h>		// isspace()

#include "emc.hh"
#include "emcpos.h"             // EmcPose
#include "rcs_print.hh"
#include "posemath.h"		// PM_POSE, PM_RPY
#include "emcIniFile.hh"
#include "initraj.hh"		// these decls
#include "emcglb.h"		/*! \todo TRAVERSE_RATE (FIXME) */
#include "inihal.hh"

extern value_inihal_data old_inihal_data;

/*
  loadTraj()

  Loads ini file params for traj

  AXES <int>                    number of axes in system
  LINEAR_UNITS <float>          units per mm
  ANGULAR_UNITS <float>         units per degree
  CYCLE_TIME <float>            cycle time for traj calculations
  DEFAULT_VELOCITY <float>      default velocity
  MAX_VELOCITY <float>          max velocity
  MAX_ACCELERATION <float>      max acceleration
  DEFAULT_ACCELERATION <float>  default acceleration
  HOME <float> ...              world coords of home, in X Y Z R P W

  calls:

  emcTrajSetAxes(int axes);
  emcTrajSetUnits(double linearUnits, double angularUnits);
  emcTrajSetCycleTime(double cycleTime);
  emcTrajSetVelocity(double vel);
  emcTrajSetAcceleration(double acc);
  emcTrajSetMaxVelocity(double vel);
  emcTrajSetMaxAcceleration(double acc);
  emcTrajSetHome(EmcPose home);
  */

static int loadTraj(EmcIniFile *trajInifile)
{
    const char *inistring;
    EmcLinearUnits linearUnits;
    EmcAngularUnits angularUnits;
    double vel;
    double acc;
    unsigned char coordinateMark[6] = { 1, 1, 1, 0, 0, 0 };
    int t;
    int len;
    char homes[LINELEN];
    char home[LINELEN];
    EmcPose homePose = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double d;

    trajInifile->EnableExceptions(EmcIniFile::ERR_CONVERSION);

    try{
	int axes = 0;
	int axismask = 0;
	const char *coord = trajInifile->Find("COORDINATES", "TRAJ");
	if(coord) {
	    if(strchr(coord, 'x') || strchr(coord, 'X')) axismask |= 1;
	    if(strchr(coord, 'y') || strchr(coord, 'Y')) axismask |= 2;
	    if(strchr(coord, 'z') || strchr(coord, 'Z')) axismask |= 4;
	    if(strchr(coord, 'a') || strchr(coord, 'A')) axismask |= 8;
	    if(strchr(coord, 'b') || strchr(coord, 'B')) axismask |= 16;
	    if(strchr(coord, 'c') || strchr(coord, 'C')) axismask |= 32;
	    if(strchr(coord, 'u') || strchr(coord, 'U')) axismask |= 64;
	    if(strchr(coord, 'v') || strchr(coord, 'V')) axismask |= 128;
	    if(strchr(coord, 'w') || strchr(coord, 'W')) axismask |= 256;
	} else {
	    axismask = 1 | 2 | 4;		// default: XYZ machine
	}
	trajInifile->Find(&axes, "AXES", "TRAJ");

        if (0 != emcTrajSetAxes(axes, axismask)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetAxes\n");
            }
            return -1;
        }

        linearUnits = 0;
        trajInifile->FindLinearUnits(&linearUnits, "LINEAR_UNITS", "TRAJ");
        angularUnits = 0;
        trajInifile->FindAngularUnits(&angularUnits, "ANGULAR_UNITS", "TRAJ");

        if (0 != emcTrajSetUnits(linearUnits, angularUnits)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetUnits\n");
            }
            return -1;
        }

        vel = 1.0;
        trajInifile->Find(&vel, "DEFAULT_VELOCITY", "TRAJ");

        // set the corresponding global
        traj_default_velocity = vel;
        old_inihal_data.traj_default_velocity = vel;

        // and set dynamic value
        if (0 != emcTrajSetVelocity(0, vel)) { //default velocity on startup 0
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetVelocity\n");
            }
            return -1;
        }

        vel = 1e99; // by default, use AXIS limit
        trajInifile->Find(&vel, "MAX_VELOCITY", "TRAJ");

        // set the corresponding global
        traj_max_velocity = vel;
        old_inihal_data.traj_max_velocity = vel;

        // and set dynamic value
        if (0 != emcTrajSetMaxVelocity(vel)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetMaxVelocity\n");
            }
            return -1;
        }

        acc = 1e99; // let the axis values apply
        trajInifile->Find(&acc, "DEFAULT_ACCELERATION", "TRAJ");

        if (0 != emcTrajSetAcceleration(acc)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetAcceleration\n");
            }
            return -1;
        }
        old_inihal_data.traj_default_acceleration = acc;

        acc = 1e99; // let the axis values apply
        trajInifile->Find(&acc, "MAX_ACCELERATION", "TRAJ");

        if (0 != emcTrajSetMaxAcceleration(acc)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetMaxAcceleration\n");
            }
            return -1;
        }
        old_inihal_data.traj_max_acceleration = acc;

        int arcBlendEnable = 1;
        int arcBlendFallbackEnable = 0;
        int arcBlendOptDepth = 50;
        int arcBlendGapCycles = 4;
        double arcBlendRampFreq = 100.0;
        double arcBlendTangentKinkRatio = 0.1;

        trajInifile->Find(&arcBlendEnable, "ARC_BLEND_ENABLE", "TRAJ");
        trajInifile->Find(&arcBlendFallbackEnable, "ARC_BLEND_FALLBACK_ENABLE", "TRAJ");
        trajInifile->Find(&arcBlendOptDepth, "ARC_BLEND_OPTIMIZATION_DEPTH", "TRAJ");
        trajInifile->Find(&arcBlendGapCycles, "ARC_BLEND_GAP_CYCLES", "TRAJ");
        trajInifile->Find(&arcBlendRampFreq, "ARC_BLEND_RAMP_FREQ", "TRAJ");
        trajInifile->Find(&arcBlendTangentKinkRatio, "ARC_BLEND_KINK_RATIO", "TRAJ");

        if (0 != emcSetupArcBlends(arcBlendEnable, arcBlendFallbackEnable,
                    arcBlendOptDepth, arcBlendGapCycles, arcBlendRampFreq, arcBlendTangentKinkRatio)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcSetupArcBlends\n");
            }
            return -1;
        } 

        old_inihal_data.traj_arc_blend_enable = arcBlendEnable;
        old_inihal_data.traj_arc_blend_fallback_enable = arcBlendFallbackEnable;
        old_inihal_data.traj_arc_blend_optimization_depth = arcBlendOptDepth;
        old_inihal_data.traj_arc_blend_gap_cycles = arcBlendGapCycles;
        old_inihal_data.traj_arc_blend_ramp_freq = arcBlendRampFreq;
        old_inihal_data.traj_arc_blend_tangent_kink_ratio = arcBlendTangentKinkRatio;
        //TODO update inihal

        int extraConsistencyChecks = 0;
        double maxPositionDriftError = 0.0001;

        trajInifile->Find(&extraConsistencyChecks, "EXTRA_CONSISTENCY_CHECKS", "TRAJ");
        trajInifile->Find(&maxPositionDriftError, "MAX_POSITION_DRIFT_ERROR", "TRAJ");

        emcSetupConsistencyChecks(extraConsistencyChecks, maxPositionDriftError);

        double maxFeedScale = 1.0;
        trajInifile->Find(&maxFeedScale, "MAX_FEED_OVERRIDE", "DISPLAY");

        if (0 != emcSetMaxFeedOverride(maxFeedScale)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcSetMaxFeedOverride\n");
            }
            return -1;
        } 
    }

    catch(EmcIniFile::Exception &e){
        e.Print();
        return -1;
    }

    // set coordinateMark[] to hold 1's for each coordinate present,
    // so that home position can be interpreted properly
    if ((inistring = trajInifile->Find("COORDINATES", "TRAJ")) != NULL) {
        len = strlen(inistring);
        // there's an entry in ini file, so clear all the marks out first
        // so that defaults don't apply at all
        for (t = 0; t < 6; t++) {
            coordinateMark[t] = 0;
        }
        // now set actual marks
        for (t = 0; t < len; t++) {
            if (inistring[t] == 'X')
                coordinateMark[0] = 1;
            else if (inistring[t] == 'Y')
                coordinateMark[1] = 1;
            else if (inistring[t] == 'Z')
                coordinateMark[2] = 1;
            else if (inistring[t] == 'R')
                coordinateMark[3] = 1;
            else if (inistring[t] == 'P')
                coordinateMark[4] = 1;
            else if (inistring[t] == 'W')
                coordinateMark[5] = 1;
        }
    }

    if (NULL != (inistring = trajInifile->Find("HOME", "TRAJ"))) {
	// found it, now interpret it according to coordinateMark[]
	strcpy(homes, inistring);
	len = 0;
	for (t = 0; t < 6; t++) {
	    if (!coordinateMark[t]) {
		continue;	// position t at index of next non-zero mark
	    }
	    // there is a mark, so read the string for a value
	    if (1 == sscanf(&homes[len], "%s", home) &&
		1 == sscanf(home, "%lf", &d)) {
		// got an entry, index into coordinateMark[] is 't'
		if (t == 0)
		    homePose.tran.x = d;
		else if (t == 1)
		    homePose.tran.y = d;
		else if (t == 2)
		    homePose.tran.z = d;
		else if (t == 3)
		    homePose.a = d;
		else if (t == 4)
		    homePose.b = d;
		else
		    homePose.c = d;

		// position string ptr past this value
		len += strlen(home);
		// and at start of next value
		while ((len < LINELEN) && (homes[len] == ' ' || homes[len] == '\t')) {
		    len++;
		}
		if (len >= LINELEN) {
		    break;	// out of for loop
		}
	    } else {
		// badly formatted entry
		rcs_print("invalid inifile value for [TRAJ] HOME: %s\n",
			  inistring);
                return -1;
	    }
	}			// end of for-loop on coordinateMark[]
    }

    if (0 != emcTrajSetHome(homePose)) {
	if (emc_debug & EMC_DEBUG_CONFIG) {
	    rcs_print("bad return value from emcTrajSetHome\n");
	}
	return -1;
    }

    return 0;
}

/*
  iniTraj(const char *filename)

  Loads ini file parameters for trajectory, from [TRAJ] section
 */
int iniTraj(const char *filename)
{
    EmcIniFile trajInifile;

    if (trajInifile.Open(filename) == false) {
	return -1;
    }
    // load trajectory values
    if (0 != loadTraj(&trajInifile)) {
	return -1;
    }

    return 0;
}
