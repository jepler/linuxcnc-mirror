/********************************************************************
* Description: tc.h
*   Discriminate-based trajectory planning
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
#ifndef TC_TYPES_H
#define TC_TYPES_H

#include "spherical_arc.h"
#include "posemath.h"
#include "emcpos.h"
#include "emcmotcfg.h"

#define BLEND_DIST_FRACTION 0.5
/* values for endFlag */
typedef enum {
    TC_TERM_COND_STOP = 0,
    TC_TERM_COND_EXACT = 1,
    TC_TERM_COND_PARABOLIC = 2,
    TC_TERM_COND_TANGENT = 3
} tc_term_cond_t;

typedef enum {
    TC_LINEAR = 1,
    TC_CIRCULAR = 2,
    TC_RIGIDTAP = 3,
    TC_SPHERICAL = 4
} tc_motion_type_t;

typedef enum {
    TC_SYNC_NONE = 0,
    TC_SYNC_VELOCITY,
    TC_SYNC_POSITION
} tc_spindle_sync_t;

#define TC_GET_PROGRESS 0
#define TC_GET_STARTPOINT 1
#define TC_GET_ENDPOINT 2

#define TC_OPTIM_UNTOUCHED 0
#define TC_OPTIM_AT_MAX 1

#define TC_ACCEL_TRAPZ 0
#define TC_ACCEL_RAMP 1

/**
 * Spiral arc length approximation by quadratic fit.
 */
typedef struct {
    double b0;                  /* 2nd order coefficient */
    double b1;                  /* 1st order coefficient */
    double total_planar_length; /* total arc length in plane */
    int spiral_in;              /* flag indicating spiral is inward,
                                   rather than outward */
} SpiralArcLengthFit;


/* structure for individual trajectory elements */

typedef struct {
    PmCartLine xyz;
    PmCartLine abc;
    PmCartLine uvw;
} PmLine9;

typedef struct {
    PmCircle xyz;
    PmCartLine abc;
    PmCartLine uvw;
    SpiralArcLengthFit fit;
} PmCircle9;

typedef struct {
    SphericalArc xyz;
    PmCartesian abc;
    PmCartesian uvw;
} Arc9;

typedef enum {
    TAPPING, REVERSING, RETRACTION, FINAL_REVERSAL, FINAL_PLACEMENT
} RIGIDTAP_STATE;

typedef unsigned long long iomask_t; // 64 bits on both x86 and x86_64

typedef struct {
    char anychanged;
    iomask_t dio_mask;
    iomask_t aio_mask;
    signed char dios[EMCMOT_MAX_DIO];
    double aios[EMCMOT_MAX_AIO];
} syncdio_t;

typedef struct {
    PmCartLine xyz;             // original, but elongated, move down
    PmCartLine aux_xyz;         // this will be generated on the fly, for the other
                            // two moves: retraction, final placement
    PmCartesian abc;
    PmCartesian uvw;
    double reversal_target;
    double spindlepos_at_reversal;
    RIGIDTAP_STATE state;
} PmRigidTap;

typedef struct {
    double cycle_time;
    //Position stuff
    double target;          // actual segment length
    double progress;        // where are we in the segment?  0..target
    double nominal_length;
    double progress_at_sync;     // When did we sync up with the spindle?

    //Velocity
    double reqvel;          // vel requested by F word, calc'd by task
    double target_vel;      // velocity to actually track, limited by other factors
    double maxvel;          // max possible vel (feed override stops here)
    double currentvel;      // keep track of current step (vel * cycle_time)
    double finalvel;        // velocity to aim for at end of segment
    double term_vel;        // actual velocity at termination of segment
    double kink_vel;        // Temporary way to store our calculation of maximum velocity we can handle if this segment is declared tangent with the next

    //Acceleration
    double maxaccel;        // accel calc'd by task
    double acc_ratio_tan;// ratio between normal and tangential accel
    
    int id;                 // segment's serial number

    union {                 // describes the segment's start and end positions
        PmLine9 line;
        PmCircle9 circle;
        PmRigidTap rigidtap;
        Arc9 arc;
    } coords;

    int motion_type;       // TC_LINEAR (coords.line) or
                            // TC_CIRCULAR (coords.circle) or
                            // TC_RIGIDTAP (coords.rigidtap)
    int active;            // this motion is being executed
    int canon_motion_type;  // this motion is due to which canon function?
    int term_cond;          // gcode requests continuous feed at the end of
                            // this segment (g64 mode)

    int blending_next;      // segment is being blended into following segment
    double blend_vel;       // velocity below which we should start blending
    double tolerance;       // during the blend at the end of this move,
                            // stay within this distance from the path.
    int synchronized;       // spindle sync state
    double uu_per_rev;      // for sync, user units per rev (e.g. 0.0625 for 16tpi)
    double vel_at_blend_start;
    int sync_accel;         // we're accelerating up to sync with the spindle
    unsigned char enables;  // Feed scale, etc, enable bits for this move
    int atspeed;           // wait for the spindle to be at-speed before starting this move
    syncdio_t syncdio;      // synched DIO's for this move. what to turn on/off
    int indexrotary;        // which rotary axis to unlock to make this move, -1 for none
    int optimization_state;             // At peak velocity during blends)
    int on_final_decel;
    int blend_prev;
    int accel_mode;
    int splitting;          // the segment is less than 1 cycle time
                            // away from the end.
    int remove;             // Flag to remove the segment from the queue
    int active_depth;       /* Active depth (i.e. how many segments
                            * after this will it take to slow to zero
                            * speed) */
    int finalized;

    // Temporary status flags (reset each cycle)
    int is_blending;
} TC_STRUCT;

#endif				/* TC_TYPES_H */
