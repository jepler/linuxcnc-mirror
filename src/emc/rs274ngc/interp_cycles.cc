/********************************************************************
* Description: interp_cycles.cc
*
*   The bulk of the functions here control how canned cycles are
*   interpreted.
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"

static const char* plane_name(CANON_PLANE p);

/****************************************************************************/

/*! convert_cycle_g81

Returned Value: int (INTERP_OK)

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle, which
is usually drilling:
1. Move the z-axis only at the current feed rate to the specified bottom_z.
2. Retract the z-axis at traverse rate to clear_z.

See [NCMS, page 99].

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g81(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z)   //!< value of z at bottom of cycle
{
    cycle_feed(block, plane, x, y, bottom_z);
    cycle_traverse(block, plane, x, y, clear_z);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g82

Returned Value: int (INTERP_OK)

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle, which
is usually drilling:
1. Move the z_axis only at the current feed rate to the specified z-value.
2. Dwell for the given number of seconds.
3. Retract the z-axis at traverse rate to the clear_z.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g82(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z,   //!< value of z at bottom of cycle
                              double dwell)      //!< dwell time
{
  cycle_feed(block, plane, x, y, bottom_z);
  DWELL(dwell);
  cycle_traverse(block, plane, x, y, clear_z);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g83

Returned Value: int (INTERP_OK)

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle,
which is usually peck drilling:
1. Move the z-axis only at the current feed rate downward by delta or
   to the specified bottom_z, whichever is less deep.
2. Rapid back out to the R plane.
3. Rapid back down to the current hole bottom, backed off a bit.
4. Repeat steps 1, 2, and 3 until the specified bottom_z is reached.
5. Retract the z-axis at traverse rate to clear_z.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

The rapid out and back in causes any long stringers (which are common
when drilling in aluminum) to be cut off and clears chips from the
hole.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g83(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double r,  //!< initial z-value
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z,   //!< value of z at bottom of cycle
                              double delta)      //!< size of z-axis feed increment
{
  double current_depth;
  double rapid_delta;

  /* Moved the check for negative Q values here as a sign
     may be used with user defined M functions
     Thanks to Billy Singleton for pointing it out... */
  CHKS((delta <= 0.0), NCE_NEGATIVE_OR_ZERO_Q_VALUE_USED);

  rapid_delta = block->p_flag?block->p_number:_setup.parameter_g83_peck_clearance;

  for (current_depth = (r - delta);
       current_depth > bottom_z; current_depth = (current_depth - delta)) {
    cycle_feed(block, plane, x, y, current_depth);
    cycle_traverse(block, plane, x, y, r);
    cycle_traverse(block, plane, x, y, current_depth + rapid_delta);
  }
  cycle_feed(block, plane, x, y, bottom_z);
  cycle_traverse(block, plane, x, y, clear_z);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g73

Returned Value: int (INTERP_OK)

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle,
which is usually peck drilling:
1. Move the z-axis only at the current feed rate downward by delta or
   to the specified bottom_z, whichever is less deep.
2. Rapid back out a bit.
4. Repeat steps 1, 2, and 3 until the specified bottom_z is reached.
5. Retract the z-axis at traverse rate to clear_z.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

The rapid out and back in causes any long stringers (which are common
when drilling in aluminum) to be cut off and clears chips from the
hole.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g73(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double r,  //!< initial z-value
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z,   //!< value of z at bottom of cycle
                              double delta)      //!< size of z-axis feed increment
{
  double current_depth;
  double rapid_delta;
  /* Moved the check for negative Q values here as a sign
     may be used with user defined M functions
     Thanks to Billy Singleton for pointing it out... */
  CHKS((delta <= 0.0), NCE_NEGATIVE_OR_ZERO_Q_VALUE_USED);

  rapid_delta = block->p_flag?block->p_number:_setup.parameter_g73_peck_clearance;

  for (current_depth = (r - delta);
    current_depth > bottom_z; current_depth = (current_depth - delta)) {
    cycle_feed(block, plane, x, y, current_depth);
    cycle_traverse(block, plane, x, y, current_depth + rapid_delta);
  }
  cycle_feed(block, plane, x, y, bottom_z);
  cycle_traverse(block, plane, x, y, clear_z);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g74_g84

Returned Value: int
   If the spindle is not turning clockwise and motion is not g84, this returns
      NCE_SPINDLE_NOT_TURNING_CLOCKWISE_IN_G84.
   If the spindle is not turning counterclockwise and motion is not g74, this returns
      NCE_SPINDLE_NOT_TURNING_COUNTER_CLOCKWISE_IN_G74.

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle,
+which is right-hand floating-chuck tapping:
1. start the feed move towards bottom_z.
2. Move the z-axis only at the current feed rate to the specified bottom_z.
3. Stop the spindle.
4. Start the spindle counterclockwise (or cw if g74), suppressing the
   wait-for-spindle-at-speed for the following feed move out.
5. Retract the z-axis at current feed rate to clear_z.
6. Stop the spindle.
7. Start the spindle clockwise (or ccw if g74), with normal wait-for-at-speed for next feed move.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.
The direction argument must be clockwise.

For the XZ and YZ planes, this makes analogous motions.

*/

InterpReturn Interp::check_g74_g84_spindle(GCodes motion, CANON_DIRECTION dir)
{
    switch (dir) {
    case CANON_STOPPED:
        ERS(_("Spindle not turning in %s"), toString((GCodes)motion).c_str());
    case CANON_CLOCKWISE:
        CHKS((motion == G_74), _("Spindle turning clockwise in G74"));
        return INTERP_OK;
    case CANON_COUNTERCLOCKWISE:
        CHKS((motion == G_84), _("Spindle turning counterclockwise in G84"));
        return INTERP_OK;
    }
    ERS(_("Spindle state unknown for %s"), toString(motion).c_str());
}

int Interp::convert_cycle_g74_g84(block_pointer block,
                                 CANON_PLANE plane, //!< selected plane
                                 double x,  //!< x-value where cycle is executed
                                 double y,  //!< y-value where cycle is executed
                                 double clear_z,    //!< z-value of clearance plane
                                 double bottom_z,   //!< value of z at bottom of cycle
                                 CANON_DIRECTION direction, //!< direction spindle turning at outset
                                 CANON_SPEED_FEED_MODE /*mode*/,       //!< the speed-feed mode at outset
                                 int motion, double dwell, int spindle)
{

    CHP(check_g74_g84_spindle((GCodes)motion, direction));

    int save_feed_override_enable;
    int save_spindle_override_enable;

    save_feed_override_enable = GET_EXTERNAL_FEED_OVERRIDE_ENABLE();
    save_spindle_override_enable = GET_EXTERNAL_SPINDLE_OVERRIDE_ENABLE(spindle);

   switch (plane) {

    case CANON_PLANE::XY:
       DISABLE_FEED_OVERRIDE();
       DISABLE_SPEED_OVERRIDE(spindle);
       cycle_feed(block, plane, x, y, bottom_z);
       STOP_SPINDLE_TURNING(spindle);
       // the zero parameter suppresses the wait for at-speed on next feed
       if (motion == G_84)
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       else
           START_SPINDLE_CLOCKWISE(spindle);
       DWELL(dwell);
       cycle_feed(block, plane, x, y, clear_z);
       STOP_SPINDLE_TURNING(spindle);
       if (motion == G_84)
           START_SPINDLE_CLOCKWISE(spindle);
       else
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       break;

    case CANON_PLANE::YZ:
       DISABLE_FEED_OVERRIDE();
       DISABLE_SPEED_OVERRIDE(spindle);
       cycle_feed(block, plane, bottom_z, x, y);
       STOP_SPINDLE_TURNING(spindle);
       if (motion == G_84)
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       else
           START_SPINDLE_CLOCKWISE(spindle);
       DWELL(dwell);
       cycle_feed(block, plane, clear_z, x, y);
       STOP_SPINDLE_TURNING(spindle);
       if (motion == G_84)
           START_SPINDLE_CLOCKWISE(spindle);
       else
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       break;

    case CANON_PLANE::XZ:
       DISABLE_FEED_OVERRIDE();
       DISABLE_SPEED_OVERRIDE(spindle);
       cycle_feed(block, plane, y, bottom_z, x);
       STOP_SPINDLE_TURNING(spindle);
       if (motion == G_84)
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       else
           START_SPINDLE_CLOCKWISE(spindle);
       DWELL(dwell);
       cycle_feed(block, plane, y, clear_z, x);
       STOP_SPINDLE_TURNING(spindle);
       if (motion == G_84)
           START_SPINDLE_CLOCKWISE(spindle);
       else
           START_SPINDLE_COUNTERCLOCKWISE(spindle);
       break;

    default:
       ERS("%s for plane %s not implemented",
           toString((GCodes)motion).c_str(), plane_name(plane));
    }
   if(save_feed_override_enable)
    ENABLE_FEED_OVERRIDE();
   if(save_spindle_override_enable)
    ENABLE_SPEED_OVERRIDE(spindle);

   return INTERP_OK;

#if 0
  START_SPEED_FEED_SYNCH();
  cycle_feed(block, plane, x, y, bottom_z);

  cycle_feed(block, plane, x, y, clear_z);
  if (mode != CANON_SYNCHED)
    STOP_SPEED_FEED_SYNCH();
  STOP_SPINDLE_TURNING();
  START_SPINDLE_CLOCKWISE();
#endif
}

/****************************************************************************/

/*! convert_cycle_g85

Returned Value: int (INTERP_OK)

Side effects:
   A number of moves are made as described below.

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle,
which is usually boring or reaming:
1. Move the z-axis only at the current feed rate to the specified z-value.
2. Retract the z-axis at the current feed rate to clear_z.

CYCLE_MACRO has positioned the tool at (x, y, r, ?, ?) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g85(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double r,  // retract plane
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z)   //!< value of z at bottom of cycle
{
  cycle_feed(block, plane, x, y, bottom_z);
  cycle_feed(block, plane, x, y, r);
  cycle_traverse(block, plane, x, y, clear_z);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g86

Returned Value: int
   If the spindle is not turning clockwise or counterclockwise,
   this returns NCE_SPINDLE_NOT_TURNING_IN_G86.
   Otherwise, it returns INTERP_OK.

Side effects:
   A number of moves are made as described below.

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the RS274/NGC following cycle,
which is usually boring:
1. Move the z-axis only at the current feed rate to bottom_z.
2. Dwell for the given number of seconds.
3. Stop the spindle turning.
4. Retract the z-axis at traverse rate to clear_z.
5. Restart the spindle in the direction it was going.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g86(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z,   //!< value of z at bottom of cycle
                              double dwell,      //!< dwell time
                              CANON_DIRECTION direction, //!< direction spindle turning at outset
                              int spindle)       // the spindle being used
{
  CHKS(((direction != CANON_CLOCKWISE) &&
       (direction != CANON_COUNTERCLOCKWISE)),
      NCE_SPINDLE_NOT_TURNING_IN_G86);

  cycle_feed(block, plane, x, y, bottom_z);
  DWELL(dwell);
  STOP_SPINDLE_TURNING(spindle);
  cycle_traverse(block, plane, x, y, clear_z);
  if (direction == CANON_CLOCKWISE)
    START_SPINDLE_CLOCKWISE(spindle);
  else
    START_SPINDLE_COUNTERCLOCKWISE(spindle);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g87

Returned Value: int
   If the spindle is not turning clockwise or counterclockwise,
   this returns NCE_SPINDLE_NOT_TURNING_IN_G87.
   Otherwise, it returns INTERP_OK.

Side effects:
   A number of moves are made as described below. This cycle is a
   modified version of [Monarch, page 5-24] since [NCMS, pages 98 - 100]
   gives no clue as to what the cycle is supposed to do. [KT] does not
   have a back boring cycle. [Fanuc, page 132] in "Canned cycle II"
   describes the G87 cycle as given here, except that the direction of
   spindle turning is always clockwise and step 7 below is omitted
   in [Fanuc].

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle, which
is usually back boring.  The situation is that you have a through hole
and you want to counterbore the bottom of hole. To do this you put an
L-shaped tool in the spindle with a cutting surface on the UPPER side
of its base. You stick it carefully through the hole when it is not
spinning and is oriented so it fits through the hole, then you move it
so the stem of the L is on the axis of the hole, start the spindle,
and feed the tool upward to make the counterbore. Then you get the
tool out of the hole.

1. Move at traverse rate parallel to the XY-plane to the point
   with x-value offset_x and y-value offset_y.
2. Stop the spindle in a specific orientation.
3. Move the z-axis only at traverse rate downward to the bottom_z.
4. Move at traverse rate parallel to the XY-plane to the x,y location.
5. Start the spindle in the direction it was going before.
6. Move the z-axis only at the given feed rate upward to the middle_z.
7. Move the z-axis only at the given feed rate back down to bottom_z.
8. Stop the spindle in the same orientation as before.
9. Move at traverse rate parallel to the XY-plane to the point
   with x-value offset_x and y-value offset_y.
10. Move the z-axis only at traverse rate to the clear z value.
11. Move at traverse rate parallel to the XY-plane to the specified x,y
    location.
12. Restart the spindle in the direction it was going before.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) before this starts.

It might be useful to add a check that clear_z > middle_z > bottom_z.
Without the check, however, this can be used to counterbore a hole in
material that can only be accessed through a hole in material above it.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g87(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double offset_x,   //!< x-axis offset position
                              double y,  //!< y-value where cycle is executed
                              double offset_y,   //!< y-axis offset position
                              double r,  //!< z_value of r_plane
                              double clear_z,    //!< z-value of clearance plane
                              double middle_z,   //!< z-value of top of back bore
                              double bottom_z,   //!< value of z at bottom of cycle
                              CANON_DIRECTION direction, //!< direction spindle turning at outset
                              int spindle)       // the spindle being used
{
  CHKS(((direction != CANON_CLOCKWISE) &&
       (direction != CANON_COUNTERCLOCKWISE)),
      NCE_SPINDLE_NOT_TURNING_IN_G87);

  cycle_traverse(block, plane, offset_x, offset_y, r);
  STOP_SPINDLE_TURNING(spindle);
  ORIENT_SPINDLE(spindle, 0.0, direction);
  cycle_traverse(block, plane, offset_x, offset_y, bottom_z);
  cycle_traverse(block, plane, x, y, bottom_z);
  if (direction == CANON_CLOCKWISE)
    START_SPINDLE_CLOCKWISE(spindle);
  else
    START_SPINDLE_COUNTERCLOCKWISE(spindle);
  cycle_feed(block, plane, x, y, middle_z);
  cycle_feed(block, plane, x, y, bottom_z);
  STOP_SPINDLE_TURNING(spindle);
  ORIENT_SPINDLE(spindle,0.0, direction);
  cycle_traverse(block, plane, offset_x, offset_y, bottom_z);
  cycle_traverse(block, plane, offset_x, offset_y, clear_z);
  cycle_traverse(block, plane, x, y, clear_z);
  if (direction == CANON_CLOCKWISE)
    START_SPINDLE_CLOCKWISE(spindle);
  else
    START_SPINDLE_COUNTERCLOCKWISE(spindle);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g88

Returned Value: int
   If the spindle is not turning clockwise or counterclockwise, this
   returns NCE_SPINDLE_NOT_TURNING_IN_G88.
   Otherwise, it returns INTERP_OK.

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

For the XY plane, this implements the following RS274/NGC cycle,
which is usually boring:
1. Move the z-axis only at the current feed rate to the specified z-value.
2. Dwell for the given number of seconds.
3. Stop the spindle turning.
4. Stop the program so the operator can retract the spindle manually.
5. Restart the spindle.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g88(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double bottom_z,   //!< value of z at bottom of cycle
                              double dwell,      //!< dwell time
                              CANON_DIRECTION direction, //!< direction spindle turning at outset
                              int spindle)       // the spindle being used
{
  CHKS(((direction != CANON_CLOCKWISE) &&
       (direction != CANON_COUNTERCLOCKWISE)),
      NCE_SPINDLE_NOT_TURNING_IN_G88);

  cycle_feed(block, plane, x, y, bottom_z);
  DWELL(dwell);
  STOP_SPINDLE_TURNING(spindle);
  PROGRAM_STOP();               /* operator retracts the spindle here */
  if (direction == CANON_CLOCKWISE)
    START_SPINDLE_CLOCKWISE(spindle);
  else
    START_SPINDLE_COUNTERCLOCKWISE(spindle);

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_g89

Returned Value: int (INTERP_OK)

Side effects: See below

Called by:
   convert_cycle_xy
   convert_cycle_yz
   convert_cycle_zx

This implements the following RS274/NGC cycle, which is intended for boring:
1. Move the z-axis only at the current feed rate to the specified z-value.
2. Dwell for the given number of seconds.
3. Retract the z-axis at the current feed rate to clear_z.

CYCLE_MACRO has positioned the tool at (x, y, r, a, b, c) when this starts.

For the XZ and YZ planes, this makes analogous motions.

*/

int Interp::convert_cycle_g89(block_pointer block,
                              CANON_PLANE plane, //!< selected plane
                              double x,  //!< x-value where cycle is executed
                              double y,  //!< y-value where cycle is executed
                              double clear_z,    //!< z-value of clearance plane
                              double bottom_z,   //!< value of z at bottom of cycle
                              double dwell)      //!< dwell time
{
  cycle_feed(block, plane, x, y, bottom_z);
  DWELL(dwell);
  cycle_feed(block, plane, x, y, clear_z);

  return INTERP_OK;
}

static const char* plane_name(CANON_PLANE p) {
  switch(p) {
    case CANON_PLANE::XY: return "XY";
    case CANON_PLANE::YZ: return "YZ";
    case CANON_PLANE::XZ: return "XZ";
    case CANON_PLANE::UV: return "UV";
    case CANON_PLANE::VW: return "VW";
    case CANON_PLANE::UW: return "UW";
    default:             return "invalid";
  }
}

/****************************************************************************/

/*! convert_cycle

Returned Value: int
   If any of the specific functions called returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The r-value is not given the first time this code is called after
      some other motion mode has been in effect:
      NCE_R_CLEARANCE_PLANE_UNSPECIFIED_IN_CYCLE
   2. The l number is zero: NCE_CANNOT_DO_ZERO_REPEATS_OF_CYCLE
   3. The currently selected plane in not XY, YZ, or XZ.
      NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ

Side effects:
   A number of moves are made to execute a canned cycle. The current
   position is reset. The values of the cycle attributes in the settings
   may be reset.

Called by: convert_motion

This function makes a couple checks and then calls one of three
functions, according to which plane is currently selected.

See the documentation of convert_cycle_xy for most of the details.

*/

int Interp::convert_cycle(int motion,    //!< a G-code between G_81 and G_89, a canned cycle
                         block_pointer block,   //!< pointer to a block of RS274 instructions
                         setup_pointer settings)        //!< pointer to machine settings
{
  CANON_PLANE plane;

  CHKS((settings->feed_rate == 0.0), _("Cannot feed with zero feed rate"));
  CHKS((settings->feed_mode == FEED_MODE::INVERSE_TIME), _("Cannot use inverse time feed with canned cycles"));
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF), _("Cannot use canned cycles with cutter compensation on"));

  plane = settings->plane;
  if (!block->r_flag) {
    if (settings->motion_mode == motion)
      block->r_number = settings->cycle_r;
    else
      ERS(NCE_R_CLEARANCE_PLANE_UNSPECIFIED_IN_CYCLE);
  }

  CHKS((block->l_number == 0), NCE_CANNOT_DO_ZERO_REPEATS_OF_CYCLE);
  if (block->l_number == -1)
    block->l_number = 1;

  switch(plane) {
  case CANON_PLANE::XY:
  case CANON_PLANE::XZ:
  case CANON_PLANE::YZ:
    CHKS(block->u_flag, "Cannot put a U in a canned cycle in the %s plane",
	plane_name(settings->plane));
    CHKS(block->v_flag, "Cannot put a V in a canned cycle in the %s plane",
	plane_name(settings->plane));
    CHKS(block->w_flag, "Cannot put a W in a canned cycle in the %s plane",
	plane_name(settings->plane));
    break;

  case CANON_PLANE::UV:
  case CANON_PLANE::VW:
  case CANON_PLANE::UW:
    CHKS(block->x_flag, "Cannot put an X in a canned cycle in the %s plane",
	plane_name(settings->plane));
    CHKS(block->y_flag, "Cannot put a Y in a canned cycle in the %s plane",
	plane_name(settings->plane));
    CHKS(block->z_flag, "Cannot put a Z in a canned cycle in the %s plane",
	plane_name(settings->plane));
  }

  //KLUDGE ugly way to save / restore motion mode flag so that state
  //tag displays correctly
  int save_mode = settings->motion_mode;
  settings->motion_mode = motion;
  write_canon_state_tag(block, settings);
  settings->motion_mode = save_mode;
  // end KLUDGE

  if (plane == CANON_PLANE::XY) {
    CHP(convert_cycle_xy(motion, block, settings));
  } else if (plane == CANON_PLANE::YZ) {
    CHP(convert_cycle_yz(motion, block, settings));
  } else if (plane == CANON_PLANE::XZ) {
    CHP(convert_cycle_zx(motion, block, settings));
  } else if (plane == CANON_PLANE::UV) {
    CHP(convert_cycle_uv(motion, block, settings));
  } else if (plane == CANON_PLANE::VW) {
    CHP(convert_cycle_vw(motion, block, settings));
  } else if (plane == CANON_PLANE::UW) {
    CHP(convert_cycle_wu(motion, block, settings));
  } else
    ERS(NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ);

  settings->cycle_l = block->l_number;
  settings->cycle_r = block->r_number;
  settings->motion_mode = motion;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_xy

Returned Value: int
   If any of the specific functions called returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The z-value is not given the first time this code is called after
      some other motion mode has been in effect:
      NCE_Z_VALUE_UNSPECIFIED_IN_XY_PLANE_CANNED_CYCLE
   2. The r clearance plane is below the bottom_z:
      NCE_R_LESS_THAN_Z_IN_CYCLE_IN_XY_PLANE
   3. the distance mode is neither absolute or incremental:
      NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91
   4. G82, G86, G88, or G89 is called when it is not already in effect,
      and no p number is in the block:
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89
   5. G83 is called when it is not already in effect,
      and no q number is in the block: NCE_Q_WORD_MISSING_WITH_G83_OR_M66
   6. G87 is called when it is not already in effect,
      and any of the i number, j number, or k number is missing:
      NCE_I_WORD_MISSING_WITH_G87
      NCE_J_WORD_MISSING_WITH_G87
      NCE_K_WORD_MISSING_WITH_G87
   7. the G-code is not between G_81 and G_89.
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   A number of moves are made to execute the G-code

Called by: convert_cycle

The function does not require that any of x,y,z, or r be specified in
the block, except that if the last motion mode command executed was
not the same as this one, the r-value and z-value must be specified.

This function is handling the repeat feature of RS274/NGC, wherein
the L word represents the number of repeats [NCMS, page 99]. We are
not allowing L=0, contrary to the manual. We are allowing L > 1
in absolute distance mode to mean "do the same thing in the same
place several times", as provided in the manual, although this seems
abnormal.

In incremental distance mode, x, y, and r values are treated as
increments to the current position and z as an increment from r.  In
absolute distance mode, x, y, r, and z are absolute. In g87, i and j
will always be increments, regardless of the distance mode setting, as
implied in [NCMS, page 98], but k (z-value of top of counterbore) will
be an absolute z-value in absolute distance mode, and an increment
(from bottom z) in incremental distance mode.

If the r position of a cycle is above the current_z position, this
retracts the z-axis to the r position before moving parallel to the
XY plane.

In the code for this function, there is a nearly identical "for" loop
in every case of the switch. The loop is the done with a compiler
macro, "CYCLE_MACRO" so that the code is easy to read, automatically
kept identical from case to case and, and much shorter than it would
be without the macro. The loop could be put outside the switch, but
then the switch would run every time around the loop, not just once,
as it does here. The loop could also be placed in the called
functions, but then it would not be clear that all the loops are the
same, and it would be hard to keep them the same when the code is
modified.  The macro would be very awkward as a regular function
because it would have to be passed all of the arguments used by any of
the specific cycles, and, if another switch in the function is to be
avoided, it would have to passed a function pointer, but the different
cycle functions have different arguments so the type of the pointer
could not be declared unless the cycle functions were re-written to
take the same arguments (in which case most of them would have several
unused arguments).

The motions within the CYCLE_MACRO (but outside a specific cycle) are
a straight traverse parallel to the selected plane to the given
position in the plane and a straight traverse of the third axis only
(if needed) to the r position.

The CYCLE_MACRO is defined here but is also used in convert_cycle_yz
and convert_cycle_zx. The variables aa, bb, and cc are used in
CYCLE_MACRO and in the other two functions just mentioned. Those
variables represent the first axis of the selected plane, the second
axis of the selected plane, and third axis which is perpendicular to
the selected plane.  In this function aa represents x, bb represents
y, and cc represents z. This usage makes it possible to have only one
version of each of the cycle functions.  The cycle_traverse and
cycle_feed functions help accomplish this.

The height of the retract move at the end of each repeat of a cycle is
determined by the setting of the retract_mode: either to the r
position (if the retract_mode is R_PLANE) or to the original
z-position (if that is above the r position and the retract_mode is
not R_PLANE). This is a slight departure from [NCMS, page 98], which
does not require checking that the original z-position is above r.

The rotary axes may not move during a canned cycle.

*/

int Interp::convert_cycle_xy(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->current_z;

  plane = CANON_PLANE::XY;
  if (settings->motion_mode != motion) {
    CHKS((!block->z_flag),
        _readers[(int)'z']? NCE_Z_VALUE_UNSPECIFIED_IN_XY_PLANE_CANNED_CYCLE: _("G17 canned cycle is not possible on a machine without Z axis"));
  }
  block->z_number =
    block->z_flag ? block->z_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    double radius, theta;
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->z_number;
    if(block->radius_flag)
        radius = block->radius;
    else
        radius = hypot(settings->current_y, settings->current_x);
    if(block->theta_flag)
        theta = D2R(block->theta);
    else
        theta = atan2(settings->current_y, settings->current_x);
    if(block->radius_flag || block->theta_flag) {
        aa = radius * cos(theta);
        bb = radius * sin(theta);
    } else {
        aa = block->x_flag ? block->x_number : settings->current_x;
        bb = block->y_flag ? block->y_number : settings->current_y;
    }
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->x_flag) aa_increment = block->x_number;
    if (block->y_flag) bb_increment = block->y_number;
    if (block->radius_flag) radius_increment = block->radius;
    if (block->theta_flag) theta_increment = D2R(block->theta);
    r = (block->r_number + old_cc);
    cc = (r + block->z_number); /* [NCMS, page 98] */
    aa = settings->current_x;
    bb = settings->current_y;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_Z_IN_CYCLE_IN_XY_PLANE);

  // First motion of a canned cycle (maybe): if we're below the R plane,
  // rapid straight up to the R plane.
  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, settings->current_x, settings->current_y, r,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      settings->u_current, settings->v_current, settings->w_current);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
      CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::XY, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::XY, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::XY, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::XY, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
      block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
      if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
      }
      CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::XY, aa, bb, clear_cc, cc,
                                       settings->spindle_turning[settings->active_spindle],
                                       settings->speed_feed_mode,
                                       motion, block->p_number, settings->active_spindle))
      settings->cycle_p = block->p_number;

      break;
  case G_85:
      CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::XY, aa, bb, r, clear_cc, cc))
      break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::XY, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle))
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      k = (cc + k);             /* k always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::XY, aa, (aa + i), bb,
                                  (bb + j), r, clear_cc, k, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::XY, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle))
    settings->cycle_p = block->p_number;
    break;

  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::XY, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->current_x = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->current_y = bb;
  settings->current_z = clear_cc;
  settings->cycle_cc = block->z_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}



int Interp::convert_cycle_uv(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  int spindle = settings->active_spindle;
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->w_current;

  plane = CANON_PLANE::UV;
  if (settings->motion_mode != motion) {
    CHKS((!block->w_flag),
        _readers[(int)'w']? NCE_W_VALUE_UNSPECIFIED_IN_UV_PLANE_CANNED_CYCLE: _("G17.1 canned cycle is not possible on a machine without W axis"));
  }
  block->w_number =
    block->w_flag ? block->w_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->w_number;
    aa = block->u_flag ? block->u_number : settings->u_current;
    bb = block->v_flag ? block->v_number : settings->v_current;
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->u_flag) aa_increment = block->u_number;
    if (block->v_flag) bb_increment = block->v_number;
    r = (block->r_number + old_cc);
    cc = (r + block->w_number); /* [NCMS, page 98] */
    aa = settings->u_current;
    bb = settings->v_current;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_W_IN_CYCLE_IN_UV_PLANE);

  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, settings->current_x, settings->current_y, settings->current_z,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      settings->u_current, settings->v_current, r);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
      CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::UV, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::UV, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::UV, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::UV, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
    block->p_number =
    block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
    }
    CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::UV, aa, bb, clear_cc, cc,
                                       settings->spindle_turning[spindle],
                                       settings->speed_feed_mode,
                                       motion, block->p_number, settings->active_spindle))
         settings->cycle_p = block->p_number;
    break;
  case G_85:
    CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::UV, aa, bb, r, clear_cc, cc))
      break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::UV, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle))
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      k = (cc + k);             /* k always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::UV, aa, (aa + i), bb,
                                  (bb + j), r, clear_cc, k, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle))
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::UV, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle))
    settings->cycle_p = block->p_number;
    break;
  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::UV, aa, bb, clear_cc, cc,
                                  block->p_number))
    settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->u_current = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->v_current = bb;
  settings->w_current = clear_cc;
  settings->cycle_cc = block->w_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cycle_yz

Returned Value: int
   If any of the specific functions called returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The x-value is not given the first time this code is called after
      some other motion mode has been in effect:
      NCE_X_VALUE_UNSPECIFIED_IN_YZ_PLANE_CANNED_CYCLE
   2. The r clearance plane is below the bottom_x:
      NCE_R_LESS_THAN_X_IN_CYCLE_IN_YZ_PLANE
   3. the distance mode is neither absolute or incremental:
      NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91
   4. G82, G86, G88, or G89 is called when it is not already in effect,
      and no p number is in the block:
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89
   5. G83 is called when it is not already in effect,
      and no q number is in the block: NCE_Q_WORD_MISSING_WITH_G83
   6. G87 is called when it is not already in effect,
      and any of the i number, j number, or k number is missing:
      NCE_I_WORD_MISSING_WITH_G87
      NCE_J_WORD_MISSING_WITH_G87
      NCE_K_WORD_MISSING_WITH_G87
   7. the G-code is not between G_81 and G_89.
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   A number of moves are made to execute a canned cycle.

Called by: convert_cycle

See the documentation of convert_cycle_xy. This function is entirely
similar. In this function aa represents y, bb represents z, and cc
represents x.

The CYCLE_MACRO is defined just before the convert_cycle_xy function.

Tool length offsets work only when the tool axis is parallel to the
Z-axis, so if this function is used, tool length offsets should be
turned off, and the NC code written to take tool length into account.

*/

int Interp::convert_cycle_yz(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274/NGC instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  int spindle = settings->active_spindle;
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  // save the current tolerance, to restore it later on
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->current_x;

  plane = CANON_PLANE::YZ;
  if (settings->motion_mode != motion) {
    CHKS((!block->x_flag),
        _readers[(int)'x']? NCE_X_VALUE_UNSPECIFIED_IN_YZ_PLANE_CANNED_CYCLE: _("G19 canned cycle is not possible on a machine without X axis"));
  }
  block->x_number =
    block->x_flag ? block->x_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->x_number;
    aa = block->y_flag ? block->y_number : settings->current_y;
    bb = block->z_flag ? block->z_number : settings->current_z;
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->y_flag) aa_increment = block->y_number;
    if (block->z_flag) bb_increment = block->z_number;
    r = (block->r_number + old_cc);
    cc = (r + block->x_number); /* [NCMS, page 98] */
    aa = settings->current_y;
    bb = settings->current_z;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_X_IN_CYCLE_IN_YZ_PLANE);

  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, r, settings->current_y, settings->current_z,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      settings->u_current, settings->v_current, settings->w_current);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
    CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::YZ, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::YZ, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::YZ, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::YZ, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
     block->p_number =
     block->p_number == -1.0 ? settings->cycle_p : block->p_number;
     if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
     CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::YZ, aa, bb, clear_cc, cc,
                                    settings->spindle_turning[spindle],
                                    settings->speed_feed_mode,
                                    motion, block->p_number, settings->active_spindle))
     settings->cycle_p = block->p_number;
     break;
  case G_85:
    CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::YZ, aa, bb, r, clear_cc, cc))
      break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::YZ, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      i = (cc + i);             /* i always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::YZ, aa, (aa + j), bb,
                                  (bb + k), r, clear_cc, i, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::YZ, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::YZ, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->current_y = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->current_z = bb;
  settings->current_x = clear_cc;
  settings->cycle_cc = block->x_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}


int Interp::convert_cycle_vw(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274/NGC instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  // save the current tolerance, to restore it later on
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->u_current;

  plane = CANON_PLANE::VW;
  if (settings->motion_mode != motion) {
    CHKS((!block->u_flag),
        _readers[(int)'u']? NCE_U_VALUE_UNSPECIFIED_IN_VW_PLANE_CANNED_CYCLE: _("G19.1 canned cycle is not possible on a machine without U axis"));
  }
  block->u_number =
    block->u_flag ? block->u_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->u_number;
    aa = block->v_flag ? block->v_number : settings->v_current;
    bb = block->w_flag ? block->w_number : settings->w_current;
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->v_flag) aa_increment = block->v_number;
    if (block->w_flag) bb_increment = block->w_number;
    r = (block->r_number + old_cc);
    cc = (r + block->u_number); /* [NCMS, page 98] */
    aa = settings->v_current;
    bb = settings->w_current;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_U_IN_CYCLE_IN_VW_PLANE);

  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, settings->current_x, settings->current_y, settings->current_z,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      r, settings->v_current, settings->w_current);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
    CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::VW, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::VW, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::VW, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::VW, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
    block->p_number =
    block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
    }
    CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::VW, aa, bb, clear_cc, cc,
                                     settings->spindle_turning[settings->active_spindle],
                                     settings->speed_feed_mode,
                                     motion, block->p_number, settings->active_spindle))
    settings->cycle_p = block->p_number;
    break;
  case G_85:
    CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::VW, aa, bb, r, clear_cc, cc))
      break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::VW, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      i = (cc + i);             /* i always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::VW, aa, (aa + j), bb,
                                  (bb + k), r, clear_cc, i, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::VW, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::VW, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->v_current = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->w_current = bb;
  settings->u_current = clear_cc;
  settings->cycle_cc = block->u_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}


/****************************************************************************/

/*! convert_cycle_zx

Returned Value: int
   If any of the specific functions called returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the ERROR code shown.
   Otherwise, it returns INTERP_OK.
   1. The y-value is not given the first time this code is called after
      some other motion mode has been in effect:
      NCE_Y_VALUE_UNSPECIFIED_IN_XZ_PLANE_CANNED_CYCLE
   2. The r clearance plane is below the bottom_y:
      NCE_R_LESS_THAN_Y_IN_CYCLE_IN_XZ_PLANE
   3. the distance mode is neither absolute or incremental:
      NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91
   4. G82, G86, G88, or G89 is called when it is not already in effect,
      and no p number is in the block:
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88
      NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89
   5. G83 is called when it is not already in effect,
      and no q number is in the block: NCE_Q_WORD_MISSING_WITH_G83
   6. G87 is called when it is not already in effect,
      and any of the i number, j number, or k number is missing:
      NCE_I_WORD_MISSING_WITH_G87
      NCE_J_WORD_MISSING_WITH_G87
      NCE_K_WORD_MISSING_WITH_G87
   7. the G-code is not between G_81 and G_89.
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   A number of moves are made to execute a canned cycle.

Called by: convert_cycle

See the documentation of convert_cycle_xy. This function is entirely
similar. In this function aa represents z, bb represents x, and cc
represents y.

The CYCLE_MACRO is defined just before the convert_cycle_xy function.

Tool length offsets work only when the tool axis is parallel to the
Z-axis, so if this function is used, tool length offsets should be
turned off, and the NC code written to take tool length into account.

It is a little distracting that this function uses zx in some places
and xz in others; uniform use of zx would be nice, since that is the
order for a right-handed coordinate system. Also with that usage,
permutation of the symbols x, y, and z would allow for automatically
converting the convert_cycle_xy function (or convert_cycle_yz) into
the convert_cycle_xz function. However, the canonical interface uses
CANON_PLANE::XZ.

*/

int Interp::convert_cycle_zx(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  // save current path-following tolerance, to restore it later on
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->current_y;

  plane = CANON_PLANE::XZ;
  if (settings->motion_mode != motion) {
    CHKS((!block->y_flag),
        _readers[(int)'y']? NCE_Y_VALUE_UNSPECIFIED_IN_XZ_PLANE_CANNED_CYCLE: _("G18 canned cycle is not possible on a machine without Y axis"));
  }
  block->y_number =
    block->y_flag ? block->y_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->y_number;
    aa = block->z_flag ? block->z_number : settings->current_z;
    bb = block->x_flag ? block->x_number : settings->current_x;
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->z_flag) aa_increment = block->z_number;
    if (block->x_flag) bb_increment = block->x_number;
    r = (block->r_number + old_cc);
    cc = (r + block->y_number); /* [NCMS, page 98] */
    aa = settings->current_z;
    bb = settings->current_x;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_Y_IN_CYCLE_IN_XZ_PLANE);

  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, settings->current_x, r, settings->current_z,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      settings->u_current, settings->v_current, settings->w_current);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
    CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::XZ, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::XZ, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::XZ, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::XZ, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
     block->p_number =
         block->p_number == -1.0 ? settings->cycle_p : block->p_number;
     if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid E-number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
    CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::XZ, aa, bb, clear_cc, cc,
                                     settings->spindle_turning[settings->active_spindle],
                                     settings->speed_feed_mode,
                                     motion, block->p_number, settings->active_spindle))
       settings->cycle_p = block->p_number;
    break;
  case G_85:
    CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::XZ, aa, bb, r, clear_cc, cc));
    break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::XZ, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      j = (cc + j);             /* j always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::XZ, aa, (aa + k), bb,
                                  (bb + i), r, clear_cc, j, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::XZ, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::XZ, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->current_z = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->current_x = bb;
  settings->current_y = clear_cc;
  settings->cycle_cc = block->y_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}

int Interp::convert_cycle_wu(int motion, //!< a G-code between G_81 and G_89, a canned cycle
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  double aa;
  double aa_increment=0.;
  double bb;
  double bb_increment=0.;
  double cc;
  double clear_cc;
  double i;
  double j;
  double k;
  double old_cc;
  double radius_increment = 0.;
  double theta_increment = 0.;
  CANON_PLANE plane;
  double r;
  int repeat;
  CANON_MOTION_MODE save_mode;
  // save current path-following tolerance, to restore it later on
  double save_tolerance, save_cam_tolerance;
  double current_cc = settings->v_current;

  plane = CANON_PLANE::UW;
  if (settings->motion_mode != motion) {
    CHKS((!block->v_flag),
        _readers[(int)'v']? NCE_V_VALUE_UNSPECIFIED_IN_UW_PLANE_CANNED_CYCLE: _("G18.1 canned cycle is not possible on a machine without V axis"));
  }
  block->v_number =
    block->v_flag ? block->v_number : settings->cycle_cc;
  if(settings->cycle_il_flag) {
      old_cc = settings->cycle_il;
  } else {
      old_cc = settings->cycle_il = current_cc;
      settings->cycle_il_flag = true;
  }

  if (settings->distance_mode == DISTANCE_MODE::ABSOLUTE) {
    aa_increment = 0.0;
    bb_increment = 0.0;
    r = block->r_number;
    cc = block->v_number;
    aa = block->w_flag ? block->w_number : settings->w_current;
    bb = block->u_flag ? block->u_number : settings->u_current;
  } else if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
    if (block->w_flag) aa_increment = block->w_number;
    if (block->u_flag) bb_increment = block->u_number;
    r = (block->r_number + old_cc);
    cc = (r + block->v_number); /* [NCMS, page 98] */
    aa = settings->w_current;
    bb = settings->u_current;
  } else
    ERS(NCE_BUG_DISTANCE_MODE_NOT_G90_OR_G91);
  CHKS((r < cc), NCE_R_LESS_THAN_V_IN_CYCLE_IN_UW_PLANE);

  if (old_cc < r) {
    STRAIGHT_TRAVERSE(block->line_number, settings->current_x, settings->current_y, settings->current_z,
                      settings->AA_current, settings->BB_current, settings->CC_current,
                      settings->u_current, r, settings->w_current);
    old_cc = r;
    current_cc = old_cc;
  }
  clear_cc = (settings->retract_mode == RETRACT_MODE::R_PLANE) ? r : old_cc;

  save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
  save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
  save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
  if (save_mode != CANON_EXACT_PATH)
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

  switch (motion) {
  case G_81:
    CYCLE_MACRO(convert_cycle_g81(block, CANON_PLANE::UW, aa, bb, clear_cc, cc))
      break;
  case G_82:
    CHKS(((settings->motion_mode != G_82) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G82);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g82(block, CANON_PLANE::UW, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  case G_73:
    CHKS(((settings->motion_mode != G_73) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G73);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g73(block, CANON_PLANE::UW, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_83:
    CHKS(((settings->motion_mode != G_83) && (block->q_number == -1.0)),
        NCE_Q_WORD_MISSING_WITH_G83);
    block->q_number =
      block->q_number == -1.0 ? settings->cycle_q : block->q_number;
    CYCLE_MACRO(convert_cycle_g83(block, CANON_PLANE::UW, aa, bb, r, clear_cc, cc,
                                  block->q_number))
      settings->cycle_q = block->q_number;
    break;
  case G_74:
  case G_84:
     block->p_number =
     block->p_number == -1.0 ? settings->cycle_p : block->p_number;
     if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
     CYCLE_MACRO(convert_cycle_g74_g84(block, CANON_PLANE::UW, aa, bb, clear_cc, cc,
                                     settings->spindle_turning[settings->active_spindle],
                                     settings->speed_feed_mode,
                                     motion, block->p_number, settings->active_spindle))
      settings->cycle_p = block->p_number;
      break;
  case G_85:
    CYCLE_MACRO(convert_cycle_g85(block, CANON_PLANE::UW, aa, bb, r, clear_cc, cc))
      break;
  case G_86:
    CHKS(((settings->motion_mode != G_86) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G86);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
    CYCLE_MACRO(convert_cycle_g86(block, CANON_PLANE::UW, aa, bb, clear_cc, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_87:
    if (settings->motion_mode != G_87) {
      CHKS((!block->i_flag), NCE_I_WORD_MISSING_WITH_G87);
      CHKS((!block->j_flag), NCE_J_WORD_MISSING_WITH_G87);
      CHKS((!block->k_flag), NCE_K_WORD_MISSING_WITH_G87);
    }
    if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
    i = block->i_flag ? block->i_number : settings->cycle_i;
    j = block->j_flag ? block->j_number : settings->cycle_j;
    k = block->k_flag ? block->k_number : settings->cycle_k;
    settings->cycle_i = i;
    settings->cycle_j = j;
    settings->cycle_k = k;
    if (settings->distance_mode == DISTANCE_MODE::INCREMENTAL) {
      j = (cc + j);             /* j always absolute in function call below */
    }
    CYCLE_MACRO(convert_cycle_g87(block, CANON_PLANE::UW, aa, (aa + k), bb,
                                  (bb + i), r, clear_cc, j, cc,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    break;
  case G_88:
    CHKS(((settings->motion_mode != G_88) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G88);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    if (block->dollar_flag){
        CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
            (_("Invalid spindle ($) number in G74/G84 cycle")));
        settings->active_spindle = (int)block->dollar_number;
     }
    CYCLE_MACRO(convert_cycle_g88(block, CANON_PLANE::UW, aa, bb, cc,
                                  block->p_number,
                                  settings->spindle_turning[settings->active_spindle],
                                  settings->active_spindle));
    settings->cycle_p = block->p_number;
    break;
  case G_89:
    CHKS(((settings->motion_mode != G_89) && (block->p_number == -1.0)),
        NCE_DWELL_TIME_P_WORD_MISSING_WITH_G89);
    block->p_number =
      block->p_number == -1.0 ? settings->cycle_p : block->p_number;
    CYCLE_MACRO(convert_cycle_g89(block, CANON_PLANE::UW, aa, bb, clear_cc, cc,
                                  block->p_number))
      settings->cycle_p = block->p_number;
    break;
  default:
    ERS(NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  }
  settings->w_current = aa;     /* CYCLE_MACRO updates aa and bb */
  settings->u_current = bb;
  settings->v_current = clear_cc;
  settings->cycle_cc = block->v_number;

  if (save_mode != CANON_EXACT_PATH) {
    SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
    SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
  }

  return INTERP_OK;
}
/****************************************************************************/

/*! cycle_feed

Returned Value: int (INTERP_OK)

Side effects:
  STRAIGHT_FEED is called.

Called by:
  convert_cycle_g81
  convert_cycle_g82
  convert_cycle_g83
  convert_cycle_g74_g84
  convert_cycle_g85
  convert_cycle_g86
  convert_cycle_g87
  convert_cycle_g88
  convert_cycle_g89

This writes a STRAIGHT_FEED command appropriate for a cycle move with
respect to the given plane. No rotary axis motion takes place.

*/

int Interp::cycle_feed(block_pointer block,
                       CANON_PLANE plane,        //!< currently selected plane
                       double end1,      //!< first coordinate value
                       double end2,      //!< second coordinate value
                       double end3)      //!< third coordinate value
{
    if (plane == CANON_PLANE::XY)
        STRAIGHT_FEED(block->line_number, end1, end2, end3,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::YZ)
        STRAIGHT_FEED(block->line_number, end3, end1, end2,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::XZ)
        STRAIGHT_FEED(block->line_number, end2, end3, end1,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::UV)
        STRAIGHT_FEED(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      end1, end2, end3);
    else if (plane == CANON_PLANE::VW)
        STRAIGHT_FEED(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      end3, end1, end2);
    else // (plane == CANON_PLANE::UW)
        STRAIGHT_FEED(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                      _setup.AA_current, _setup.BB_current, _setup.CC_current,
                      end2, end3, end1);
    return INTERP_OK;
}

/****************************************************************************/

/*! cycle_traverse

Returned Value: int (INTERP_OK)

Side effects:
  STRAIGHT_TRAVERSE is called.

Called by:
  convert_cycle
  convert_cycle_g81
  convert_cycle_g82
  convert_cycle_g83
  convert_cycle_g86
  convert_cycle_g87
  convert_cycle_xy (via CYCLE_MACRO)
  convert_cycle_yz (via CYCLE_MACRO)
  convert_cycle_zx (via CYCLE_MACRO)

This writes a STRAIGHT_TRAVERSE command appropriate for a cycle
move with respect to the given plane. No rotary axis motion takes place.

*/

int Interp::cycle_traverse(block_pointer block,
                           CANON_PLANE plane,    //!< currently selected plane
                           double end1,  //!< first coordinate value
                           double end2,  //!< second coordinate value
                           double end3)  //!< third coordinate value
{

    if (plane == CANON_PLANE::XY)
        STRAIGHT_TRAVERSE(block->line_number, end1, end2, end3,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::YZ)
        STRAIGHT_TRAVERSE(block->line_number, end3, end1, end2,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::XZ)
        STRAIGHT_TRAVERSE(block->line_number, end2, end3, end1,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          _setup.u_current, _setup.v_current, _setup.w_current);
    else if (plane == CANON_PLANE::UV)
        STRAIGHT_TRAVERSE(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          end1, end2, end3);
    else if (plane == CANON_PLANE::VW)
        STRAIGHT_TRAVERSE(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          end3, end1, end2);
    else // (plane == CANON_PLANE::UW)
        STRAIGHT_TRAVERSE(block->line_number, _setup.current_x, _setup.current_y, _setup.current_z,
                          _setup.AA_current, _setup.BB_current, _setup.CC_current,
                          end2, end3, end1);
    return INTERP_OK;
}
