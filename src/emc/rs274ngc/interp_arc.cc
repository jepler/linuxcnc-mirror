/********************************************************************
* Description: interp_arc.cc
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <libintl.h>
#include <rtapi_math.h>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "rs274ngc_interp.hh"
#include "interp_internal.hh"

#define _(s) gettext(s)

char Interp::arc_axis1(CANON_PLANE plane) {
    switch(plane) {
    case CANON_PLANE::XY: return 'X';
    case CANON_PLANE::XZ: return 'Z';
    case CANON_PLANE::YZ: return 'Y';
    default: return '!';
    }
}

char Interp::arc_axis2(CANON_PLANE plane) {
    switch(plane) {
    case CANON_PLANE::XY: return 'Y';
    case CANON_PLANE::XZ: return 'X';
    case CANON_PLANE::YZ: return 'Z';
    default: return '!';
    }
}


/***********************************************************************/

/*! arc_data_comp_ijk

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The two calculable values of the radius differ by more than
      tolerance: NCE_RADIUS_TO_END_OF_ARC_DIFFERS_FROM_RADIUS_TO_START
   2. move is not G_2 or G_3: NCE_BUG_CODE_NOT_G2_OR_G3

Side effects:
   This finds and sets the values of center_x, center_y, and turn.

Called by: convert_arc_comp1

This finds the center coordinates and number of full or partial turns
counterclockwise of a helical or circular arc in ijk-format in the XY
plane. The center is computed easily from the current point and center
offsets, which are given. It is checked that the end point lies one
tool radius from the arc.


*/

int Interp::arc_data_comp_ijk(int move,  //!<either G_2 (cw arc) or G_3 (ccw arc)
                             CANON_PLANE plane, //!<active plane
                             CUTTER_COMP side,  //!<either RIGHT or LEFT
                             double tool_radius,        //!<radius of the tool
                             double current_x,  //!<first coordinate of current point
                             double current_y,  //!<second coordinate of current point
                             double end_x,      //!<first coordinate of arc end point
                             double end_y,      //!<second coordinate of arc end point
                             int ij_absolute,   //!<how to interpret i/j numbers
                             double i_number,   //!<first coordinate of center (abs or incr)
                             double j_number,   //!<second coordinate of center (abs or incr)
                             int p_number,
                             double *center_x,  //!<pointer to first coordinate of center of arc
                             double *center_y,  //!<pointer to second coordinate of center of arc
                             int *turn, //!<pointer to number of full or partial circles CCW
                             double radius_tolerance, //!<minimum radius tolerance
                             double spiral_abs_tolerance,  //!<tolerance of start and end radius difference
                             double spiral_rel_tolerance)
{
  double arc_radius;
  double radius2;
  char a = arc_axis1(plane), b = arc_axis2(plane);

  if ( ij_absolute ) {
    *center_x = (i_number);
    *center_y = (j_number);
  } else {
    *center_x = (current_x + i_number);
    *center_y = (current_y + j_number);
  }
  arc_radius = hypot((*center_x - current_x), (*center_y - current_y));
  radius2 = hypot((*center_x - end_x), (*center_y - end_y));
  CHKS(((arc_radius < radius_tolerance) || (radius2 < radius_tolerance)),
          _("Zero-radius arc: "
       "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) r1=%.4f r2=%.4f"),
       a, current_x, b, current_y,
       a, *center_x, b, *center_y,
       a, end_x, b, end_y, arc_radius, radius2);

  double abs_err = fabs(arc_radius - radius2);
  double rel_err = abs_err / std::max(arc_radius, radius2);

  CHKS((abs_err > spiral_abs_tolerance * 100.0) ||
          (rel_err > spiral_rel_tolerance && abs_err > spiral_abs_tolerance),
      _("Radius to end of arc differs from radius to start: "
       "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) "
       "r1=%.4f r2=%.4f abs_err=%.4g rel_err=%.4f%%"),
       a, current_x, b, current_y, 
       a, *center_x, b, *center_y, 
       a, end_x, b, end_y, arc_radius, radius2,
       abs_err, rel_err*100);

  CHKS(((arc_radius <= tool_radius) && (((side == CUTTER_COMP::LEFT) && (move == G_3)) ||
                                       ((side == CUTTER_COMP::RIGHT) && (move == G_2)))),
      NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);

  /* This catches an arc too small for the tool, also */
  if (move == G_2)
    *turn = -1 * p_number;
  else if (move == G_3)
    *turn = 1 * p_number;
  else
    ERS(NCE_BUG_CODE_NOT_G2_OR_G3);
  return INTERP_OK;
}

/****************************************************************************/

/*! arc_data_comp_r

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The arc radius is too small to reach the end point:
      NCE_RADIUS_TOO_SMALL_TO_REACH_END_POINT
   2. The arc radius is not greater than the tool_radius, but should be:
      NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP
   3. An imaginary value for offset would be found, which should never
      happen if the theory is correct: NCE_BUG_IN_TOOL_RADIUS_COMP

Side effects:
   This finds and sets the values of center_x, center_y, and turn.

Called by: convert_arc_comp1

This finds the center coordinates and number of full or partial turns
counterclockwise of a helical or circular arc (call it arc1) in
r-format in the XY plane.  Arc2 is constructed so that it is tangent
to a circle whose radius is tool_radius and whose center is at the
point (current_x, current_y) and passes through the point (end_x,
end_y). Arc1 has the same center as arc2. The radius of arc1 is one
tool radius larger or smaller than the radius of arc2.

If the value of the big_radius argument is negative, that means [NCMS,
page 21] that an arc larger than a semicircle is to be made.
Otherwise, an arc of a semicircle or less is made.

The algorithm implemented here is to construct a line L from the
current point to the end point, and a perpendicular to it from the
center of the arc which intersects L at point P. Since the distance
from the end point to the center and the distance from the current
point to the center are known, two equations for the length of the
perpendicular can be written. The right sides of the equations can be
set equal to one another and the resulting equation solved for the
length of the line from the current point to P. Then the location of
P, the length of the perpendicular, the angle of the perpendicular,
and the location of the center, can be found in turn.

This needs to be better documented, with figures. There are eight
possible arcs, since there are three binary possibilities: (1) tool
inside or outside arc, (2) clockwise or counterclockwise (3) two
positions for each arc (of the given radius) tangent to the tool
outline and through the end point. All eight are calculated below,
since theta, radius2, and turn may each have two values.

To see two positions for each arc, imagine the arc is a hoop, the
tool is a cylindrical pin, and the arc may rotate around the end point.
The rotation covers all possible positions of the arc. It is easy to
see the hoop is constrained by the pin at two different angles, whether
the pin is inside or outside the hoop.

*/

int Interp::arc_data_comp_r(int move,    //!< either G_2 (cw arc) or G_3 (ccw arc)
                           CANON_PLANE plane,
                           CUTTER_COMP side,    //!< either RIGHT or LEFT
                           double tool_radius,  //!< radius of the tool
                           double current_x,    //!< first coordinate of current point
                           double current_y,    //!< second coordinate of current point
                           double end_x,        //!< first coordinate of arc end point
                           double end_y,        //!< second coordinate of arc end point
                           double big_radius,   //!< radius of arc
                           int p_number,
                           double *center_x,    //!< pointer to first coordinate of center of arc
                           double *center_y,    //!< pointer to second coordinate of center of arc
                           int *turn,           //!< pointer to number of full or partial circles CCW
                           double tolerance)    //!< tolerance of differing radii
{
  double abs_radius;            // absolute value of big_radius

  abs_radius = fabs(big_radius);
  CHKS(((abs_radius <= tool_radius) && (((side == CUTTER_COMP::LEFT) && (move == G_3)) ||
                                       ((side == CUTTER_COMP::RIGHT) && (move == G_2)))),
      NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);

  return arc_data_r(move, plane, current_x, current_y, end_x, end_y, big_radius, p_number,
             center_x, center_y, turn, tolerance);

}

/****************************************************************************/

/*! arc_data_ijk

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The two calculable values of the radius differ by more than
      tolerance: NCE_RADIUS_TO_END_OF_ARC_DIFFERS_FROM_RADIUS_TO_START
   2. The move code is not G_2 or G_3: NCE_BUG_CODE_NOT_G2_OR_G3
   3. Either of the two calculable values of the radius is zero:
      NCE_ZERO_RADIUS_ARC

Side effects:
   This finds and sets the values of center_x, center_y, and turn.

Called by:
   convert_arc2
   convert_arc_comp2

This finds the center coordinates and number of full or partial turns
counterclockwise of a helical or circular arc in ijk-format. This
function is used by convert_arc2 for all three planes, so "x" and
"y" really mean "first_coordinate" and "second_coordinate" wherever
they are used here as suffixes of variable names. The i and j prefixes
are handled similarly.

*/

int Interp::arc_data_ijk(int move,       //!< either G_2 (cw arc) or G_3 (ccw arc)
                        CANON_PLANE plane,
                        double current_x,       //!< first coordinate of current point
                        double current_y,       //!< second coordinate of current point
                        double end_x,   //!< first coordinate of arc end point
                        double end_y,   //!< second coordinate of arc end point
                        int ij_absolute,        //!<how to interpret i/j numbers
                        double i_number,        //!<first coordinate of center (abs or incr)
                        double j_number,        //!<second coordinate of center (abs or incr)
                        int p_number,
                        double *center_x,       //!< pointer to first coordinate of center of arc
                        double *center_y,       //!< pointer to second coordinate of center of arc
                        int *turn,      //!< pointer to no. of full or partial circles CCW
                        double radius_tolerance, //!<minimum radius tolerance
                        double spiral_abs_tolerance,  //!<tolerance of start and end radius difference
                        double spiral_rel_tolerance)
{
  double radius;                /* radius to current point */
  double radius2;               /* radius to end point     */
  char a = arc_axis1(plane), b = arc_axis2(plane);

  if ( ij_absolute ) {
    *center_x = (i_number);
    *center_y = (j_number);
  } else {
    *center_x = (current_x + i_number);
    *center_y = (current_y + j_number);
  }
  radius = hypot((*center_x - current_x), (*center_y - current_y));
  radius2 = hypot((*center_x - end_x), (*center_y - end_y));
  CHKS(((radius < radius_tolerance) || (radius2 < radius_tolerance)),_("Zero-radius arc: "
       "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) r1=%.4f r2=%.4f"),
       a, current_x, b, current_y,
       a, *center_x, b, *center_y,
       a, end_x, b, end_y, radius, radius2);
  double abs_err = fabs(radius - radius2);
  double rel_err = abs_err / std::max(radius, radius2);
  CHKS((abs_err > spiral_abs_tolerance * 100.0) ||
          (rel_err > spiral_rel_tolerance && abs_err > spiral_abs_tolerance),
      _("Radius to end of arc differs from radius to start: "
       "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) "
       "r1=%.4f r2=%.4f abs_err=%.4g rel_err=%.4f%%"),
       a, current_x, b, current_y, 
       a, *center_x, b, *center_y, 
       a, end_x, b, end_y, radius, radius2,
       abs_err, rel_err*100);

  if (move == G_2)
    *turn = -1 * p_number;
  else if (move == G_3)
    *turn = 1 * p_number;
  else
    ERS(NCE_BUG_CODE_NOT_G2_OR_G3);
  return INTERP_OK;
}

/****************************************************************************/

/*! arc_data_r

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The radius is too small to reach the end point:
      NCE_ARC_RADIUS_TOO_SMALL_TO_REACH_END_POINT
   2. The current point is the same as the end point of the arc
      (so that it is not possible to locate the center of the circle):
      NCE_CURRENT_POINT_SAME_AS_END_POINT_OF_ARC

Side effects:
   This finds and sets the values of center_x, center_y, and turn.

Called by:
   convert_arc2
   convert_arc_comp2

This finds the center coordinates and number of full or partial turns
counterclockwise of a helical or circular arc in the r format. This
function is used by convert_arc2 for all three planes, so "x" and
"y" really mean "first_coordinate" and "second_coordinate" wherever
they are used here as suffixes of variable names.

If the value of the radius argument is negative, that means [NCMS,
page 21] that an arc larger than a semicircle is to be made.
Otherwise, an arc of a semicircle or less is made.

The algorithm used here is based on finding the midpoint M of the line
L between the current point and the end point of the arc. The center
of the arc lies on a line through M perpendicular to L.

*/

int Interp::arc_data_r(int move, //!< either G_2 (cw arc) or G_3 (ccw arc)
                      CANON_PLANE /*plane*/,
                      double current_x, //!< first coordinate of current point
                      double current_y, //!< second coordinate of current point
                      double end_x,     //!< first coordinate of arc end point
                      double end_y,     //!< second coordinate of arc end point
                      double radius,    //!< radius of arc
                      int p_number,
                      double *center_x, //!< pointer to first coordinate of center of arc
                      double *center_y, //!< pointer to second coordinate of center of arc
                      int *turn,        //!< pointer to number of full or partial circles CCW
                      double tolerance) //!< tolerance of differing radii
{
  double abs_radius;            /* absolute value of given radius */
  double half_length;           /* distance from M to end point   */
  double mid_x;                 /* first coordinate of M          */
  double mid_y;                 /* second coordinate of M         */
  double offset;                /* distance from M to center      */
  double theta;                 /* angle of line from M to center */
  double turn2;                 /* absolute value of half of turn */

  CHKS(((end_x == current_x) && (end_y == current_y)),
      NCE_CURRENT_POINT_SAME_AS_END_POINT_OF_ARC);
  abs_radius = fabs(radius);
  mid_x = (end_x + current_x) / 2.0;
  mid_y = (end_y + current_y) / 2.0;
  half_length = hypot((mid_x - end_x), (mid_y - end_y));
  CHKS(((half_length - abs_radius) > tolerance),
      NCE_ARC_RADIUS_TOO_SMALL_TO_REACH_END_POINT);
  if ((half_length / abs_radius) > (1 - TINY))
    half_length = abs_radius;   /* allow a small error for semicircle */
  /* check needed before calling asin   */
  if (((move == G_2) && (radius > 0)) || ((move == G_3) && (radius < 0)))
    theta = atan2((end_y - current_y), (end_x - current_x)) - M_PI_2l;
  else
    theta = atan2((end_y - current_y), (end_x - current_x)) + M_PI_2l;

  turn2 = asin(half_length / abs_radius);
  offset = abs_radius * cos(turn2);
  *center_x = mid_x + (offset * cos(theta));
  *center_y = mid_y + (offset * sin(theta));
  *turn = (move == G_2) ? -1 * p_number : 1 * p_number;

  return INTERP_OK;
}
