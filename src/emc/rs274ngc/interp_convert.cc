/* misnomer: settings->current_pocket,selected_pocket
** These variables are actually indexes to sequential tool
** data structs (not real pockets).
** Future renaming will affect current usage in python remaps.
*/

/********************************************************************
* Description: interp_convert.cc
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
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
#include <string>
#include "rtapi_math.h"
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "rs274ngc_interp.hh"
#include "interp_internal.hh"
#include "interp_queue.hh"
#include "interp_parameter_def.hh"
#include <rtapi_string.h>

#include "units.h"
#define TOOL_INSIDE_ARC(side, turn) (((side)==CUTTER_COMP::LEFT&&(turn)>0)||((side)==CUTTER_COMP::RIGHT&&(turn)<0))
#define DEBUG_EMC

using namespace interp_param_global;

// These four functions help make the rest of cutter comp
// plane-agnostic in much the same way the ARC_FEED canon call is.
// The programmer can gleefully think of only the XY plane when
// reading convert_[straight|arc]_comp[1|2].
//
// Because the STRAIGHT_[FEED|TRAVERSE] canon calls are not
// plane-agnostic, the opposite plane conversion happens in
// enqueue_STRAIGHT_[FEED|TRAVERSE] when adding to the interp queue.

int Interp::comp_get_current(setup_pointer settings, double *x, double *y, double *z) {
    switch(settings->plane) {
    case CANON_PLANE::XY:
        *x = settings->current_x;
        *y = settings->current_y;
        *z = settings->current_z;
        break;
    case CANON_PLANE::XZ:
        *x = settings->current_z;
        *y = settings->current_x;
        *z = settings->current_y;
        break;
    default:
        ERS("BUG: Invalid plane in comp_get_current");
    }
    return INTERP_OK;
}

int Interp::comp_set_current(setup_pointer settings, double x, double y, double z) {
    switch(settings->plane) {
    case CANON_PLANE::XY:
        settings->current_x = x;
        settings->current_y = y;
        settings->current_z = z;
        break;
    case CANON_PLANE::XZ:
        settings->current_x = y;
        settings->current_y = z;
        settings->current_z = x;
        break;
    default:
        ERS("BUG: Invalid plane in comp_set_current");
    }
    return INTERP_OK;
}

int Interp::comp_get_programmed(setup_pointer settings, double *x, double *y, double *z) {
    switch(settings->plane) {
    case CANON_PLANE::XY:
        *x = settings->program_x;
        *y = settings->program_y;
        *z = settings->program_z;
        break;
    case CANON_PLANE::XZ:
        *x = settings->program_z;
        *y = settings->program_x;
        *z = settings->program_y;
        break;
    default:
        ERS("BUG: Invalid plane in comp_get_programmed");
    }
    return INTERP_OK;
}

int Interp::comp_set_programmed(setup_pointer settings, double x, double y, double z) {
    switch(settings->plane) {
    case CANON_PLANE::XY:
        settings->program_x = x;
        settings->program_y = y;
        settings->program_z = z;
        break;
    case CANON_PLANE::XZ:
        settings->program_x = y;
        settings->program_y = z;
        settings->program_z = x;
        break;
    default:
        ERS("BUG: Invalid plane in comp_set_programmed");
    }
    return INTERP_OK;
}

/* Set *result to the integer nearest to value; return TRUE if value is
 * within .0001 of an integer
 */
static int is_near_int(int *result, double value) {
    *result = (int)(value + .5);
    return fabs(*result - value) < .0001;
}

/****************************************************************************/

/*! convert_nurbs
 *
 * Returned value: int
 * Returns a rs274ngc error code, or INTERP_OK if everything is OK.
 *
 * Side effects: Generates a nurbs move and updates the position of the tool
 
Reference for this code is:
Lo Valvo, E., Drago, S. (2014).
An Efficient NURBS Path Generator for an Open Source CNC. 
Recent Advances in Mechanical Engineering (pp.173-180). WSEAS Press.

Q=3 NICL NURBS interpolation with linear motion 
Q=2 NICC NURBS interpolation with circular motion
Q=1 NICU NURBS interpolation with biarch, du=const
*/
static unsigned int Q_G6_option = 0;	// (NICU, NICL, NICC see publication from Lo Valvo and Drago)
static unsigned int nurbs_order;
static std::vector < NURBS_CONTROL_POINT > nurbs_g5_control_points;
static std::vector < NURBS_G6_CONTROL_POINT > nurbs_g6_control_points;
int ff_G6_line_counter = 0;	//inizializzo il contatore

void Interp::nurbs_reset_global_variables(void)
{				// called from open gcode file and Interp::convert_stop
	Q_G6_option = 0;
	ff_G6_line_counter = 0;
	if (!nurbs_g5_control_points.empty()) {
		nurbs_g5_control_points.clear();
	}
	if (!nurbs_g6_control_points.empty()) {
		nurbs_g6_control_points.clear();
	}
}

int Interp::convert_nurbs(int mode,
			  block_pointer block, setup_pointer settings)
{

	double end_x, end_y, end_z, AA_end, BB_end, CC_end, u_end, v_end,
	    w_end;
	NURBS_CONTROL_POINT CP_G5;
	NURBS_G6_CONTROL_POINT CP_G6;

	if (mode == G_5_2) {

		if (settings->plane == CANON_PLANE::XY) {
			CHKS((((block->x_flag) && !(block->y_flag))
			      || (!(block->x_flag)
				  && (block->y_flag))),
			     (_
			      ("You must specify both X and Y coordinates for Control Points")));
			CHKS((!(block->x_flag) && !(block->y_flag)
			      && (block->p_number > 0)
			      && (!nurbs_g5_control_points.empty())),
			     (_
			      ("Can specify P without X and Y only for the first control point")));
		}
		if (settings->plane == CANON_PLANE::YZ) {
			CHKS((((block->y_flag) && !(block->z_flag))
			      || (!(block->y_flag)
				  && (block->z_flag))),
			     (_
			      ("You must specify both Y and Z coordinates for Control Points")));
			CHKS((!(block->y_flag) && !(block->z_flag)
			      && (block->p_number > 0)
			      && (!nurbs_g5_control_points.empty())),
			     (_
			      ("Can specify P without Y and Z only for the first control point")));
		}
		if (settings->plane == CANON_PLANE::XZ) {
			CHKS((((block->x_flag) && !(block->z_flag))
			      || (!(block->x_flag)
				  && (block->z_flag))),
			     (_
			      ("You must specify both X and Z coordinates for Control Points")));
			CHKS((!(block->x_flag) && !(block->z_flag)
			      && (block->p_number > 0)
			      && (!nurbs_g5_control_points.empty())),
			     (_
			      ("Can specify P without X and Z only for the first control point")));
		}

		CHKS(((block->p_number <= 0)
		      && (!nurbs_g5_control_points.empty())),
		     (_
		      ("Must specify positive weight P for every Control Point")));
		if (settings->feed_mode == FEED_MODE::UNITS_PER_MINUTE) {
			CHKS((settings->feed_rate == 0.0),
			     (_("Cannot make a NURBS with 0 feedrate")));
		}
		if (settings->motion_mode != mode)
			nurbs_g5_control_points.clear();

		if (nurbs_g5_control_points.empty()) {
			if (settings->plane == CANON_PLANE::XY) {
				CP_G5.NURBS_X = settings->current_x;
				CP_G5.NURBS_Y = settings->current_y;
				if (!(block->x_flag) && !(block->y_flag)
				    && (block->p_number > 0)) {
					CP_G5.NURBS_W = block->p_number;
				} else {
					CP_G5.NURBS_W = 1;
				}
			}
			if (settings->plane == CANON_PLANE::YZ) {
				CP_G5.NURBS_X = settings->current_y;
				CP_G5.NURBS_Y = settings->current_z;
				if (!(block->y_flag) && !(block->z_flag)
				    && (block->p_number > 0)) {
					CP_G5.NURBS_W = block->p_number;
				} else {
					CP_G5.NURBS_W = 1;
				}
			}
			if (settings->plane == CANON_PLANE::XZ) {
				CP_G5.NURBS_X = settings->current_z;
				CP_G5.NURBS_Y = settings->current_x;
				if (!(block->x_flag) && !(block->z_flag)
				    && (block->p_number > 0)) {
					CP_G5.NURBS_W = block->p_number;
				} else {
					CP_G5.NURBS_W = 1;
				}
			}
			nurbs_order = 3;
			nurbs_g5_control_points.push_back(CP_G5);
		}
		if (block->l_number != -1 && block->l_number > 3) {
			nurbs_order = block->l_number;
		}

		if (settings->plane == CANON_PLANE::XY) {
			if ((block->x_flag) && (block->y_flag)) {
				CHP(find_ends
				    (block, settings, &CP_G5.NURBS_X,
				     &CP_G5.NURBS_Y, &end_z, &AA_end,
				     &BB_end, &CC_end, &u_end, &v_end,
				     &w_end));
				CP_G5.NURBS_W = block->p_number;
				nurbs_g5_control_points.push_back(CP_G5);
			}
		}
		if (settings->plane == CANON_PLANE::YZ) {
			if ((block->y_flag) && (block->z_flag)) {
				CHP(find_ends
				    (block, settings, &end_x,
				     &CP_G5.NURBS_X, &CP_G5.NURBS_Y,
				     &AA_end, &BB_end, &CC_end, &u_end,
				     &v_end, &w_end));
				CP_G5.NURBS_W = block->p_number;
				nurbs_g5_control_points.push_back(CP_G5);
			}
		}
		if (settings->plane == CANON_PLANE::XZ) {
			if ((block->x_flag) && (block->z_flag)) {
				CHP(find_ends
				    (block, settings, &CP_G5.NURBS_Y,
				     &end_y, &CP_G5.NURBS_X, &AA_end,
				     &BB_end, &CC_end, &u_end, &v_end,
				     &w_end));
				CP_G5.NURBS_W = block->p_number;
				nurbs_g5_control_points.push_back(CP_G5);
			}
		}

/*
	for (long unsigned int i=0;i<nurbs_control_points.size();i++) {
		//printf( "X %8.4f, Y %8.4f, Z %8.4f W %8.4f\n",
		printf( "X: %8.4f, Y: %8.4f W: %8.4f\n",
		nurbs_control_points[i].NURBS_X,
		nurbs_control_points[i].NURBS_Y,
		nurbs_control_points[i].NURBS_W);
	}
	printf("*----------------------------------------- (%s %d)\n", __FILE__,__LINE__);
*/
		settings->motion_mode = mode;
	}

	else if (mode == G_5_3) {
		CHKS((settings->motion_mode != G_5_2),
		     (_("Cannot use G5.3 without G5.2 first")));
		CHKS((nurbs_g5_control_points.size() < nurbs_order),
		     _
		     ("You must specify a number of control points at least equal to the order L = %d"),
		     nurbs_order);

		if (settings->plane == CANON_PLANE::XY) {
			settings->current_x =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_X;
			settings->current_y =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_Y;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			settings->current_y =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_X;
			settings->current_z =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_Y;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			settings->current_z =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_X;
			settings->current_x =
			    nurbs_g5_control_points
			    [nurbs_g5_control_points.size() - 1].NURBS_Y;
		}
/*
		for (long unsigned int i=0;i<nurbs_control_points.size();i++) 
			{
			printf( "X: %8.4f, Y: %8.4f W: %8.4f\n",
			nurbs_g5_control_points[i].NURBS_X,
			nurbs_g5_control_points[i].NURBS_Y,
			nurbs_g5_control_points[i].W);
			}
		printf("*----------------------------------------- hier sind alle nurbs-Punkte da (%s %d)\n", __FILE__,__LINE__);
*/
		NURBS_G5_FEED(block->line_number, nurbs_g5_control_points,
			      nurbs_order, settings->plane);
		nurbs_g5_control_points.clear();
		settings->motion_mode = -1;
	}
//
	if (mode == G_6_2) {
		// jjf additional for Q (as replacement for #1 in the original code from Lo Valvo)
		// Q=3 NICL NURBS interpolation with linear motion 
		// Q=2 NICC NURBS interpolation with circular motion
		// Q=1 NICU NURBS interpolation with biarch, du=const

		CHKS(((settings->plane != CANON_PLANE::XY)
		      && (settings->plane != CANON_PLANE::YZ)
		      && (settings->plane != CANON_PLANE::XZ)),
		     (_
		      ("one plane must be selected for nurbs: XY, YZ, ZX")));

		if (settings->plane == CANON_PLANE::XY) {
			CHKS((((block->x_flag) && !(block->y_flag))
			      || (!(block->x_flag)
				  && (block->y_flag))),
			     (_
			      ("You must specify both X and Y coordinates for Control Points")));
		}
		if (settings->plane == CANON_PLANE::YZ) {
			CHKS((((block->y_flag) && !(block->z_flag))
			      || (!(block->y_flag)
				  && (block->z_flag))),
			     (_
			      ("You must specify both Y and Z coordinates for Control Points")));
		}
		if (settings->plane == CANON_PLANE::XZ) {
			CHKS((((block->x_flag) && !(block->z_flag))
			      || (!(block->x_flag)
				  && (block->z_flag))),
			     (_
			      ("You must specify both X and Z coordinates for Control Points")));
		}

		CHKS(((!block->p_flag)
		      && (nurbs_g6_control_points.empty())),
		     (_
		      ("Program the nurbs order P in the first block of instruction")));
		int p;
		if (block->p_flag) {
			CHKS((!is_near_int(&p, block->p_number)),
			     _("nurbs order P number is not an integer"));
		}

		CHKS(((block->q_number <= 0)
		      && (nurbs_g6_control_points.empty())),
		     (_
		      ("Program the nurbs Q value (1,2,3) in the first block of nurbs instruction")));
		int q;
		if (block->q_flag) {
			CHKS((block->q_number < 1.0)
			     || (block->q_number > 3.0),
			     (_("nurbs Q value not in range 1 to 3")));
			CHKS((!is_near_int(&q, block->q_number)),
			     _("nurbs Q number is not an integer"));
		}

		CHKS(((block->r_number <= 0)
		      && (!nurbs_g6_control_points.empty())),
		     (_
		      ("Must specify positive nurbs weight R for every Control Point")));

		CHKS(((block->k_number < 0)
		      && (!nurbs_g6_control_points.empty())),
		     (_
		      ("Must specify nurbs K number for every Control Point")));
		int k;
		if (block->k_flag) {
			CHKS((!is_near_int(&k, block->k_number)),
			     _("nurbs K number is not an integer"));
		}

		if (settings->feed_mode == FEED_MODE::UNITS_PER_MINUTE) {
			CHKS((settings->feed_rate == 0.0),
			     (_("Cannot make a nurbs with 0 feedrate")));
		}

		if (block->p_number != -1) {
			nurbs_order = block->p_number;
		}

		if (block->k_flag) {
			CP_G6.NURBS_K = block->k_number;
		}
		// PER DEFINIRE UNO FRA I TRE MODI DI INTERPOLARE (NURBS)
		// EINE DER DREI WEGE VON INTERPOLATION (NURBS) AUSWAEHLERN
		// TO DEFINE ONE OF THE THREE WAYS OF INTERPOLAR (NURBS)
		if (ff_G6_line_counter == 0) {
			Q_G6_option = block->q_number;	// Q_G6_option=1...3 (NICU, NICL, NICC see publication from Lo Valvo and Drago)
		}

		if (settings->plane == CANON_PLANE::XY) {
			if (((block->x_flag) && (block->y_flag))
			    || block->k_flag) {
				CHP(find_ends
				    (block, settings, &CP_G6.NURBS_X,
				     &CP_G6.NURBS_Y, &end_z, &AA_end,
				     &BB_end, &CC_end, &u_end, &v_end,
				     &w_end));
				CP_G6.NURBS_R = block->r_number;
				nurbs_g6_control_points.push_back(CP_G6);
			}
		}
		if (settings->plane == CANON_PLANE::YZ) {
			if (((block->y_flag) && (block->z_flag))
			    || block->k_flag) {
				CHP(find_ends
				    (block, settings, &end_x,
				     &CP_G6.NURBS_X, &CP_G6.NURBS_Y,
				     &AA_end, &BB_end, &CC_end, &u_end,
				     &v_end, &w_end));
				CP_G6.NURBS_R = block->r_number;
				nurbs_g6_control_points.push_back(CP_G6);
			}
		}
		if (settings->plane == CANON_PLANE::XZ) {
			if (((block->x_flag) && (block->z_flag))
			    || block->k_flag) {
				CHP(find_ends
				    (block, settings, &CP_G6.NURBS_Y,
				     &end_z, &CP_G6.NURBS_X, &AA_end,
				     &BB_end, &CC_end, &u_end, &v_end,
				     &w_end));
				CP_G6.NURBS_R = block->r_number;
				nurbs_g6_control_points.push_back(CP_G6);
			}
		}

		settings->motion_mode = mode;

		ff_G6_line_counter++;
		if (nurbs_g6_control_points.size() > nurbs_order) {
			if (nurbs_g6_control_points
			    [ff_G6_line_counter - 1].NURBS_K ==
			    nurbs_g6_control_points[ff_G6_line_counter -
						    nurbs_order + 1 -
						    1].NURBS_K) {
				CHKS((nurbs_g6_control_points.size() <
				      nurbs_order),
				     _
				     ("You must specify a number of control points at least equal to the order P = %d"),
				     nurbs_order);
				settings->current_x =
				    nurbs_g6_control_points
				    [nurbs_g6_control_points.size() -
				     1].NURBS_X;
				settings->current_y =
				    nurbs_g6_control_points
				    [nurbs_g6_control_points.size() -
				     1].NURBS_Y;
/*
				for (long unsigned int i=0;i<nurbs_g6_control_points.size();i++) 
					{
					printf("i: %3ld X: %8.4f Y: %8.4f R: %8.4f K: %8.4f \n",
					i,
					nurbs_g6_control_points[i].NURBS_X,
					nurbs_g6_control_points[i].NURBS_Y,
					nurbs_g6_control_points[i].NURBS_R,
					nurbs_g6_control_points[i].NURBS_K
					);
				}
				printf("*----------------------------------------- (%s %d)\n", __FILE__,__LINE__);
*/
				NURBS_G6_FEED(block->line_number,
					      nurbs_g6_control_points,
					      nurbs_order,
					      settings->feed_rate,
					      Q_G6_option,
					      settings->plane);
				nurbs_g6_control_points.clear();
				ff_G6_line_counter = 0;
				settings->motion_mode = -1;
			}
		}
	}
//
	if (mode == G_6_3)	// jjf this is missing from Lo Valvo code! not used there and here!
	{
		printf("mode == G_6_3 not used (%s %d)\n", __FILE__,
		       __LINE__);
	}
	return INTERP_OK;
}

 /****************************************************************************/
 /*! convert_spline
  * 
  * Returned value: int
  * Returns a rs274ngc error code, or INTERP_OK if everything is OK.
  *
  * Side effects: Generates a spline move and updates the position of the tool
  */
int Interp::convert_spline(int mode, block_pointer block,	//!< pointer to a block of RS274 instructions
			   setup_pointer settings)
{				//!< pointer to machine settings
	double x1, y1, z1, x2, y2, z2, x3, y3, z3;
	double end_x, end_y, end_z, AA_end, BB_end, CC_end, u_end, v_end,
	    w_end;
	NURBS_CONTROL_POINT cp;

	x1 = 0;
	y1 = 0;
	z1 = 0;			//to avoid compiler warning "may be used uninitialized" 

	CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF), _("Cannot convert spline with cutter radius compensation"));	// XXX

	if (settings->feed_mode == FEED_MODE::UNITS_PER_MINUTE) {
		CHKS((settings->feed_rate == 0.0),
		     NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
	} else if (settings->feed_mode == FEED_MODE::INVERSE_TIME) {
		CHKS((!block->f_flag),
		     NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE);
	}

	CHKS((block->a_flag || block->b_flag
	      || block->c_flag),
	     _("Splines may not have motion in A, B, or C"));

	if ((mode == G_5_1) || (mode == G_6_1)) {
		printf("mode == G_5_1 or G_6_1 (%s %d)\n", __FILE__,
		       __LINE__);
		CHKS(!block->i_flag
		     || !block->j_flag,
		     _("Must specify both I and J with G5.1 or G6.1"));

		if (settings->plane == CANON_PLANE::XY) {
			x1 = settings->current_x + block->i_number;
			y1 = settings->current_y + block->j_number;
			CHP(find_ends
			    (block, settings, &x2, &y2, &end_z, &AA_end,
			     &BB_end, &CC_end, &u_end, &v_end, &w_end));
			printf("find_ends x2: %8.4f y2: %8.4f (%s %d)\n",
			       x2, y2, __FILE__, __LINE__);
		}
		if (settings->plane == CANON_PLANE::YZ) {
			y1 = settings->current_y + block->i_number;
			z1 = settings->current_z + block->j_number;
			CHP(find_ends
			    (block, settings, &end_x, &y2, &z2, &AA_end,
			     &BB_end, &CC_end, &u_end, &v_end, &w_end));
			printf("find_ends y2: %8.4f z2: %8.4f (%s %d)\n",
			       y2, z2, __FILE__, __LINE__);
		}
		if (settings->plane == CANON_PLANE::XZ) {
			x1 = settings->current_x + block->i_number;
			z1 = settings->current_z + block->j_number;
			CHP(find_ends
			    (block, settings, &x2, &end_y, &z2, &AA_end,
			     &BB_end, &CC_end, &u_end, &v_end, &w_end));
			printf("find_ends x2: %8.4f z2: %8.4f (%s %d)\n",
			       x2, z2, __FILE__, __LINE__);
		}

		cp.NURBS_W = 1;

		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = settings->current_x;
			cp.NURBS_Y = settings->current_y;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = settings->current_y;
			cp.NURBS_Y = settings->current_z;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = settings->current_z;
			cp.NURBS_Y = settings->current_x;
		}
		nurbs_g5_control_points.push_back(cp);

		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = x1;
			cp.NURBS_Y = y1;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = y1;
			cp.NURBS_Y = z1;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = z1;
			cp.NURBS_Y = x1;
		}
		nurbs_g5_control_points.push_back(cp);

		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = x2;
			cp.NURBS_Y = y2;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = y2;
			cp.NURBS_Y = z2;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = z2;
			cp.NURBS_Y = x2;
		}
		nurbs_g5_control_points.push_back(cp);

		for (long unsigned int i = 0;
		     i < nurbs_g5_control_points.size(); i++) {
			printf("X %8.4f, Y %8.4f W %8.4f\n",
			       nurbs_g5_control_points[i].NURBS_X,
			       nurbs_g5_control_points[i].NURBS_Y,
			       nurbs_g5_control_points[i].NURBS_W);
		}
		printf
		    ("*----------------------------------------- (%s %d)\n",
		     __FILE__, __LINE__);

		NURBS_G5_FEED(block->line_number, nurbs_g5_control_points,
			      3, settings->plane);
		nurbs_g5_control_points.clear();

		if (settings->plane == CANON_PLANE::XY) {
			settings->current_x = x2;
			settings->current_y = y2;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			settings->current_y = y2;
			settings->current_z = z2;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			settings->current_x = x2;
			settings->current_z = z2;
		}
	} else {		// !(mode == G_5_1)
		if (!block->i_flag || !block->j_flag) {
			CHKS(block->i_flag
			     || block->j_flag,
			     _("Must specify both I and J, or neither"));
			if (settings->plane == CANON_PLANE::XY) {
				x1 = settings->current_x +
				    settings->cycle_i;
				y1 = settings->current_y +
				    settings->cycle_j;
			}
			if (settings->plane == CANON_PLANE::YZ) {
				y1 = settings->current_y +
				    settings->cycle_i;
				z1 = settings->current_z +
				    settings->cycle_j;
			}
			if (settings->plane == CANON_PLANE::XZ) {
				x1 = settings->current_x +
				    settings->cycle_i;
				z1 = settings->current_z +
				    settings->cycle_j;
			}
		} else {
			if (settings->plane == CANON_PLANE::XY) {
				x1 = settings->current_x + block->i_number;
				y1 = settings->current_y + block->j_number;
				CHP(find_ends
				    (block, settings, &x3, &y3, &end_z,
				     &AA_end, &BB_end, &CC_end, &u_end,
				     &v_end, &w_end));
				printf("x3: %8.4f y3: %8.4f (%s %d)\n", x3,
				       y3, __FILE__, __LINE__);
			}
			if (settings->plane == CANON_PLANE::YZ) {
				y1 = settings->current_y + block->i_number;
				z1 = settings->current_z + block->j_number;
				CHP(find_ends
				    (block, settings, &end_x, &y3, &z3,
				     &AA_end, &BB_end, &CC_end, &u_end,
				     &v_end, &w_end));
				printf("y3: %8.4f z3: %8.4f (%s %d)\n", y3,
				       z3, __FILE__, __LINE__);
			}
			if (settings->plane == CANON_PLANE::XZ) {
				x1 = settings->current_x + block->i_number;
				z1 = settings->current_z + block->j_number;
				CHP(find_ends
				    (block, settings, &x3, &end_y, &z3,
				     &AA_end, &BB_end, &CC_end, &u_end,
				     &v_end, &w_end));
				printf("x3: %8.4f z3: %8.4f (%s %d)\n", x3,
				       z3, __FILE__, __LINE__);
			}
		}

		CHKS(!block->p_flag
		     || !block->q_flag,
		     _("Must specify both P and Q with G5"));
		if (settings->plane == CANON_PLANE::XY) {
			x2 = x3 + block->p_number;
			y2 = y3 + block->q_number;
			printf("x2: %8.4f y2: %8.4f (%s %d)\n", x2, y2,
			       __FILE__, __LINE__);
		}
		if (settings->plane == CANON_PLANE::YZ) {
			y2 = y3 + block->p_number;
			z2 = z3 + block->q_number;
			printf("y2: %8.4f z2: %8.4f (%s %d)\n", y2, z2,
			       __FILE__, __LINE__);
		}
		if (settings->plane == CANON_PLANE::XZ) {
			z2 = z3 + block->p_number;
			x2 = x3 + block->q_number;
			printf("x2: %8.4f z2: %8.4f (%s %d)\n", x2, z2,
			       __FILE__, __LINE__);
		}

		cp.NURBS_W = 1;
		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = settings->current_x;
			cp.NURBS_Y = settings->current_y;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = settings->current_y;
			cp.NURBS_Y = settings->current_z;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = settings->current_z;
			cp.NURBS_Y = settings->current_x;
		}
		nurbs_g5_control_points.push_back(cp);
		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = x1;
			cp.NURBS_Y = y1;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = y1;
			cp.NURBS_Y = z1;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = z1;
			cp.NURBS_Y = x1;
		}
		nurbs_g5_control_points.push_back(cp);
		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = x2;
			cp.NURBS_Y = y2;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = y2;
			cp.NURBS_Y = z2;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = z2;
			cp.NURBS_Y = x2;
		}
		nurbs_g5_control_points.push_back(cp);
		if (settings->plane == CANON_PLANE::XY) {
			cp.NURBS_X = x3;
			cp.NURBS_Y = y3;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			cp.NURBS_X = y3;
			cp.NURBS_Y = z3;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			cp.NURBS_X = z3;
			cp.NURBS_Y = x3;
		}
		nurbs_g5_control_points.push_back(cp);

/*        
        for (long unsigned int i=0;i<nurbs_g5_control_points.size();i++)          {
            //printf( "X %8.4f, Y %8.4f, Z %8.4f W %8.4f\n",
            printf( "X: %8.4f Y: %8.4f W: %8.4f\n",
                    nurbs_g5_control_points[i].NURBS_X,
                    nurbs_g5_control_points[i].NURBS_Y,
                    nurbs_g5_control_points[i].NURBS_W);
        }
        printf("*----------------------------------------- (%s %d)\n", __FILE__,__LINE__);
*/
		NURBS_G5_FEED(block->line_number, nurbs_g5_control_points,
			      4, settings->plane);
		nurbs_g5_control_points.clear();

		settings->cycle_i = -block->p_number;
		settings->cycle_j = -block->q_number;
		if (settings->plane == CANON_PLANE::XY) {
			settings->current_x = x3;
			settings->current_y = y3;
		}
		if (settings->plane == CANON_PLANE::YZ) {
			settings->current_y = y3;
			settings->current_z = z3;
		}
		if (settings->plane == CANON_PLANE::XZ) {
			settings->current_x = x3;
			settings->current_z = z3;
		}
	}
	return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc

Returned Value: int
   If one of the following functions returns an error code,
   this returns that error code.
      convert_arc_comp1
      convert_arc_comp2
      convert_arc2
   If any of the following errors occur, this returns the error code shown.
   Otherwise, this returns INTERP_OK.
   1. The block has neither an r value nor any i,j,k values:
      NCE_R_I_J_K_WORDS_ALL_MISSING_FOR_ARC
   2. The block has both an r value and one or more i,j,k values:
      NCE_MIXED_RADIUS_IJK_FORMAT_FOR_ARC
   3. In the ijk format the XY-plane is selected and
      the block has a k value: NCE_K_WORD_GIVEN_FOR_ARC_IN_XY_PLANE
   4. In the ijk format the YZ-plane is selected and
      the block has an i value: NCE_I_WORD_GIVEN_FOR_ARC_IN_YZ_PLANE
   5. In the ijk format the XZ-plane is selected and
      the block has a j value: NCE_J_WORD_GIVEN_FOR_ARC_IN_XZ_PLANE
   6. In either format any of the following occurs.
      a. The XY-plane is selected and the block has no x or y value:
         NCE_X_AND_Y_WORDS_MISSING_FOR_ARC_IN_XY_PLANE
      b. The YZ-plane is selected and the block has no y or z value:
         NCE_Y_AND_Z_WORDS_MISSING_FOR_ARC_IN_YZ_PLANE
      c. The ZX-plane is selected and the block has no z or x value:
         NCE_X_AND_Z_WORDS_MISSING_FOR_ARC_IN_XZ_PLANE
   7. The selected plane is an unknown plane:
      NCE_BUG_PLANE_NOT_XY_YZ__OR_XZ
   8. The feed rate mode is UNITS_PER_MINUTE and feed rate is zero:
      NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE
   9. The feed rate mode is INVERSE_TIME and the block has no f word:
      NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE

Side effects:
   This generates and executes an arc command at feed rate
   (and, possibly a second arc command). It also updates the setting
   of the position of the tool point to the end point of the move.

Called by: convert_motion.

This converts a helical or circular arc.  The function calls:
convert_arc2 (when cutter radius compensation is off) or
convert_arc_comp1 (when cutter comp is on and this is the first move) or
convert_arc_comp2 (when cutter comp is on and this is not the first move).

If the ijk format is used, at least one of the offsets in the current
plane must be given in the block; it is common but not required to
give both offsets. The offsets are always incremental [NCMS, page 21].

If cutter compensation is in use, the path's length may increase or
decrease.  Also an arc may be added, to go around a corner, before the
original arc move.  For the purpose of calculating the feed rate when in
inverse time mode, this length increase or decrease is ignored.  The
feed is still set to the original programmed arc length divided by the F
number (with the above lower bound).  The new arc (if needed) and the
new longer or shorter original arc are taken at this feed.

*/

int Interp::convert_arc(int move,        //!< either G_2 (cw arc) or G_3 (ccw arc)
                       block_pointer block,     //!< pointer to a block of RS274 instructions
                       setup_pointer settings)  //!< pointer to machine settings
{
  int status;
  int first;                    /* flag set true if this is first move after comp true */
  int ijk_flag;                 /* flag set true if any of i,j,k present in NC code  */
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end, v_end, w_end;

  CHKS((settings->arc_not_allowed), (_("The move just after exiting cutter compensation mode must be straight, not an arc")));

  ijk_flag = block->i_flag || block->j_flag || block->k_flag;
  first = settings->cutter_comp_firstmove;

  CHKS((settings->plane == CANON_PLANE::UV
            || settings->plane == CANON_PLANE::VW
            || settings->plane == CANON_PLANE::UW),
    _("Cannot do an arc in planes G17.1, G18.1, or G19.1"));
  CHKS(((!block->r_flag) && (!ijk_flag)),
      NCE_R_I_J_K_WORDS_ALL_MISSING_FOR_ARC);
  CHKS(((block->r_flag) && (ijk_flag)),
      NCE_MIXED_RADIUS_IJK_FORMAT_FOR_ARC);
  if (settings->feed_mode == FEED_MODE::UNITS_PER_MINUTE) {
    CHKS((settings->feed_rate == 0.0),
        NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
  } else if(settings->feed_mode == FEED_MODE::UNITS_PER_REVOLUTION) {
    CHKS((settings->feed_rate == 0.0),
        NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
    CHKS((settings->speed[settings->active_spindle] == 0.0),
	_("Cannot feed with zero spindle speed in feed per rev mode"));
  } else if (settings->feed_mode == FEED_MODE::INVERSE_TIME) {
    CHKS((!block->f_flag),
        NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE);
  }

  if (ijk_flag) {
    if (settings->plane == CANON_PLANE::XY) {
      CHKS((block->k_flag), NCE_K_WORD_GIVEN_FOR_ARC_IN_XY_PLANE);
      if (!block->i_flag) { /* i or j flag on to get here */
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'I');
	} else {
	  block->i_number = 0.0;
	}
      } else if (!block->j_flag) {
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'J');
	} else {
	  block->j_number = 0.0;
	}
      }
    } else if (settings->plane == CANON_PLANE::YZ) {
      CHKS((block->i_flag), NCE_I_WORD_GIVEN_FOR_ARC_IN_YZ_PLANE);
      if (!block->j_flag) { /* j or k flag on to get here */
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'J');
	} else {
	  block->j_number = 0.0;
	}
      } else if (!block->k_flag) {
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'K');
	} else {
	  block->k_number = 0.0;
	}
      }
    } else if (settings->plane == CANON_PLANE::XZ) {
      CHKS((block->j_flag), NCE_J_WORD_GIVEN_FOR_ARC_IN_XZ_PLANE);
      if (!block->i_flag) { /* i or k flag on to get here */
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'I');
	} else {
	  block->i_number = 0.0;
	}
      } else if (!block->k_flag) {
	if (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE) {
	  ERS(_("%c word missing in absolute center arc"), 'K');
	} else {
	  block->k_number = 0.0;
	}
      }
    } else {
      ERS(NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ);
    }
  } else {
    // in R format, we need some XYZ words specified because a full circle is not allowed.
    if (settings->plane == CANON_PLANE::XY) {
        CHKS(((!block->x_flag) && (!block->y_flag) && (!block->radius_flag) && (!block->theta_flag)),
            NCE_X_AND_Y_WORDS_MISSING_FOR_ARC_IN_XY_PLANE);
    } else if (settings->plane == CANON_PLANE::YZ) {
        CHKS(((!block->y_flag) && (!block->z_flag)),
            NCE_Y_AND_Z_WORDS_MISSING_FOR_ARC_IN_YZ_PLANE);
    } else if (settings->plane == CANON_PLANE::XZ) {
        CHKS(((!block->x_flag) && (!block->z_flag)),
            NCE_X_AND_Z_WORDS_MISSING_FOR_ARC_IN_XZ_PLANE);
    }
  }


  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end,
                &u_end, &v_end, &w_end));

  settings->motion_mode = move;

  // Should be done with changes to settings here, so we can pack the state
  write_canon_state_tag(block, settings);

  if (settings->plane == CANON_PLANE::XY) {
    if ((settings->cutter_comp_side == CUTTER_COMP::OFF) ||
        (settings->cutter_comp_radius == 0.0)) {
      status =
        convert_arc2(move, block, settings,
                     &(settings->current_x), &(settings->current_y),
                     &(settings->current_z), end_x, end_y, end_z,
                     AA_end, BB_end, CC_end,
                     u_end, v_end, w_end,
                     block->i_number, block->j_number);
      CHP(status);
    } else if (first) {
      status = convert_arc_comp1(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    } else {
      status = convert_arc_comp2(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    }
  } else if (settings->plane == CANON_PLANE::XZ) {
    if ((settings->cutter_comp_side == CUTTER_COMP::OFF) ||
        (settings->cutter_comp_radius == 0.0)) {
      status =
        convert_arc2(move, block, settings,
                     &(settings->current_z), &(settings->current_x),
                     &(settings->current_y), end_z, end_x, end_y,
                     AA_end, BB_end, CC_end,
                     u_end, v_end, w_end,
                     block->k_number, block->i_number);
      CHP(status);
    } else if (first) {
      status = convert_arc_comp1(move, block, settings, end_z, end_x, end_y,
                                 block->k_number, block->i_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    } else {
      status = convert_arc_comp2(move, block, settings, end_z, end_x, end_y,
                                 block->k_number, block->i_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);

      CHP(status);
    }
  } else if (settings->plane == CANON_PLANE::YZ) {
    status =
      convert_arc2(move, block, settings,
                   &(settings->current_y), &(settings->current_z),
                   &(settings->current_x), end_y, end_z, end_x,
                   AA_end, BB_end, CC_end,
                   u_end, v_end, w_end,
                   block->j_number, block->k_number);
    CHP(status);
  } else
    ERS(NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc2

Returned Value: int
   If arc_data_ijk or arc_data_r returns an error code,
   this returns that code.
   Otherwise, it returns INTERP_OK.

Side effects:
   This executes an arc command at feed rate. It also updates the
   setting of the position of the tool point to the end point of the move.

Called by: convert_arc.

This converts a helical or circular arc.

*/

int Interp::convert_arc2(int move,       //!< either G_2 (cw arc) or G_3 (ccw arc)
                        block_pointer block,    //!< pointer to a block of RS274 instructions
                        setup_pointer settings, //!< pointer to machine settings
                        double *current1,       //!< pointer to current value of coordinate 1
                        double *current2,       //!< pointer to current value of coordinate 2
                        double *current3,       //!< pointer to current value of coordinate 3
                        double end1,    //!< coordinate 1 value at end of arc
                        double end2,    //!< coordinate 2 value at end of arc
                        double end3,    //!< coordinate 3 value at end of arc
                        double AA_end,  //!< a-value at end of arc
                        double BB_end,  //!< b-value at end of arc
                        double CC_end,  //!< c-value at end of arc
                         double u, double v, double w, //!< values at end of arc
                        double offset1, //!< center, either abs or offset from current
                        double offset2)
{
  double center1;
  double center2;
  int turn;                     /* number of full or partial turns CCW in arc */
  CANON_PLANE plane = settings->plane;

  // Spiral tolerance is the amount of "spiral" allowed in a given arc segment, or (r2-r1)/theta
  double spiral_abs_tolerance = (settings->length_units == CANON_UNITS_INCHES) ?
    settings->center_arc_radius_tolerance_inch : settings->center_arc_radius_tolerance_mm;

  // Radius tolerance allows a bit of leeway on the minimum radius for a radius defined arc.
  double radius_tolerance = (settings->length_units == CANON_UNITS_INCHES) ?
    RADIUS_TOLERANCE_INCH : RADIUS_TOLERANCE_MM;

  if (block->r_flag) {
      CHP(arc_data_r(move, plane, *current1, *current2, end1, end2,
                     block->r_number, block->p_flag? round_to_int(block->p_number) : 1,
                     &center1, &center2, &turn, radius_tolerance));
  } else {
      CHP(arc_data_ijk(move, plane, *current1, *current2, end1, end2,
                       (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE),
                       offset1, offset2, block->p_flag? round_to_int(block->p_number) : 1,
                       &center1, &center2, &turn, radius_tolerance, spiral_abs_tolerance, SPIRAL_RELATIVE_TOLERANCE));
  }
  inverse_time_rate_arc(*current1, *current2, *current3, center1, center2,
                        turn, end1, end2, end3, block, settings);

  ARC_FEED(block->line_number, end1, end2, center1, center2, turn, end3,
           AA_end, BB_end, CC_end, u, v, w);
  *current1 = end1;
  *current2 = end2;
  *current3 = end3;
  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u;
  settings->v_current = v;
  settings->w_current = w;

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc_comp1

Returned Value: int
   If arc_data_comp_ijk or arc_data_comp_r returns an error code,
   this returns that code.
   Otherwise, it returns INTERP_OK.

Side effects:
   This executes an arc command at
   feed rate. It also updates the setting of the position of
   the tool point to the end point of the move.

Called by: convert_arc.

This function converts a helical or circular arc, generating only one
arc. This is called when cutter radius compensation is on and this is
the first cut after the turning on.

The arc which is generated is derived from a second arc which passes
through the programmed end point and is tangent to the cutter at its
current location. The generated arc moves the tool so that it stays
tangent to the second arc throughout the move.

*/

int Interp::convert_arc_comp1(int move,  //!< either G_2 (cw arc) or G_3 (ccw arc)
                              block_pointer block,       //!< pointer to a block of RS274/NGC instructions
                              setup_pointer settings,    //!< pointer to machine settings
                              double end_x,      //!< x-value at end of programmed (then actual) arc
                              double end_y,      //!< y-value at end of programmed (then actual) arc
                              double end_z,      //!< z-value at end of arc
                              double offset_x, double offset_y,
                              double AA_end,     //!< a-value at end of arc
                             double BB_end,     //!< b-value at end of arc
                              double CC_end,     //!< c-value at end of arc
                              double u_end, double v_end, double w_end) //!< uvw at end of arc
{
    double center_x, center_y;
    double gamma;                 /* direction of perpendicular to arc at end */
    CUTTER_COMP side;                     /* offset side - right or left              */
    double tool_radius;
    int turn;                     /* 1 for counterclockwise, -1 for clockwise */
    double cx, cy, cz; // current
    CANON_PLANE plane = settings->plane;

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */

    double spiral_abs_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? settings->center_arc_radius_tolerance_inch : settings->center_arc_radius_tolerance_mm;
    double radius_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? RADIUS_TOLERANCE_INCH : RADIUS_TOLERANCE_MM;

    comp_get_current(settings, &cx, &cy, &cz);

    CHKS((hypot((end_x - cx), (end_y - cy)) <= tool_radius),
         _("Radius of cutter compensation entry arc is not greater than the tool radius"));

    if (block->r_flag) {
        CHP(arc_data_comp_r(move, plane, side, tool_radius, cx, cy, end_x, end_y,
                            block->r_number, block->p_flag? round_to_int(block->p_number): 1,
                            &center_x, &center_y, &turn, radius_tolerance));
    } else {
        CHP(arc_data_comp_ijk(move, plane, side, tool_radius, cx, cy, end_x, end_y,
                              (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE),
                              offset_x, offset_y, block->p_flag? round_to_int(block->p_number): 1,
                              &center_x, &center_y, &turn, radius_tolerance, spiral_abs_tolerance, SPIRAL_RELATIVE_TOLERANCE));
    }

    inverse_time_rate_arc(cx, cy, cz, center_x, center_y,
                          turn, end_x, end_y, end_z, block, settings);


    // the tool will end up in gamma direction from the programmed arc endpoint
    if TOOL_INSIDE_ARC(side, turn) {
        // tool inside the arc: ends up toward the center
        gamma = atan2((center_y - end_y), (center_x - end_x));
    } else {
        // outside: away from the center
        gamma = atan2((end_y - center_y), (end_x - center_x));
    }

    settings->cutter_comp_firstmove = false;

    comp_set_programmed(settings, end_x, end_y, end_z);

    // move endpoint to the compensated position.  This changes the radius and center.
    end_x += tool_radius * cos(gamma);
    end_y += tool_radius * sin(gamma);

    /* To find the new center:
       imagine a right triangle ABC with A being the endpoint of the
       compensated arc, B being the center of the compensated arc, C being
       the midpoint between start and end of the compensated arc. AB_ang
       is the direction of A->B.  A_ang is the angle of the triangle
       itself.  We need to find a new center for the compensated arc
       (point B). */

    double b_len = hypot(cy - end_y, cx - end_x) / 2.0;
    double AB_ang = atan2(center_y - end_y, center_x - end_x);
    double A_ang = atan2(cy - end_y, cx - end_x) - AB_ang;

    CHKS((fabs(cos(A_ang)) < TOLERANCE_EQUAL), NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);

    double c_len = b_len/cos(A_ang);

    // center of the arc is c_len from end in direction AB
    center_x = end_x + c_len * cos(AB_ang);
    center_y = end_y + c_len * sin(AB_ang);

    /* center to endpoint distances matched before - they still should. */
    CHKS((fabs(hypot(center_x-end_x,center_y-end_y) -
              hypot(center_x-cx,center_y-cy)) > spiral_abs_tolerance),
        NCE_BUG_IN_TOOL_RADIUS_COMP);

    // need this move for lathes to move the tool origin first.  otherwise, the arc isn't an arc.
    if (settings->cutter_comp_orientation != 0 && settings->cutter_comp_orientation != 9) {
        enqueue_STRAIGHT_FEED(settings, block->line_number,
                              0, 0, 0,
                              cx, cy, cz,
                              AA_end, BB_end, CC_end, u_end, v_end, w_end);
        set_endpoint(cx, cy);
    }

    enqueue_ARC_FEED(settings, block->line_number,
                     find_turn(cx, cy, center_x, center_y, turn, end_x, end_y),
                     end_x, end_y, center_x, center_y, turn, end_z,
                     AA_end, BB_end, CC_end, u_end, v_end, w_end);

    comp_set_current(settings, end_x, end_y, end_z);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc_comp2

Returned Value: int
   If arc_data_ijk or arc_data_r returns an error code,
   this returns that code.
   If any of the following errors occurs, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. A concave corner is found: NCE_CONCAVE_CORNER_WITH_CUTTER_RADIUS_COMP
   2. The tool will not fit inside an arc:
      NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP

Side effects:
   This executes an arc command feed rate. If needed, at also generates
   an arc to go around a convex corner. It also updates the setting of
   the position of the tool point to the end point of the move.
Called by: convert_arc.

This function converts a helical or circular arc. The axis must be
parallel to the z-axis. This is called when cutter radius compensation
is on and this is not the first cut after the turning on.

If one or more rotary axes is moved in this block and an extra arc is
required to go around a sharp corner, all the rotary axis motion
occurs on the main arc and none on the extra arc.  An alternative
might be to distribute the rotary axis motion over the extra arc and
the programmed arc in proportion to their lengths.

If the Z-axis is moved in this block and an extra arc is required to
go around a sharp corner, all the Z-axis motion occurs on the main arc
and none on the extra arc.  An alternative might be to distribute the
Z-axis motion over the extra arc and the main arc in proportion to
their lengths.

*/

int Interp::convert_arc_comp2(int move,  //!< either G_2 (cw arc) or G_3 (ccw arc)
                              block_pointer block,       //!< pointer to a block of RS274/NGC instructions
                              setup_pointer settings,    //!< pointer to machine settings
                              double end_x,      //!< x-value at end of programmed (then actual) arc
                              double end_y,      //!< y-value at end of programmed (then actual) arc
                              double end_z,      //!< z-value at end of arc
                              double offset_x, double offset_y,
                              double AA_end,     //!< a-value at end of arc
                              double BB_end,     //!< b-value at end of arc
                              double CC_end,     //!< c-value at end of arc
                              double u, double v, double w) //!< uvw at end of arc
{
    double alpha;                 /* direction of tangent to start of arc */
    double arc_radius;
    double beta;                  /* angle between two tangents above */
    double centerx, centery;              /* center of arc */
    double delta;                 /* direction of radius from start of arc to center of arc */
    double gamma;                 /* direction of perpendicular to arc at end */
    double midx, midy;
    CUTTER_COMP side;
    double small = TOLERANCE_CONCAVE_CORNER;      /* angle for testing corners */
    double opx = 0, opy = 0, opz = 0;
    double theta;                 /* direction of tangent to last cut */
    double tool_radius;
    int turn;                     /* number of full or partial circles CCW */
    CANON_PLANE plane = settings->plane;
    double cx, cy, cz;
    double new_end_x, new_end_y;

    double spiral_abs_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? settings->center_arc_radius_tolerance_inch : settings->center_arc_radius_tolerance_mm;
    double radius_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? RADIUS_TOLERANCE_INCH : RADIUS_TOLERANCE_MM;

    /* find basic arc data: center_x, center_y, and turn */

    comp_get_programmed(settings, &opx, &opy, &opz);
    comp_get_current(settings, &cx, &cy, &cz);


    if (block->r_flag) {
        CHP(arc_data_r(move, plane, opx, opy, end_x, end_y,
                       block->r_number, block->p_flag? round_to_int(block->p_number): 1,
                       &centerx, &centery, &turn, radius_tolerance));
    } else {
        CHP(arc_data_ijk(move, plane,
                         opx, opy, end_x, end_y,
                         (settings->ijk_distance_mode == DISTANCE_MODE::ABSOLUTE),
                         offset_x, offset_y, block->p_flag? round_to_int(block->p_number): 1,
                         &centerx, &centery, &turn, radius_tolerance, spiral_abs_tolerance, SPIRAL_RELATIVE_TOLERANCE));
    }

    inverse_time_rate_arc(opx, opy, opz, centerx, centery,
                          turn, end_x, end_y, end_z, block, settings);

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */
    arc_radius = hypot((centerx - end_x), (centery - end_y));
    theta = atan2(cy - opy, cx - opx);
    theta = (side == CUTTER_COMP::LEFT) ? (theta - M_PI_2l) : (theta + M_PI_2l);
    delta = atan2(centery - opy, centerx - opx);
    alpha = (move == G_3) ? (delta - M_PI_2l) : (delta + M_PI_2l);
    beta = (side == CUTTER_COMP::LEFT) ? (theta - alpha) : (alpha - theta);

    // normalize beta -90 to +270?
    beta = (beta > (1.5 * M_PIl)) ? (beta - (2 * M_PIl)) : (beta < -M_PI_2l) ? (beta + (2 * M_PIl)) : beta;

    if (((side == CUTTER_COMP::LEFT) && (move == G_3)) || ((side == CUTTER_COMP::RIGHT) && (move == G_2))) {
        // we are cutting inside the arc
        gamma = atan2((centery - end_y), (centerx - end_x));
        CHKS((arc_radius <= tool_radius),
            NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);
    } else {
        gamma = atan2((end_y - centery), (end_x - centerx));
        delta = (delta + M_PIl);
    }

    // move arc endpoint to the compensated position
    new_end_x = end_x + tool_radius * cos(gamma);
    new_end_y = end_y + tool_radius * sin(gamma);

    if (beta < -small ||
        beta > M_PIl + small ||
        // special detection for convex corner on tangent arc->arc (like atop the middle of "m" shape)
        // or tangent line->arc (atop "h" shape)
        (fabs(beta - M_PIl) < small && !TOOL_INSIDE_ARC(side, turn))
        ) {
        // concave
        if (qc().front().type != QARC_FEED) {
            // line->arc
            double cy = arc_radius * sin(beta - M_PI_2l);
            double toward_nominal;
            double dist_from_center;
            double angle_from_center;

            if TOOL_INSIDE_ARC(side, turn) {
                // tool is inside the arc
                dist_from_center = arc_radius - tool_radius;
                toward_nominal = cy + tool_radius;
                double l = toward_nominal / dist_from_center;
                CHKS((l > 1.0 || l < -1.0), _("Arc move in concave corner cannot be reached by the tool without gouging"));
                if(turn > 0) {
                    angle_from_center = theta + asin(l);
                } else {
                    angle_from_center = theta - asin(l);
                }
            } else {
                dist_from_center = arc_radius + tool_radius;
                toward_nominal = cy - tool_radius;
                double l = toward_nominal / dist_from_center;
                CHKS((l > 1.0 || l < -1.0), _("Arc move in concave corner cannot be reached by the tool without gouging"));
                if(turn > 0) {
                    angle_from_center = theta + M_PIl - asin(l);
                } else {
                    angle_from_center = theta + M_PIl + asin(l);
                }
            }

            midx = centerx + dist_from_center * cos(angle_from_center);
            midy = centery + dist_from_center * sin(angle_from_center);

            CHP(move_endpoint_and_flush(settings, midx, midy));
        } else {
            // arc->arc
            struct arc_feed &prev = qc().front().data.arc_feed;
            double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
            double newrad;
            if TOOL_INSIDE_ARC(side, turn) {
                newrad = arc_radius - tool_radius;
            } else {
                newrad = arc_radius + tool_radius;
            }

            double arc_cc, pullback, cc_dir, a;
            arc_cc = hypot(prev.center2 - centery, prev.center1 - centerx);

            CHKS((oldrad == 0 || arc_cc == 0), _("Arc to arc motion is invalid because the arcs have the same center"));
            a = (SQ(oldrad) + SQ(arc_cc) - SQ(newrad)) / (2 * oldrad * arc_cc);

            CHKS((a > 1.0 || a < -1.0), (_("Arc to arc motion makes a corner the compensated tool can't fit in without gouging")));
            pullback = acos(a);
            cc_dir = atan2(centery - prev.center2, centerx - prev.center1);

            double dir;
            if TOOL_INSIDE_ARC(side, prev.turn) {
                if(turn > 0)
                    dir = cc_dir + pullback;
                else
                    dir = cc_dir - pullback;
            } else {
                if(turn > 0)
                    dir = cc_dir - pullback;
                else
                    dir = cc_dir + pullback;
            }

            midx = prev.center1 + oldrad * cos(dir);
            midy = prev.center2 + oldrad * sin(dir);

            CHP(move_endpoint_and_flush(settings, midx, midy));
        }
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else if (beta > small) {           /* convex, two arcs needed */
        midx = opx + tool_radius * cos(delta);
        midy = opy + tool_radius * sin(delta);
        dequeue_canons(settings);
        enqueue_ARC_FEED(settings, block->line_number,
                         0.0, // doesn't matter since we won't move this arc's endpoint
                         midx, midy, opx, opy, ((side == CUTTER_COMP::LEFT) ? -1 : 1),
                         cz,
                         AA_end, BB_end, CC_end, u, v, w);
        dequeue_canons(settings);
        set_endpoint(midx, midy);
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else {                      /* convex, one arc needed */
        dequeue_canons(settings);
        set_endpoint(cx, cy);
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    }

    comp_set_programmed(settings, end_x, end_y, end_z);
    comp_set_current(settings, new_end_x, new_end_y, end_z);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u;
    settings->v_current = v;
    settings->w_current = w;

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_axis_offsets

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The function is called when cutter radius compensation is on:
      NCE_CANNOT_CHANGE_AXIS_OFFSETS_WITH_CUTTER_RADIUS_COMP
   2. The g_code argument is not G_52, G_92, G_92_1, G_92_2, or G_92_3
      NCE_BUG_CODE_NOT_IN_G52_G92_SERIES

Side effects:
   SET_G92_OFFSET is called, and the coordinate
   values for the axis offsets are reset. The coordinates of the
   current point are reset. Parameters may be set.

Called by: convert_modal_0.

The action of G92 is described in [NCMS, pages 10 - 11] and {Fanuc,
pages 61 - 63]. [NCMS] is ambiguous about the intent, but [Fanuc]
is clear. When G92 is executed, an offset of the origin is calculated
so that the coordinates of the current point with respect to the moved
origin are as specified on the line containing the G92. If an axis
is not mentioned on the line, the coordinates of the current point
are not changed. The execution of G92 results in an axis offset being
calculated and saved for each of the six axes, and the axis offsets
are always used when motion is specified with respect to absolute
distance mode using any of the nine coordinate systems (those designated
by G54 - G59.3). Thus all nine coordinate systems are affected by G92.

G92 and G52 offsets are relative to the current G5x offsets so they
are applied after the G5x offsets and G10 R rotations are applied.

Being in incremental distance mode has no effect on the action of G92
in this implementation. [NCMS] is not explicit about this, but it is
implicit in the second sentence of [Fanuc, page 61].

The offset is the amount the origin must be moved so that the
coordinate of the controlled point has the specified value. For
example, if the current point is at X=4 in the currently specified
coordinate system and the current X-axis offset is zero, then "G92 x7"
causes the X-axis offset to be reset to -3.

Since a non-zero offset may be already be in effect when the G92 is
called, that must be taken into account.

In addition to causing the axis offset values in the _setup model to be
set, G52 and G92 set parameters 5211 to 5216 to the x,y,z,a,b,c axis
offsets.

The action of G92.2 is described in [NCMS, page 12]. There is no
equivalent command in [Fanuc]. G92.2 resets axis offsets to zero.
G92.1, also included in [NCMS, page 12] (but the usage here differs
slightly from the spec), is like G92.2, except that it also causes
the axis offset parameters to be set to zero, whereas G92.2 does not
zero out the parameters.

G92.3 is not in [NCMS]. It sets the axis offset values to the values
given in the parameters.

*/

int Interp::convert_axis_offsets(int g_code,     //!< g_code being executed (must be in G_92 series)
                                block_pointer block,    //!< pointer to a block of RS274/NGC instructions
                                setup_pointer settings) //!< pointer to machine settings
{
  double *pars;                 /* short name for settings->parameters            */

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),      /* not "== true" */
      NCE_CANNOT_CHANGE_AXIS_OFFSETS_WITH_CUTTER_RADIUS_COMP);
  CHKS((block->a_flag && settings->a_axis_wrapped &&
	(block->a_number <= -360.0 || block->a_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")),
       block->a_number, 'A');
  CHKS((block->b_flag && settings->b_axis_wrapped &&
	(block->b_number <= -360.0 || block->b_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")),
       block->b_number, 'B');
  CHKS((block->c_flag && settings->c_axis_wrapped &&
	(block->c_number <= -360.0 || block->c_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")),
       block->c_number, 'C');
  pars = settings->parameters;
  if ((g_code == G_52) || (g_code == G_92)) {
      pars[G92_APPLIED] = 1.0;

      if (g_code == G_52) {
	  if (block->x_flag) {
	      settings->current_x += settings->axis_offset_x - block->x_number;
	      settings->axis_offset_x = block->x_number;
	  }

	  if (block->y_flag) {
	      settings->current_y += settings->axis_offset_y - block->y_number;
	      settings->axis_offset_y = block->y_number;
	  }

	  if (block->z_flag) {
	      settings->current_z += settings->axis_offset_z - block->z_number;
	      settings->axis_offset_z = block->z_number;
	  }
	  if (block->a_flag) {
	      settings->AA_current += settings->AA_axis_offset - block->a_number;
	      settings->AA_axis_offset = block->a_number;
	  }
	  if (block->b_flag) {
	      settings->BB_current += settings->BB_axis_offset - block->b_number;
	      settings->BB_axis_offset = block->b_number;
	  }
	  if (block->c_flag) {
	      settings->CC_current += settings->CC_axis_offset - block->c_number;
	      settings->CC_axis_offset = block->c_number;
	  }
	  if (block->u_flag) {
	      settings->u_current += settings->u_axis_offset - block->u_number;
	      settings->u_axis_offset = block->u_number;
	  }
	  if (block->v_flag) {
	      settings->v_current += settings->v_axis_offset - block->v_number;
	      settings->v_axis_offset = block->v_number;
	  }
	  if (block->w_flag) {
	      settings->w_current += settings->w_axis_offset - block->w_number;
	      settings->w_axis_offset = block->w_number;
	  }

      } else {
	  if (block->x_flag) {
	      settings->axis_offset_x =
		  (settings->current_x + settings->axis_offset_x - block->x_number);
	      settings->current_x = block->x_number;
	  }

	  if (block->y_flag) {
	      settings->axis_offset_y =
		  (settings->current_y + settings->axis_offset_y - block->y_number);
	      settings->current_y = block->y_number;
	  }

	  if (block->z_flag) {
	      settings->axis_offset_z =
		  (settings->current_z + settings->axis_offset_z - block->z_number);
	      settings->current_z = block->z_number;
	  }
	  if (block->a_flag) {
	      settings->AA_axis_offset = (settings->AA_current +
					  settings->AA_axis_offset - block->a_number);
	      settings->AA_current = block->a_number;
	  }
	  if (block->b_flag) {
	      settings->BB_axis_offset = (settings->BB_current +
					  settings->BB_axis_offset - block->b_number);
	      settings->BB_current = block->b_number;
	  }
	  if (block->c_flag) {
	      settings->CC_axis_offset = (settings->CC_current +
					  settings->CC_axis_offset - block->c_number);
	      settings->CC_current = block->c_number;
	  }
	  if (block->u_flag) {
	      settings->u_axis_offset = (settings->u_current +
					 settings->u_axis_offset - block->u_number);
	      settings->u_current = block->u_number;
	  }
	  if (block->v_flag) {
	      settings->v_axis_offset = (settings->v_current +
					 settings->v_axis_offset - block->v_number);
	      settings->v_current = block->v_number;
	  }
	  if (block->w_flag) {
	      settings->w_axis_offset = (settings->w_current +
					 settings->w_axis_offset - block->w_number);
	      settings->w_current = block->w_number;
	  }
      }

    SET_G92_OFFSET(settings->axis_offset_x,
                   settings->axis_offset_y,
                   settings->axis_offset_z,
                   settings->AA_axis_offset,
                   settings->BB_axis_offset,
                   settings->CC_axis_offset,
                   settings->u_axis_offset,
                   settings->v_axis_offset,
                   settings->w_axis_offset);

    pars[5211] = PROGRAM_TO_USER_LEN(settings->axis_offset_x);
    pars[5212] = PROGRAM_TO_USER_LEN(settings->axis_offset_y);
    pars[5213] = PROGRAM_TO_USER_LEN(settings->axis_offset_z);
    pars[5214] = PROGRAM_TO_USER_ANG(settings->AA_axis_offset);
    pars[5215] = PROGRAM_TO_USER_ANG(settings->BB_axis_offset);
    pars[5216] = PROGRAM_TO_USER_ANG(settings->CC_axis_offset);
    pars[5217] = PROGRAM_TO_USER_LEN(settings->u_axis_offset);
    pars[5218] = PROGRAM_TO_USER_LEN(settings->v_axis_offset);
    pars[5219] = PROGRAM_TO_USER_LEN(settings->w_axis_offset);

  } else if ((g_code == G_92_1) || (g_code == G_92_2)) {
    pars[5210] = 0.0;
    settings->current_x = settings->current_x + settings->axis_offset_x;
    settings->current_y = settings->current_y + settings->axis_offset_y;
    settings->current_z = settings->current_z + settings->axis_offset_z;
    settings->AA_current = (settings->AA_current + settings->AA_axis_offset);
    settings->BB_current = (settings->BB_current + settings->BB_axis_offset);
    settings->CC_current = (settings->CC_current + settings->CC_axis_offset);
    settings->u_current = (settings->u_current + settings->u_axis_offset);
    settings->v_current = (settings->v_current + settings->v_axis_offset);
    settings->w_current = (settings->w_current + settings->w_axis_offset);

    SET_G92_OFFSET(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    settings->axis_offset_x = 0.0;
    settings->axis_offset_y = 0.0;
    settings->axis_offset_z = 0.0;
    settings->AA_axis_offset = 0.0;
    settings->BB_axis_offset = 0.0;
    settings->CC_axis_offset = 0.0;
    settings->u_axis_offset = 0.0;
    settings->v_axis_offset = 0.0;
    settings->w_axis_offset = 0.0;
    if (g_code == G_92_1) {
      pars[G92_X] = 0.0;
      pars[G92_Y] = 0.0;
      pars[G92_Z] = 0.0;
      pars[G92_A] = 0.0;
      pars[G92_B] = 0.0;
      pars[G92_C] = 0.0;
      pars[G92_U] = 0.0;
      pars[G92_V] = 0.0;
      pars[G92_W] = 0.0;
    }
  } else if (g_code == G_92_3) {
    pars[5210] = 1.0;
    settings->current_x =
      settings->current_x + settings->axis_offset_x - USER_TO_PROGRAM_LEN(pars[5211]);
    settings->current_y =
      settings->current_y + settings->axis_offset_y - USER_TO_PROGRAM_LEN(pars[5212]);
    settings->current_z =
      settings->current_z + settings->axis_offset_z - USER_TO_PROGRAM_LEN(pars[5213]);
    settings->AA_current =
      settings->AA_current + settings->AA_axis_offset - USER_TO_PROGRAM_ANG(pars[5214]);
    settings->BB_current =
      settings->BB_current + settings->BB_axis_offset - USER_TO_PROGRAM_ANG(pars[5215]);
    settings->CC_current =
      settings->CC_current + settings->CC_axis_offset - USER_TO_PROGRAM_ANG(pars[5216]);
    settings->u_current =
      settings->u_current + settings->u_axis_offset - USER_TO_PROGRAM_LEN(pars[5217]);
    settings->v_current =
      settings->v_current + settings->v_axis_offset - USER_TO_PROGRAM_LEN(pars[5218]);
    settings->w_current =
      settings->w_current + settings->w_axis_offset - USER_TO_PROGRAM_LEN(pars[5219]);

    settings->axis_offset_x = USER_TO_PROGRAM_LEN(pars[5211]);
    settings->axis_offset_y = USER_TO_PROGRAM_LEN(pars[5212]);
    settings->axis_offset_z = USER_TO_PROGRAM_LEN(pars[5213]);
    settings->AA_axis_offset = USER_TO_PROGRAM_ANG(pars[5214]);
    settings->BB_axis_offset = USER_TO_PROGRAM_ANG(pars[5215]);
    settings->CC_axis_offset = USER_TO_PROGRAM_ANG(pars[5216]);
    settings->u_axis_offset = USER_TO_PROGRAM_LEN(pars[5217]);
    settings->v_axis_offset = USER_TO_PROGRAM_LEN(pars[5218]);
    settings->w_axis_offset = USER_TO_PROGRAM_LEN(pars[5219]);

    SET_G92_OFFSET(settings->axis_offset_x,
                   settings->axis_offset_y,
                   settings->axis_offset_z,
                   settings->AA_axis_offset,
                   settings->BB_axis_offset,
                   settings->CC_axis_offset,
                   settings->u_axis_offset,
                   settings->v_axis_offset,
                   settings->w_axis_offset);
  } else
    ERS(NCE_BUG_CODE_NOT_IN_G52_G92_SERIES);

  return INTERP_OK;
}

#define VAL_LEN 30

int Interp::convert_param_comment(char *comment, char *expanded, int /*len*/)
{
    FORCE_LC_NUMERIC_C;
    int i;
    char param[LINELEN+1];
    char format[5] = "%lf";
    int paramNumber;
    int stat;
    double value;
    char valbuf[VAL_LEN]; // max double length + room
    char *v;
    int found;

    while(*comment)
    {

        if(*comment == '%')
        {
            // skip over the '%'
            comment++;

            // convenient integer looking
            if(*comment == 'd')
            {
                comment++;
                strcpy(format, "%.0f");
            }
            // convenient 4 position float
            else if(*comment == 'f')
            {
                comment++;
                strcpy(format, "%.4f");
            }
            // arbitrary 0-9 position float
            else if(*comment == '.')
            {
                comment++;
                if(isdigit(*comment))
                {
                    // forward to the (hopefully) letter f
                    comment++;
                    if(*comment == 'f')
                    {
                        // back up to get the digit into format
                        comment--;
                        format[0] = '%';
                        format[1] = '.';
                        format[2] = *comment;
                        format[3] = 'f';
                        format[4] = 0;
                        comment++;
                        comment++;
                    }
                    else
                    {
                        // not a format string so,
                        // back up to the digit to continue processing
                        comment--;
                        *expanded++ = '.';

                    }
                }
                else
                {
                    *expanded++ = '.';
                }

            }
            else
            {
                *expanded++ = '%';
            }
        }
        else if(*comment == '#')
        {
            found = 0;
            logDebug("a parameter");

            // skip over the '#'
            comment++;
            CHKS((0 == *comment), NCE_NAMED_PARAMETER_NOT_TERMINATED);

            if(isdigit(*comment))  // is this numeric param?
            {
                logDebug("numeric parameter");
                for(i=0; isdigit(*comment)&& (i<LINELEN); i++)
                {
                    param[i] = *comment++;
                }
                param[i] = 0;
                paramNumber = atoi(param);
                if((paramNumber >= 0) &&
                   (paramNumber < RS274NGC_MAX_PARAMETERS))
                {
                    value = _setup.parameters[paramNumber];
                    found = 1;
                }
            }
            else if(*comment == '<')
            {
                logDebug("name parameter");
                // this is a name parameter
                // skip over the '<'
                comment++;
                CHKS((0 == *comment), NCE_NAMED_PARAMETER_NOT_TERMINATED);

                for(i=0; (')' != *comment) &&
                        (i<LINELEN) && (0 != *comment);)
                {
                    if('>' == *comment)
                    {
                        break;     // done
                    }
                    if(isspace(*comment)) // skip space inside the param
                    {
                        comment++;
                        continue;
                    }
                    else
                    {
		        // if tolower is a macro, may need this int
		        int c = *comment++;
			if (FEATURE(NO_DOWNCASE_OWORD))
			    param[i] = c;
			else
			    param[i] = tolower(c);
                        i++;
                    }
                }
                if('>' != *comment)
                {
                    ERS(NCE_NAMED_PARAMETER_NOT_TERMINATED);
                }
                else
                {
                    comment++;
                }

                // terminate the name
                param[i] = 0;

                // now lookup the name
                find_named_param(param, &stat, &value);
                if(stat)
                {
                    found = 1;
                }
            }
            else
            {
                // neither numeric or name
                logDebug("neither numeric nor name");
                // just store the '#'
                *expanded++ = '#';

                CHKS((*comment == 0), NCE_NAMED_PARAMETER_NOT_TERMINATED);
                continue;
            }

            // we have a parameter -- now insert it
            // we have the value
            if(found)
            {
		// avoid -0.0/0.0 issues
		double pvalue = equal(value, 0.0) ? 0.0 : value;
                int n = snprintf(valbuf, VAL_LEN, format, pvalue);
                bool fail = (n >= VAL_LEN || n < 0);
                if(fail)
                    rtapi_strxcpy(valbuf, "######");

            }
            else
            {
                rtapi_strxcpy(valbuf, "######");
            }
            logDebug("found:%d value:|%s|", found, valbuf);

            v = valbuf;
            while(*v)
            {
                *expanded++ = *v++;
            }
        }
        else  // not a '#'
        {
            *expanded++ = *comment++;
        }
    }
    *expanded = 0; // the final nul

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_comment

Returned Value: int (INTERP_OK)

Side effects:
   The message function is called if the string starts with "MSG,".
   Otherwise, the comment function is called.

Called by: execute_block

To be a message, the first four characters of the comment after the
opening left parenthesis must be "MSG,", ignoring the case of the
letters and allowing spaces or tabs anywhere before the comma (to make
the treatment of case and white space consistent with how it is
handled elsewhere).

Messages are not provided for in [NCMS]. They are implemented here as a
subtype of comment. This is an extension to the rs274NGC language.

*/

static int streq(char *s1, char *s2) {
    return !strcmp(s1, s2);
}

static int startswith(char *haystack, char *needle) {
    return !strncmp(haystack, needle, strlen(needle));
}

int Interp::convert_comment(char *comment, bool enqueue)       //!< string with comment
{
  enum
  { LC_SIZE = 256, EX_SIZE = 2*LC_SIZE};            // 256 from comment[256] in rs274ngc.hh
  char lc[LC_SIZE+1];
  char expanded[EX_SIZE+1];
  char MSG_STR[] = "msg,";

  //!!!KL add two -- debug => same as msg
  //!!!KL         -- print => goes to stdout
  char DEBUG_STR[] = "debug,";
  char PRINT_STR[] = "print,";
  char LOG_STR[] = "log,";
  char LOGOPEN_STR[] = "logopen,";
  char LOGAPPEND_STR[] = "logappend,";
  char LOGCLOSE_STR[] = "logclose";
  char PY_STR[] = "py,";
  char PYRUN_STR[] = "pyrun,";
  char PYRELOAD_STR[] = "pyreload";
  char ABORT_STR[] = "abort,";
  int m, n, start;

  // step over leading white space in comment
  m = 0;
  while (isspace(comment[m]))
    m++;
  start = m;
  // copy lowercase comment to lc[]
  for (n = 0; n < LC_SIZE && comment[m] != 0; m++, n++) {
    lc[n] = tolower(comment[m]);
  }
  lc[n] = 0;                    // null terminate

  // compare with MSG, SYSTEM, DEBUG, PRINT
  if (startswith(lc, MSG_STR)) {
    MESSAGE(comment + start + strlen(MSG_STR));
    return INTERP_OK;
  }
  else if (startswith(lc, DEBUG_STR))
  {
      convert_param_comment(comment+start+strlen(DEBUG_STR), expanded,
                            EX_SIZE);
      if (_setup.parameters[5599] > 0.0)
	  MESSAGE(expanded);
      return INTERP_OK;
  }
  else if (startswith(lc, PRINT_STR))
  {
      FILE *fd = get_stdout();
      if (fd) {
          convert_param_comment(comment+start+strlen(PRINT_STR), expanded,
                                EX_SIZE);
          fprintf(fd, "%s\n", expanded);
          fflush(fd);
      }
      return INTERP_OK;
  }
  else if (startswith(lc, LOG_STR))
  {
      convert_param_comment(comment+start+strlen(LOG_STR), expanded,
                            EX_SIZE);
      LOG(expanded);
      return INTERP_OK;
  }
  else if (startswith(lc, LOGOPEN_STR))
  {
      LOGOPEN(comment + start + strlen(LOGOPEN_STR));
      return INTERP_OK;
  }
  else if (startswith(lc, LOGAPPEND_STR))
  {
      LOGAPPEND(comment + start + strlen(LOGAPPEND_STR));
      return INTERP_OK;
  }
  else if (startswith(lc, PY_STR))
  {
      return py_execute(comment + start + strlen(PY_STR), false);
  }
  else if (startswith(lc, PYRUN_STR))
  {
      return py_execute(comment + start + strlen(PYRUN_STR), true);
  }
  else if (startswith(lc, PYRELOAD_STR))
  {
      return py_reload();
  }
  else if (startswith(lc, ABORT_STR))
  {
      convert_param_comment(comment+start+strlen(ABORT_STR), expanded,
                            EX_SIZE);
      setSavedError(expanded); // avoid printf interpretation
      return INTERP_ERROR;
  }
  else if (streq(lc, LOGCLOSE_STR))
  {
      LOGCLOSE();
      return INTERP_OK;
  }
  // else it's a real comment
  if (enqueue)
      enqueue_COMMENT(comment + start);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_control_mode

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_61, G_61_1, G_64 : NCE_BUG_CODE_NOT_G61_G61_1_OR_G64

Side effects: See below

Called by: convert_g.

The interpreter switches the machine settings to indicate the
control mode (CANON_EXACT_STOP, CANON_EXACT_PATH or CANON_CONTINUOUS)

A call is made to SET_MOTION_CONTROL_MODE(CANON_XXX), where CANON_XXX is
CANON_EXACT_PATH if g_code is G_61, CANON_EXACT_STOP if g_code is G_61_1,
and CANON_CONTINUOUS if g_code is G_64.

Setting the control mode to CANON_EXACT_STOP on G_61 would correspond
more closely to the meaning of G_61 as given in [NCMS, page 40], but
CANON_EXACT_PATH has the advantage that the tool does not stop if it
does not have to, and no evident disadvantage compared to
CANON_EXACT_STOP, so it is being used for G_61. G_61_1 is not defined
in [NCMS], so it is available and is used here for setting the control
mode to CANON_EXACT_STOP.

It is OK to call SET_MOTION_CONTROL_MODE(CANON_XXX) when CANON_XXX is
already in force.

*/

int Interp::convert_control_mode(
    int g_code,                   // g_code being executed (G_61, G61_1, G_64)
    double tolerance_in,          // tolerance for the path following in G64
    double naivecam_tolerance_in, // tolerance for the naivecam
    setup_pointer settings)       // pointer to machine settings
{
    double tolerance, naivecam_tolerance;
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
       (_("Cannot change control mode with cutter radius compensation on")));
  if (g_code == G_61) {
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);
    settings->control_mode = CANON_EXACT_PATH;
  } else if (g_code == G_61_1) {
    SET_MOTION_CONTROL_MODE(CANON_EXACT_STOP, 0);
    settings->control_mode = CANON_EXACT_STOP;
  } else if (g_code == G_64) {
      if (tolerance_in >= 0)
	  tolerance = tolerance_in;
      else
	  tolerance = 0;
      settings->control_mode = CANON_CONTINUOUS;
      settings->tolerance = tolerance;
      SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, tolerance);

      if (naivecam_tolerance_in >= 0)
	  naivecam_tolerance = naivecam_tolerance_in;
      else if (tolerance_in >= 0)
	  // if no naivecam_tolerance specified use same for both
	  naivecam_tolerance = tolerance_in;
      else
	  naivecam_tolerance = 0;
      settings->naivecam_tolerance = naivecam_tolerance;
      SET_NAIVECAM_TOLERANCE(naivecam_tolerance);

  } else
    ERS(NCE_BUG_CODE_NOT_G61_G61_1_OR_G64);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_coordinate_system

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The value of the g_code argument is not 540, 550, 560, 570, 580, 590
      591, 592, or 593:
      NCE_BUG_CODE_NOT_IN_RANGE_G54_TO_G593

Side effects:
   If the coordinate system selected by the g_code is not already in
   use, the canonical program coordinate system axis offset values are
   reset and the coordinate values of the current point are reset.

Called by: convert_g.

COORDINATE SYSTEMS (involves g10, g53, g54 - g59.3, g92)

The canonical machining functions view of coordinate systems is:
1. There are two coordinate systems: absolute and program.
2. All coordinate values are given in terms of the program coordinate system.
3. The offsets of the program coordinate system may be reset.

The RS274/NGC view of coordinate systems, as given in section 3.2
of [NCMS] is:
1. there are ten coordinate systems: absolute and 9 program. The
   program coordinate systems are numbered 1 to 9.
2. you can switch among the 9 but not to the absolute one. G54
   selects coordinate system 1, G55 selects 2, and so on through
   G56, G57, G58, G59, G59.1, G59.2, and G59.3.
3. you can set the offsets of the 9 program coordinate systems
   using G10 L2 Pn (n is the number of the coordinate system) with
   values for the axes in terms of the absolute coordinate system.
4. the first one of the 9 program coordinate systems is the default.
5. data for coordinate systems is stored in parameters [NCMS, pages 59 - 60].
6. g53 means to interpret coordinate values in terms of the absolute
   coordinate system for the one block in which g53 appears.
7. You can offset the current coordinate system using g92. This offset
   will then apply to all nine program coordinate systems.

The approach used in the interpreter mates the canonical and NGC views
of coordinate systems as follows:

During initialization, data from the parameters for the first NGC
coordinate system is used in a SET_ORIGIN_OFFSETS function call and
origin_index in the machine model is set to 1.

If a g_code in the range g54 - g59.3 is encountered in an NC program,
the data from the appropriate NGC coordinate system is copied into the
origin offsets used by the interpreter, a SET_ORIGIN_OFFSETS function
call is made, and the current position is reset.

If a g10 is encountered, the convert_setup function is called to reset
the offsets of the program coordinate system indicated by the P number
given in the same block.

If a g53 is encountered, the axis values given in that block are used
to calculate what the coordinates are of that point in the current
coordinate system, and a STRAIGHT_TRAVERSE or STRAIGHT_FEED function
call to that point using the calculated values is made. No offset
values are changed.

If a g92 is encountered, that is handled by the convert_axis_offsets
function. A g92 results in an axis offset for each axis being calculated
and stored in the machine model. The axis offsets are applied to all
nine coordinate systems. Axis offsets are initialized to zero.

*/

void Interp::rotate(double *x, double *y, double theta) {
    double xx, yy;
    double t = D2R(theta);
    xx = *x * cos(t) - *y * sin(t);
    yy = *x * sin(t) + *y * cos(t);
    *x = xx;
    *y = yy;
}

int Interp::convert_coordinate_system(int g_code,        //!< g_code called (must be one listed above)
                                     setup_pointer settings)    //!< pointer to machine settings
{
  int origin;
  double *parameters;

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
       (_("Cannot change coordinate systems with cutter radius compensation on")));
  parameters = settings->parameters;
  switch (g_code) {
  case G_54:
    origin = 1;
    break;
  case G_55:
    origin = 2;
    break;
  case G_56:
    origin = 3;
    break;
  case G_57:
    origin = 4;
    break;
  case G_58:
    origin = 5;
    break;
  case G_59:
    origin = 6;
    break;
  case G_59_1:
    origin = 7;
    break;
  case G_59_2:
    origin = 8;
    break;
  case G_59_3:
    origin = 9;
    break;
  default:
    ERS(NCE_BUG_CODE_NOT_IN_RANGE_G54_TO_G593);
  }

  if (origin == settings->origin_index) {       /* already using this origin */
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: continuing to use same coordinate system");
#endif
    return INTERP_OK;
  }

  // move the current point into the new system
  find_current_in_system(settings, origin,
                         &settings->current_x, &settings->current_y, &settings->current_z,
                         &settings->AA_current, &settings->BB_current, &settings->CC_current,
                         &settings->u_current, &settings->v_current, &settings->w_current);

  // remember that this is new system
  settings->origin_index = origin;
  parameters[5220] = (double) origin;

  // load the origin of the newly-selected system
  settings->origin_offset_x = USER_TO_PROGRAM_LEN(parameters[5201 + (origin * 20)]);
  settings->origin_offset_y = USER_TO_PROGRAM_LEN(parameters[5202 + (origin * 20)]);
  settings->origin_offset_z = USER_TO_PROGRAM_LEN(parameters[5203 + (origin * 20)]);
  settings->AA_origin_offset = USER_TO_PROGRAM_ANG(parameters[5204 + (origin * 20)]);
  settings->BB_origin_offset = USER_TO_PROGRAM_ANG(parameters[5205 + (origin * 20)]);
  settings->CC_origin_offset = USER_TO_PROGRAM_ANG(parameters[5206 + (origin * 20)]);
  settings->u_origin_offset = USER_TO_PROGRAM_LEN(parameters[5207 + (origin * 20)]);
  settings->v_origin_offset = USER_TO_PROGRAM_LEN(parameters[5208 + (origin * 20)]);
  settings->w_origin_offset = USER_TO_PROGRAM_LEN(parameters[5209 + (origin * 20)]);
  settings->rotation_xy = parameters[5210 + (origin * 20)];

  SET_G5X_OFFSET(origin,
                 settings->origin_offset_x,
                 settings->origin_offset_y,
                 settings->origin_offset_z,
                 settings->AA_origin_offset,
                 settings->BB_origin_offset,
                 settings->CC_origin_offset,
                 settings->u_origin_offset,
                 settings->v_origin_offset,
                 settings->w_origin_offset);

  SET_G92_OFFSET(settings->axis_offset_x,
                 settings->axis_offset_y,
                 settings->axis_offset_z,
                 settings->AA_axis_offset,
                 settings->BB_axis_offset,
                 settings->CC_axis_offset,
                 settings->u_axis_offset,
                 settings->v_axis_offset,
                 settings->w_axis_offset);

  SET_XY_ROTATION(settings->rotation_xy);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation

Returned Value: int
   If convert_cutter_compensation_on or convert_cutter_compensation_off
      is called and returns an error code, this returns that code.
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code is not G_40, G_41, or G_42:
      NCE_BUG_CODE_NOT_G40_G41_OR_G42

Side effects:
   The value of cutter_comp_side in the machine model mode is
   set to RIGHT, LEFT, or false. The currently active tool table index in
   the machine model (which is the index of the slot whose diameter
   value is used in cutter radius compensation) is updated.

Since cutter radius compensation is performed in the interpreter, no
call is made to any canonical function regarding cutter radius compensation.

Called by: convert_g

*/

int Interp::convert_cutter_compensation(int g_code,      //!< must be G_40, G_41, or G_42
                                       block_pointer block,     //!< pointer to a block of RS274 instructions
                                       setup_pointer settings)  //!< pointer to machine settings
{

  if (g_code == G_40) {
    CHP(convert_cutter_compensation_off(settings));
  } else if (g_code == G_41) {
    CHP(convert_cutter_compensation_on(CUTTER_COMP::LEFT, block, settings));
  } else if (g_code == G_42) {
    CHP(convert_cutter_compensation_on(CUTTER_COMP::RIGHT, block, settings));
  } else if (g_code == G_41_1) {
    CHP(convert_cutter_compensation_on(CUTTER_COMP::LEFT, block, settings));
  } else if (g_code == G_42_1) {
    CHP(convert_cutter_compensation_on(CUTTER_COMP::RIGHT, block, settings));
  } else
    ERS("BUG: Code not G40, G41, G41.1, G42, G42.1");

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation_off

Returned Value: int (INTERP_OK)

Side effects:
   A comment is made that cutter radius compensation is turned off.
   The machine model of the cutter radius compensation mode is set to false.
   The value of cutter_comp_firstmove in the machine model is set to true.
     This serves as a flag when cutter radius compensation is
     turned on again.

Called by: convert_cutter_compensation

*/

int Interp::convert_cutter_compensation_off(setup_pointer settings)      //!< pointer to machine settings
{
#ifdef DEBUG_EMC
  enqueue_COMMENT("interpreter: cutter radius compensation off");
#endif
  if(settings->cutter_comp_side != CUTTER_COMP::OFF && settings->cutter_comp_radius > 0.0 &&
     !settings->cutter_comp_firstmove) {
      double cx, cy, cz;
      comp_get_current(settings, &cx, &cy, &cz);
      CHP(move_endpoint_and_flush(settings, cx, cy));
      dequeue_canons(settings);
      settings->current_x = settings->program_x;
      settings->current_y = settings->program_y;
      settings->current_z = settings->program_z;
      settings->arc_not_allowed = true;
  }
  settings->cutter_comp_side = CUTTER_COMP::OFF;
  settings->cutter_comp_firstmove = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation_on

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The selected plane is not the XY plane:
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_OUT_OF_XY_PLANE
   2. Cutter radius compensation is already on:
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_WHEN_ON

Side effects:
   A COMMENT function call is made (conditionally) saying that the
   interpreter is switching mode so that cutter radius compensation is on.
   The value of cutter_comp_radius in the machine model mode is
   set to the absolute value of the radius given in the tool table.
   The value of cutter_comp_side in the machine model mode is
   set to RIGHT or LEFT. The currently active tool table index in
   the machine model is updated.

Called by: convert_cutter_compensation

check_other_codes checks that a d word occurs only in a block with g41
or g42.

Cutter radius compensation is carried out in the interpreter, so no
call is made to a canonical function (although there is a canonical
function, START_CUTTER_RADIUS_COMPENSATION, that could be called if
the primitive level could execute it).

This version uses a D word if there is one in the block, but it does
not require a D word, since the sample programs which the interpreter
is supposed to handle do not have them.  Logically, the D word is
optional, since the D word is always (except in cases we have never
heard of) the slot number of the tool in the spindle. Not requiring a
D word is contrary to [Fanuc, page 116] and [NCMS, page 79], however.
Both manuals require the use of the D-word with G41 and G42.

This version handles a negative offset radius, which may be
encountered if the programmed tool path is a center line path for
cutting a profile and the path was constructed using a nominal tool
diameter. Then the value in the tool table for the diameter is set to
be the difference between the actual diameter and the nominal
diameter. If the actual diameter is less than the nominal, the value
in the table is negative. The method of handling a negative radius is
to switch the side of the offset and use a positive radius. This
requires that the profile use arcs (not straight lines) to go around
convex corners.

*/

int Interp::convert_cutter_compensation_on(CUTTER_COMP side,     //!< side of path cutter is on (LEFT or RIGHT)
                                          block_pointer block,  //!< pointer to a block of RS274 instructions
                                          setup_pointer settings)       //!< pointer to machine settings
{
  double radius;
  int idx, orientation;

  CHKS((settings->plane != CANON_PLANE::XY && settings->plane != CANON_PLANE::XZ),
      NCE_RADIUS_COMP_ONLY_IN_XY_OR_XZ);
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_WHEN_ON);
  if(block->g_modes[GM_CUTTER_COMP] == G_41_1 || block->g_modes[GM_CUTTER_COMP] == G_42_1) {
      CHKS((!block->d_flag),
              _("G%d.1 with no D word"), block->g_modes[GM_CUTTER_COMP]/10 );
      radius = block->d_number_float / 2;
      if(block->l_number != -1) {
          CHKS((settings->plane != CANON_PLANE::XZ), _("G%d.1 with L word, but plane is not G18"), block->g_modes[GM_CUTTER_COMP]/10);
          orientation = block->l_number;
      } else {
          orientation = 0;
      }
  } else {
      if(!block->d_flag) {
          idx = 0;
      } else {
          int tool;
          CHKS(!is_near_int(&tool, block->d_number_float),
                  _("G%d requires D word to be a whole number"),
                   block->g_modes[GM_CUTTER_COMP]/10);
          CHKS((tool < 0), NCE_NEGATIVE_D_WORD_TOOL_RADIUS_INDEX_USED);
          CHP((find_tool_index(settings, tool, &idx)));
      }
      radius = USER_TO_PROGRAM_LEN(settings->tool_table[idx].diameter) / 2.0;
      orientation = settings->tool_table[idx].orientation;
      CHKS((settings->plane != CANON_PLANE::XZ && orientation != 0 && orientation != 9), _("G%d with lathe tool, but plane is not G18"), block->g_modes[GM_CUTTER_COMP]/10);
  }
  if (radius < 0.0) { /* switch side & make radius positive if radius negative */
    radius = -radius;
    if (side == CUTTER_COMP::RIGHT)
      side = CUTTER_COMP::LEFT;
    else
      side = CUTTER_COMP::RIGHT;
  }
#ifdef DEBUG_EMC
  if (side == CUTTER_COMP::RIGHT)
    enqueue_COMMENT("interpreter: cutter radius compensation on right");
  else
    enqueue_COMMENT("interpreter: cutter radius compensation on left");
#endif

  settings->cutter_comp_radius = radius;
  settings->cutter_comp_orientation = orientation;
  settings->cutter_comp_side = side;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_distance_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_90 or G_91: NCE_BUG_CODE_NOT_G90_OR_G91

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

// OK to call this in a concave corner with a deferred move, since it
// doesn't issue any CANONs

int Interp::convert_distance_mode(int g_code,    //!< g_code being executed (must be G_90 or G_91)
                                 setup_pointer settings)        //!< pointer to machine settings
{
  if (g_code == G_90) {
    if (settings->distance_mode != DISTANCE_MODE::ABSOLUTE) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: distance mode changed to absolute");
#endif
      settings->distance_mode = DISTANCE_MODE::ABSOLUTE;
    }
  } else if (g_code == G_91) {
    if (settings->distance_mode != DISTANCE_MODE::INCREMENTAL) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: distance mode changed to incremental");
#endif
      settings->distance_mode = DISTANCE_MODE::INCREMENTAL;
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G90_OR_G91);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_ijk_distance_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_90.1 or G_91.1: NCE_BUG_CODE_NOT_G90_OR_G91

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode for arc centers (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

// OK to call this in a concave corner with a deferred move, since it
// doesn't issue any CANONs except comments (and who cares where the comments are)

int Interp::convert_ijk_distance_mode(int g_code,    //!< g_code being executed (must be G_90_1 or G_91_1)
                                 setup_pointer settings)        //!< pointer to machine settings
{
  if (g_code == G_90_1) {
    if (settings->ijk_distance_mode != DISTANCE_MODE::ABSOLUTE) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: IJK distance mode changed to absolute");
#endif
      settings->ijk_distance_mode = DISTANCE_MODE::ABSOLUTE;
    }
  } else if (g_code == G_91_1) {
    if (settings->ijk_distance_mode != DISTANCE_MODE::INCREMENTAL) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: IJK distance mode changed to incremental");
#endif
      settings->ijk_distance_mode = DISTANCE_MODE::INCREMENTAL;
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G90_OR_G91);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_lathe_diameter_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_07 or G_08: NCE_BUG_CODE_NOT_G07_OR_G08

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode for arc centers (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

int Interp::convert_lathe_diameter_mode(int g_code,    //!< g_code being executed (must be G_90_1 or G_91_1)
                  block_pointer block,           //!< pointer to current block
                  setup_pointer settings)        //!< pointer to machine settings
{
  if (g_code == G_7) {
    if (!settings->lathe_diameter_mode) {
      if(block->x_flag)
      {
        block->x_number /= 2; //Apply scaling now
      }
      if(block->motion_to_be == G_76) {
          block->i_number /= 2;
          block->j_number /= 2;
          block->k_number /= 2;
      }
#ifdef DEBUG_EMC
      COMMENT("interpreter: Lathe diameter mode changed to diameter");
#endif
      settings->lathe_diameter_mode = true;
    }
  } else if (g_code == G_8) {
    if (settings->lathe_diameter_mode) {
      if(block->x_flag)
      {
        block->x_number *= 2; //Remove any existing scaling
      }
      if(block->motion_to_be == G_76) {
          block->i_number *= 2;
          block->j_number *= 2;
          block->k_number *= 2;
      }
#ifdef DEBUG_EMC
      COMMENT("interpreter: Lathe diameter mode changed to radius");
#endif
      settings->lathe_diameter_mode = false;
    }
  } else
    ERS("BUG: Code not G7 or G8");
  return INTERP_OK;
}


/****************************************************************************/

/*! convert_dwell

Returned Value: int (INTERP_OK)

Side effects:
   A dwell command is executed.

Called by: convert_g.

*/

int Interp::convert_dwell(setup_pointer /*settings*/, double time)   //!< time in seconds to dwell  */
{
  enqueue_DWELL(time);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_feed_mode

Returned Value: int
   If any of the following errors occur, this returns an error code.
   Otherwise, it returns INTERP_OK.
   1.  g_code isn't G_93, G_94 or G_95

Side effects:
   The interpreter switches the machine settings to indicate the current
   feed mode (UNITS_PER_MINUTE or INVERSE_TIME).

   The canonical machine to which commands are being sent does not have
   a feed mode, so no command setting the distance mode is generated in
   this function. A comment function call is made (conditionally)
   explaining the change in mode, however.

Called by: execute_block.

*/

int Interp::convert_feed_mode(int g_code,        //!< g_code being executed (must be G_93, G_94 or G_95)
                             setup_pointer settings)    //!< pointer to machine settings
{
  if (g_code == G_93) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to inverse time");
#endif
    settings->feed_mode = FEED_MODE::INVERSE_TIME;
    enqueue_SET_FEED_MODE(0, 0);
  } else if (g_code == G_94) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to units per minute");
#endif
    settings->feed_mode = FEED_MODE::UNITS_PER_MINUTE;
    enqueue_SET_FEED_MODE(0, 0);
    settings->feed_rate = 0.0;
    enqueue_SET_FEED_RATE(0);
  } else if(g_code == G_95) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to units per revolution");
#endif
    settings->feed_mode = FEED_MODE::UNITS_PER_REVOLUTION;
    enqueue_SET_FEED_MODE(settings->active_spindle , 1);
    settings->feed_rate = 0.0;
    enqueue_SET_FEED_RATE(0);
  } else
    ERS("BUG: Code not G93, G94, or G95");
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_feed_rate

Returned Value: int (INTERP_OK)

Side effects:
   The machine feed_rate is set to the value of f_number in the
     block by function call.
   The machine model feed_rate is set to that value.

Called by: execute_block

This is called only if the feed mode is UNITS_PER_MINUTE or UNITS_PER_REVOLUTION.

*/

int Interp::convert_feed_rate(block_pointer block,       //!< pointer to a block of RS274 instructions
                             setup_pointer settings)    //!< pointer to machine settings
{
  settings->feed_rate = block->f_number;
  enqueue_SET_FEED_RATE(block->f_number);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_g

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_control_mode
      convert_coordinate_system
      convert_cutter_compensation
      convert_distance_mode
      convert_ijk_distance_mode
      convert_lathe_diameter_mode
      convert_dwell
      convert_length_units
      convert_modal_0
      convert_motion
      convert_retract_mode
      convert_set_plane
      convert_tool_length_offset
   Otherwise, it returns INTERP_OK.

Side effects:
   Any g_codes in the block (excluding g93 and 94) and any implicit
   motion g_code are executed.

Called by: execute_block.

This takes a pointer to a block of RS274/NGC instructions (already
read in) and creates the appropriate output commands corresponding to
any "g" codes in the block.

Codes g93 and g94, which set the feed mode, are executed earlier by
execute_block before reading the feed rate.

G-codes are are executed in the following order.
1.  mode 0, G4 only - dwell. Left here from earlier versions.
2.  mode 2, one of (G17, G18, G19) - plane selection.
3.  mode 6, one of (G20, G21) - length units.
4.  mode 15 one of (G07,G08) - lathe diameter mode
5.  mode 7, one of (G40, G41, G42) - cutter radius compensation.
6.  mode 8, one of (G43, G49) - tool length offset
7.  mode 12, one of (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3)
    - coordinate system selection.
8.  mode 13, one of (G61, G61.1, G64, G50, G51) - control mode
9.  mode 3, one of (G90, G91) - distance mode.
10.  mode 4, one of (G90.1, G91.1) - arc i,j,k mode.
11. mode 10, one of (G98, G99) - retract mode.
12. mode 0, one of (G10, G28, G30, G92, G92.1, G92.2, G92.3) -
13. mode 1, one of (G0, G1, G2, G3, G38.2, G80, G81 to G89, G33, G33.1, G76) - motion or cancel.
    G53 from mode 0 is also handled here, if present.

Some mode 0 and most mode 1 G-codes must be executed after the length units
are set, since they use coordinate values. Mode 1 codes also must wait
until most of the other modes are set.

*/

int Interp::convert_g(block_pointer block,       //!< pointer to a block of RS274/NGC instructions
		      setup_pointer settings)    //!< pointer to machine settings
{
    int status;

    if ((block->g_modes[GM_MODAL_0] == G_4) && ONCE(STEP_DWELL)) {
      status = convert_dwell(settings, block->p_number);
      CHP(status);
    }
    if ((block->g_modes[GM_SET_PLANE] != -1) && ONCE(STEP_SET_PLANE)) {
	status = convert_set_plane(block->g_modes[GM_SET_PLANE], settings);
	CHP(status);
    }
    if ((block->g_modes[GM_LENGTH_UNITS] != -1) && ONCE(STEP_LENGTH_UNITS)) {
	status = convert_length_units(block->g_modes[GM_LENGTH_UNITS], settings);
	CHP(status);
    }
    if ((block->g_modes[GM_LATHE_DIAMETER_MODE] != -1) && ONCE(STEP_LATHE_DIAMETER_MODE)) {
	status = convert_lathe_diameter_mode(block->g_modes[GM_LATHE_DIAMETER_MODE], block, settings);
	CHP(status);
    }
    if ((block->g_modes[GM_CUTTER_COMP] != -1) && ONCE(STEP_CUTTER_COMP)) {
	status = convert_cutter_compensation(block->g_modes[GM_CUTTER_COMP], block, settings);
	CHP(status);
    }
    if ((block->g_modes[GM_TOOL_LENGTH_OFFSET] != -1) && ONCE(STEP_TOOL_LENGTH_OFFSET)){
	status = convert_tool_length_offset(block->g_modes[GM_TOOL_LENGTH_OFFSET], block, settings);
	CHP(status);
    }
    if ((block->g_modes[GM_COORD_SYSTEM] != -1) && ONCE(STEP_COORD_SYSTEM)){
	status = convert_coordinate_system(block->g_modes[GM_COORD_SYSTEM], settings);
	CHP(status);
    }
    if ((block->g_modes[GM_CONTROL_MODE] != -1) && ONCE(STEP_CONTROL_MODE)) {
	status = convert_control_mode(block->g_modes[GM_CONTROL_MODE],
				      block->p_number, block->q_number, settings);
	CHP(status);
    }
    if ((block->g_modes[GM_DISTANCE_MODE] != -1) && ONCE(STEP_DISTANCE_MODE)) {
      status = convert_distance_mode(block->g_modes[GM_DISTANCE_MODE], settings);
      CHP(status);
    }
    if ((block->g_modes[GM_IJK_DISTANCE_MODE] != -1) && ONCE(STEP_IJK_DISTANCE_MODE)){
	status = convert_ijk_distance_mode(block->g_modes[GM_IJK_DISTANCE_MODE], settings);
	CHP(status);
    }
    if ((block->g_modes[GM_RETRACT_MODE] != -1)  && ONCE(STEP_RETRACT_MODE)){
	status = convert_retract_mode(block->g_modes[GM_RETRACT_MODE], settings);
	CHP(status);
    }
    if ((block->g_modes[GM_MODAL_0] != -1) && ONCE(STEP_MODAL_0)) {
      status = convert_modal_0(block->g_modes[GM_MODAL_0], block, settings);
      CHP(status);
    }
    if ((block->g_modes[GM_G92_IS_APPLIED] != -1) && ONCE(STEP_G92_IS_APPLIED)) {
      status = convert_g92_is_applied(block->g_modes[GM_G92_IS_APPLIED], block, settings);
    }
    if ((block->motion_to_be != -1)  && ONCE(STEP_MOTION)){
      status = convert_motion(block->motion_to_be, block, settings);
      CHP(status);
  }
  return INTERP_OK;
}

/*! convert_savehome

Returned Value: int
   Returns an error if cutter_comp_side is set or the routine is called
   from a gcode other than G_28_1 or G_30_1

Side effects:
   None

Called by: convert_modal_0

Saves the absolute coordinates of the current point in parameters 5161-5169
   for G28.1, or 5181-5189 for G30.1

*/

int Interp::convert_savehome(int code, block_pointer /*block*/, setup_pointer s) {
    double *p = s->parameters;

    if(s->cutter_comp_side != CUTTER_COMP::OFF) {
        ERS(_("Cannot set reference point with cutter compensation in effect"));
    }

    double x = s->current_x + s->axis_offset_x;
    double y = s->current_y + s->axis_offset_y;
    rotate(&x, &y, s->rotation_xy);
    x = PROGRAM_TO_USER_LEN(x + s->tool_offset.tran.x + s->origin_offset_x);
    y = PROGRAM_TO_USER_LEN(y + s->tool_offset.tran.y + s->origin_offset_y);
    double z = PROGRAM_TO_USER_LEN(s->current_z + s->tool_offset.tran.z + s->origin_offset_z + s->axis_offset_z);
    double a = PROGRAM_TO_USER_ANG(s->AA_current + s->tool_offset.a + s->AA_origin_offset + s->AA_axis_offset);
    double b = PROGRAM_TO_USER_ANG(s->BB_current + s->tool_offset.b + s->BB_origin_offset + s->BB_axis_offset);
    double c = PROGRAM_TO_USER_ANG(s->CC_current + s->tool_offset.c + s->CC_origin_offset + s->CC_axis_offset);
    double u = PROGRAM_TO_USER_LEN(s->u_current + s->tool_offset.u + s->u_origin_offset + s->u_axis_offset);
    double v = PROGRAM_TO_USER_LEN(s->v_current + s->tool_offset.v + s->v_origin_offset + s->v_axis_offset);
    double w = PROGRAM_TO_USER_LEN(s->w_current + s->tool_offset.w + s->w_origin_offset + s->w_axis_offset);

    if(s->a_axis_wrapped) {
        a = fmod(a, 360.0);
        if(a<0) a += 360.0;
    }

    if(s->b_axis_wrapped) {
        b = fmod(b, 360.0);
        if(b<0) b += 360.0;
    }

    if(s->c_axis_wrapped) {
        c = fmod(c, 360.0);
        if(c<0) c += 360.0;
    }

    if(code == G_28_1) {
        p[5161] = x;
        p[5162] = y;
        p[5163] = z;
        p[5164] = a;
        p[5165] = b;
        p[5166] = c;
        p[5167] = u;
        p[5168] = v;
        p[5169] = w;
    } else if(code == G_30_1) {
        p[5181] = x;
        p[5182] = y;
        p[5183] = z;
        p[5184] = a;
        p[5185] = b;
        p[5186] = c;
        p[5187] = u;
        p[5188] = v;
        p[5189] = w;
    } else {
        ERS("BUG: Code not G28.1 or G38.1");
    }
    return INTERP_OK;
}


/****************************************************************************/

/*! convert_home

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. cutter radius compensation is on:
      NCE_CANNOT_USE_G28_OR_G30_WITH_CUTTER_RADIUS_COMP
   2. The code is not G28 or G30: NCE_BUG_CODE_NOT_G28_OR_G30

Side effects:
   This executes a straight traverse to the programmed point, using
   the current coordinate system, tool length offset, and motion mode
   to interpret the coordinate values. Then it executes a straight
   traverse to move one or more axes to the location of reference
   point 1 (if G28) or reference point 2 (if G30).  If any axis words
   are specified in this block, only those axes are moved to the
   reference point.  If none are specified, all axes are moved.  It
   also updates the setting of the position of the tool point to the
   end point of the move.

   If either move would affect the position of one or more locking
   rotaries, the rotaries are unlocked and indexed one at a time,
   in the order A,B,C and then the other axes are moved.

   N.B. Many gcode programmers call the reference point a home
   position, and that is exactly what it is if the parameters are
   zero.  Do not confuse this with homing the axis (searching for
   a switch or index pulse).

Called by: convert_modal_0.

*/

int Interp::convert_home(int move,       //!< G-code, must be G_28 or G_30
                        block_pointer block,    //!< pointer to a block of RS274 instructions
                        setup_pointer settings) //!< pointer to machine settings
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end;
  double v_end;
  double w_end;
  double end_x_home;
  double end_y_home;
  double end_z_home;
  double AA_end_home;
  double BB_end_home;
  double CC_end_home;
  double u_end_home;
  double v_end_home;
  double w_end_home;
  double *parameters;

  parameters = settings->parameters;
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end,
                &u_end, &v_end, &w_end));

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
      NCE_CANNOT_USE_G28_OR_G30_WITH_CUTTER_RADIUS_COMP);

  // waypoint is in currently active coordinate system

  // move indexers first, one at a time
  // JOINTS_AXES settings->*_indexer_jnum == -1 means notused
  if (AA_end != settings->AA_current && (-1 != settings->a_indexer_jnum) )
      issue_straight_index(3,settings->a_indexer_jnum, AA_end, block->line_number, settings);
  if (BB_end != settings->BB_current && (-1 != settings->b_indexer_jnum) )
      issue_straight_index(4,settings->b_indexer_jnum, BB_end, block->line_number, settings);
  if (CC_end != settings->CC_current && (-1 != settings->c_indexer_jnum) )
      issue_straight_index(5,settings->c_indexer_jnum, CC_end, block->line_number, settings);

  // Create a state tag and dump it to canon
  write_canon_state_tag(block, settings);

  STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                    AA_end, BB_end, CC_end,
                    u_end, v_end, w_end);

  settings->current_x = end_x;
  settings->current_y = end_y;
  settings->current_z = end_z;
  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u_end;
  settings->v_current = v_end;
  settings->w_current = w_end;

  if (move == G_28) {
      find_relative(USER_TO_PROGRAM_LEN(parameters[5161]),
                    USER_TO_PROGRAM_LEN(parameters[5162]),
                    USER_TO_PROGRAM_LEN(parameters[5163]),
                    USER_TO_PROGRAM_ANG(parameters[5164]),
                    USER_TO_PROGRAM_ANG(parameters[5165]),
                    USER_TO_PROGRAM_ANG(parameters[5166]),
                    USER_TO_PROGRAM_LEN(parameters[5167]),
                    USER_TO_PROGRAM_LEN(parameters[5168]),
                    USER_TO_PROGRAM_LEN(parameters[5169]),
                    &end_x_home, &end_y_home, &end_z_home,
                    &AA_end_home, &BB_end_home, &CC_end_home,
                    &u_end_home, &v_end_home, &w_end_home, settings);
  } else if (move == G_30) {
      find_relative(USER_TO_PROGRAM_LEN(parameters[5181]),
                    USER_TO_PROGRAM_LEN(parameters[5182]),
                    USER_TO_PROGRAM_LEN(parameters[5183]),
                    USER_TO_PROGRAM_ANG(parameters[5184]),
                    USER_TO_PROGRAM_ANG(parameters[5185]),
                    USER_TO_PROGRAM_ANG(parameters[5186]),
                    USER_TO_PROGRAM_LEN(parameters[5187]),
                    USER_TO_PROGRAM_LEN(parameters[5188]),
                    USER_TO_PROGRAM_LEN(parameters[5189]),
                    &end_x_home, &end_y_home, &end_z_home,
                    &AA_end_home, &BB_end_home, &CC_end_home,
                    &u_end_home, &v_end_home, &w_end_home, settings);
  } else
    ERS(NCE_BUG_CODE_NOT_G28_OR_G30);

  // if any axes are specified, home only those axes after the waypoint
  // (both fanuc & haas, contrary to emc historical operation)

  if (block->x_flag) end_x = end_x_home;
  if (block->y_flag) end_y = end_y_home;
  if (block->z_flag) end_z = end_z_home;
  if (block->a_flag) AA_end = AA_end_home;
  if (block->b_flag) BB_end = BB_end_home;
  if (block->c_flag) CC_end = CC_end_home;
  if (block->u_flag) u_end = u_end_home;
  if (block->v_flag) v_end = v_end_home;
  if (block->w_flag) w_end = w_end_home;

  // but, if no axes are specified, home all of them
  // (haas does this, emc historical did, throws an error in fanuc)

  if (!block->x_flag && !block->y_flag && !block->z_flag &&
      !block->a_flag && !block->b_flag && !block->c_flag &&
      !block->u_flag && !block->v_flag && !block->w_flag) {
      end_x = end_x_home;
      end_y = end_y_home;
      end_z = end_z_home;
      AA_end = AA_end_home;
      BB_end = BB_end_home;
      CC_end = CC_end_home;
      u_end = u_end_home;
      v_end = v_end_home;
      w_end = w_end_home;
  }

  // move indexers first, one at a time
  // JOINTS_AXES settings->*_indexer_jnum == -1 means notused
  if (AA_end != settings->AA_current && (-1 != settings->a_indexer_jnum) )
      issue_straight_index(3,settings->a_indexer_jnum, AA_end, block->line_number, settings);
  if (BB_end != settings->BB_current && (-1 != settings->b_indexer_jnum) )
      issue_straight_index(4,settings->b_indexer_jnum, BB_end, block->line_number, settings);
  if (CC_end != settings->CC_current && (-1 != settings->c_indexer_jnum) )
      issue_straight_index(5,settings->c_indexer_jnum, CC_end, block->line_number, settings);

  STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                    AA_end, BB_end, CC_end,
                    u_end, v_end, w_end);
  settings->current_x = end_x;
  settings->current_y = end_y;
  settings->current_z = end_z;
  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u_end;
  settings->v_current = v_end;
  settings->w_current = w_end;

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_length_units

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The g_code argument isn't G_20 or G_21:
      NCE_BUG_CODE_NOT_G20_OR_G21
   2. Cutter radius compensation is on:
      NCE_CANNOT_CHANGE_UNITS_WITH_CUTTER_RADIUS_COMP

Side effects:
   A command setting the length units is executed. The machine
   settings are reset regarding length units and current position.

Called by: convert_g.

Tool length offset and diameter, work coordinate systems, feed rate,
g28/g30 home positions, and g53 motion in absolute coordinates all work
properly after switching units.  Historically these had problems but the
intention is that they work properly now.

The tool table in settings is not converted here; it is always in
inifile units and the conversion happens when reading an entry.  Tool
offsets and feed rate that are in effect are converted by rereading them
from the canon level.

Cutter diameter is not converted because radius comp is not in effect
when we are changing units.

XXX Other distance items in the settings (such as the various parameters
for cycles) need testing.

*/

int Interp::convert_length_units(int g_code,     //!< g_code being executed (must be G_20 or G_21)
                                setup_pointer settings) //!< pointer to machine settings
{
  if (g_code == G_20) {
    USE_LENGTH_UNITS(CANON_UNITS_INCHES);
    if (settings->length_units != CANON_UNITS_INCHES) {
      settings->length_units = CANON_UNITS_INCHES;
      settings->current_x = (settings->current_x * INCH_PER_MM);
      settings->current_y = (settings->current_y * INCH_PER_MM);
      settings->current_z = (settings->current_z * INCH_PER_MM);
      settings->program_x = (settings->program_x * INCH_PER_MM);
      settings->program_y = (settings->program_y * INCH_PER_MM);
      settings->program_z = (settings->program_z * INCH_PER_MM);
      qc_scale(INCH_PER_MM);
      settings->cutter_comp_radius *= INCH_PER_MM;
      settings->axis_offset_x = (settings->axis_offset_x * INCH_PER_MM);
      settings->axis_offset_y = (settings->axis_offset_y * INCH_PER_MM);
      settings->axis_offset_z = (settings->axis_offset_z * INCH_PER_MM);
      settings->origin_offset_x = (settings->origin_offset_x * INCH_PER_MM);
      settings->origin_offset_y = (settings->origin_offset_y * INCH_PER_MM);
      settings->origin_offset_z = (settings->origin_offset_z * INCH_PER_MM);

      settings->u_current = (settings->u_current * INCH_PER_MM);
      settings->v_current = (settings->v_current * INCH_PER_MM);
      settings->w_current = (settings->w_current * INCH_PER_MM);
      settings->u_axis_offset = (settings->u_axis_offset * INCH_PER_MM);
      settings->v_axis_offset = (settings->v_axis_offset * INCH_PER_MM);
      settings->w_axis_offset = (settings->w_axis_offset * INCH_PER_MM);
      settings->u_origin_offset = (settings->u_origin_offset * INCH_PER_MM);
      settings->v_origin_offset = (settings->v_origin_offset * INCH_PER_MM);
      settings->w_origin_offset = (settings->w_origin_offset * INCH_PER_MM);

      settings->tool_offset.tran.x = GET_EXTERNAL_TOOL_LENGTH_XOFFSET();
      settings->tool_offset.tran.y = GET_EXTERNAL_TOOL_LENGTH_YOFFSET();
      settings->tool_offset.tran.z = GET_EXTERNAL_TOOL_LENGTH_ZOFFSET();
      settings->tool_offset.a = GET_EXTERNAL_TOOL_LENGTH_AOFFSET();
      settings->tool_offset.b = GET_EXTERNAL_TOOL_LENGTH_BOFFSET();
      settings->tool_offset.c = GET_EXTERNAL_TOOL_LENGTH_COFFSET();
      settings->tool_offset.u = GET_EXTERNAL_TOOL_LENGTH_UOFFSET();
      settings->tool_offset.v = GET_EXTERNAL_TOOL_LENGTH_VOFFSET();
      settings->tool_offset.w = GET_EXTERNAL_TOOL_LENGTH_WOFFSET();
      settings->feed_rate = GET_EXTERNAL_FEED_RATE();
      settings->tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
      settings->naivecam_tolerance =
	  GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
    }
  } else if (g_code == G_21) {
    USE_LENGTH_UNITS(CANON_UNITS_MM);
    if (settings->length_units != CANON_UNITS_MM) {
      settings->length_units = CANON_UNITS_MM;
      settings->current_x = (settings->current_x * MM_PER_INCH);
      settings->current_y = (settings->current_y * MM_PER_INCH);
      settings->current_z = (settings->current_z * MM_PER_INCH);
      settings->program_x = (settings->program_x * MM_PER_INCH);
      settings->program_y = (settings->program_y * MM_PER_INCH);
      settings->program_z = (settings->program_z * MM_PER_INCH);
      qc_scale(MM_PER_INCH);
      settings->cutter_comp_radius *= MM_PER_INCH;
      settings->axis_offset_x = (settings->axis_offset_x * MM_PER_INCH);
      settings->axis_offset_y = (settings->axis_offset_y * MM_PER_INCH);
      settings->axis_offset_z = (settings->axis_offset_z * MM_PER_INCH);
      settings->origin_offset_x = (settings->origin_offset_x * MM_PER_INCH);
      settings->origin_offset_y = (settings->origin_offset_y * MM_PER_INCH);
      settings->origin_offset_z = (settings->origin_offset_z * MM_PER_INCH);

      settings->u_current = (settings->u_current * MM_PER_INCH);
      settings->v_current = (settings->v_current * MM_PER_INCH);
      settings->w_current = (settings->w_current * MM_PER_INCH);
      settings->u_axis_offset = (settings->u_axis_offset * MM_PER_INCH);
      settings->v_axis_offset = (settings->v_axis_offset * MM_PER_INCH);
      settings->w_axis_offset = (settings->w_axis_offset * MM_PER_INCH);
      settings->u_origin_offset = (settings->u_origin_offset * MM_PER_INCH);
      settings->v_origin_offset = (settings->v_origin_offset * MM_PER_INCH);
      settings->w_origin_offset = (settings->w_origin_offset * MM_PER_INCH);

      settings->tool_offset.tran.x = GET_EXTERNAL_TOOL_LENGTH_XOFFSET();
      settings->tool_offset.tran.y = GET_EXTERNAL_TOOL_LENGTH_YOFFSET();
      settings->tool_offset.tran.z = GET_EXTERNAL_TOOL_LENGTH_ZOFFSET();
      settings->tool_offset.a = GET_EXTERNAL_TOOL_LENGTH_AOFFSET();
      settings->tool_offset.b = GET_EXTERNAL_TOOL_LENGTH_BOFFSET();
      settings->tool_offset.c = GET_EXTERNAL_TOOL_LENGTH_COFFSET();
      settings->tool_offset.u = GET_EXTERNAL_TOOL_LENGTH_UOFFSET();
      settings->tool_offset.v = GET_EXTERNAL_TOOL_LENGTH_VOFFSET();
      settings->tool_offset.w = GET_EXTERNAL_TOOL_LENGTH_WOFFSET();
      settings->feed_rate = GET_EXTERNAL_FEED_RATE();
      settings->tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
      settings->naivecam_tolerance =
	  GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G20_OR_G21);
  return INTERP_OK;
}


/*
 * given two int arrays and two double arrays representing interpreter
 * settings as stored in _setup.active_g_codes and
 * _setup.active_settings, construct a G-code sequence to synchronize
 * their state.
 */
int Interp::gen_settings(
    int *int_current, int *int_saved,            // G-codes
    double * /*float_current*/, double *float_saved,  // S, F, other
    std::string &cmd)                            // command buffer
{
    FORCE_LC_NUMERIC_C;
    int i, val;
    int g64_changed = 0;
    char buf[LINELEN];

    // F, S
    for (i = 0; i < ACTIVE_SETTINGS; i++) {
	// "if" masked to address https://github.com/LinuxCNC/linuxcnc/issues/1987
	// The setting value is correct, but seems to be mislaid downstream
	//if (float_saved[i] != float_current[i]) {
	    switch (i) {
	    case GM_FIELD_FLOAT_LINE_NUMBER:
		// sequence_number - no point in restoring
		break;
	    case GM_FIELD_FLOAT_FEED:
		snprintf(buf,sizeof(buf)," F%.1f", float_saved[i]);
                cmd += buf;
		break;
	    case GM_FIELD_FLOAT_SPEED:
		snprintf(buf,sizeof(buf)," S%.0f", float_saved[i]);
                cmd += buf;
		break;
	    case GM_FIELD_FLOAT_PATH_TOLERANCE:
	    case GM_FIELD_FLOAT_NAIVE_CAM_TOLERANCE:
		// G64 special case; see below
		g64_changed = 1;
		break;
	   }
	//}
    }

    // G-codes
    for (i = 0; i < ACTIVE_G_CODES; i++) {
	val = int_saved[i];
	if (val != int_current[i]) {

	    switch (i) {
	    case 0:
		// // sequence_number - no point in restoring
		break;
	    case 2: // FIXME - I dont understand this:
		//   gez[2] = ((block == NULL) ? -1 : block->g_modes[0]);
		break;
	    case 12:  // mystery slot
		break;
	    case 5: // - length units
		// this is treated before all others - see convert_m()
		break;
	    case 1: // - motion_mode
		// restoring the motion mode is a real bad idea to start with
		break;
	    case 3: // - plane
	    case 4: // - cutter compensation
	    case 6: // - distance mode
	    case 7: // - feed mode
	    case 8: // - coordinate system
	    case 9: // - tool offset (G43/G49)
	    case 10: // - retract mode
	    case 13: // - spindle mode
	    case 14: // - ijk distance mode
	    case 15: // - lathe diameter mode
            case 16: // - whether g92 is applied

		if (val != -1) { // FIXME not sure if this is correct!
		    // if this was set in sub, and unset in caller, it will
		    // not be reset
		    if (val % 10) {
			snprintf(buf,sizeof(buf)," G%d.%d", val / 10, val % 10);
		    } else {
			snprintf(buf,sizeof(buf)," G%d", val / 10);
		    }
                    cmd += buf;
		} else {
		    // so complain rather loudly
		    MSG("------ gen_settings BUG: index %d = -1!!\n",i);
		}
		break;
	    case 11: // - control mode
		// Special case, since G64 requires P and Q flags
		// FIXME what about when P or Q changes, even though still G64?
		if (val == G_64)
		    // G64 special case; see below
		    g64_changed = 1;
		else if (val == G_61) {
		    snprintf(buf,sizeof(buf)," G61");
		    cmd += buf;
		} else if (val == G_61_1) {
		    snprintf(buf,sizeof(buf)," G61.1");
		    cmd += buf;
		} else
		    MSG("------ gen_settings BUG: index %d = -1!!\n",i);
		break;
	    }
	}
    }


    // Special case for restoring G64:  `cmd` may contain a `G64` if
    // current int settings is `G61` or if current float settings
    // contain different `G64 P* Q*` args; in these cases, only add a
    // single `G64` command
    if ((int_saved[11] == G_64) && g64_changed) {
	if (float_saved[GM_FIELD_FLOAT_PATH_TOLERANCE] < 0)
	    // No P, Q args
	    snprintf(buf,sizeof(buf)," G64");
	else if (
	    float_saved[GM_FIELD_FLOAT_NAIVE_CAM_TOLERANCE] < 0)
	    // Only P arg
	    snprintf(buf,sizeof(buf)," G64 P%f",
		     float_saved[GM_FIELD_FLOAT_PATH_TOLERANCE]);
	else  // Both P, Q args
	    snprintf(
		buf,sizeof(buf)," G64 P%f Q%f",
		float_saved[GM_FIELD_FLOAT_PATH_TOLERANCE],
		float_saved[GM_FIELD_FLOAT_NAIVE_CAM_TOLERANCE]);
	cmd += buf;
    }
    return INTERP_OK;
}

/*
 * given two int arrays representing interpreter settings as stored in
 * _setup.active_m_codes, construct a M-code sequence to synchronize their state.
 *
 * use multiple lines here because M7 and M8 may not be on the same line since
 * they are in the same modal group.
 */
int Interp::gen_m_codes(int *current, int *saved, std::string &cmd)
{
    FORCE_LC_NUMERIC_C;
    int i,val;
    char buf[LINELEN];
    for (i = 0; i < ACTIVE_M_CODES; i++) {
	val = saved[i];
	if (val != current[i]) {
	    switch (i) {
	    case 0: /* 0 seq number  */
		break;
	    case 1: /* 1 stopping    */
		// FIXME - is the next line needed at all?
		// emz[1] = (block == NULL) ? -1 : block->m_modes[4];
		break;
	    case 3: /* 3 tool change */
		// FIXME - dont know how to handle this.
		// emz[3] =
		//  (block == NULL) ? -1 : block->m_modes[6];
		break;
	    case 2: // spindle
	    case 4: // mist
	    case 5: // flood
	    case 6: // speed/feed override
	    case 7: // adaptive feed
	    case 8: // feed hold
		if (val != -1) {  // unsure..
		    snprintf(buf,sizeof(buf),"M%d\n", val);
		    cmd += buf;
		} else {
		    MSG("------ gen_m_codes: index %d = -1!!\n",i);
		}
		break;
	    }
	}
    }
    return INTERP_OK;
}


/**
 * Create a G-code string to restore a modal state on program abort.
 * Note: This is only designed to be called on program abort. It restores the
 * modal state in the interpreter to the equivalent state at the most recent
 * motion line, but does not restore M codes.
 */
int Interp::gen_restore_cmd(int *current_g,
			    int * /*current_m*/,
			    double *current_settings,
			    StateTag const &saved,
			    std::string &cmd)
{
    int res;
    // A local copy of the saved settings, unpacked from a state tag
    int saved_g[ACTIVE_G_CODES];
    int saved_m[ACTIVE_M_CODES];
    double saved_settings[ACTIVE_SETTINGS];

    //Extract saved state to local vectors
    // FIXME Use saved_settings to store state tag floats, incl. in
    int res_unpack = active_modes(saved_g, saved_m, saved_settings, saved);
    if (res_unpack != INTERP_OK) {
	logStateTags("gen_restore_cmd() error %d:  failed to unpack state tag",
		     res_unpack);
        return INTERP_ERROR;
    }

    /* Now we clean up any G / M modes that should be reset when ending a
     * program. Note that most of these mode states are based on logic in
     * write_XXX functions.
     */
    // Force cancellation of tool compensation
    saved_g[4] = G_40;

    //Mimic the order of restoration commands used elsewhere
    if (current_g[5] != saved_g[5]) {
        char buf[LINELEN];
        snprintf(buf,sizeof(buf), "G%d",saved_g[5]/10);
        CHKS(execute(buf) != INTERP_OK, _("gen_restore G20/G21 failed: '%s'"),
	     cmd.c_str());
    }

    if ((res = gen_settings(
	     current_g, saved_g, current_settings, saved_settings, cmd))) {
	logStateTags("gen_restore_cmd():  error restoring settings (%d)",
		     res);
        return INTERP_ERROR;
    }
    // M codes should not be restored during an abort with gen_m_codes()

    return INTERP_OK;
}


int Interp::save_settings(setup_pointer settings)
{
      // the state is sprinkled all over _setup
      // collate state in _setup.active_* arrays
      write_g_codes((block_pointer) NULL, settings);
      write_m_codes((block_pointer) NULL, settings);
      write_settings(settings);
      write_state_tag((block_pointer) NULL, settings, settings->state_tag);

      // save in the current call frame
      active_g_codes((int *)settings->sub_context[settings->call_level].saved_g_codes);
      active_m_codes((int *)settings->sub_context[settings->call_level].saved_m_codes);
      active_settings((double *)settings->sub_context[settings->call_level].saved_settings);

      // TBD: any other state deemed important to save/restore should be added here
      // context_struct might need to be extended too.

      return INTERP_OK;
}

/* restore global settings/gcodes/mcodes to current call level from a valid context
 * used by:
 *   M72 - restores context from same level
 *       example: restore_settings(settings->call_level)
 *
 *   an o-word return/endsub if auto-restore (M73) was issued
 *       issue this like so - after call_level has been decremented:
 *       restore_settings(settings->call_level + 1)
 */
int Interp::restore_settings(setup_pointer settings,
			    int from_level)    //!< call level of context to restore from
{

    CHKS((from_level < settings->call_level),
	 (_("BUG: cannot restore from a lower call level (%d) to a higher call level (%d)")),from_level,settings->call_level);
    CHKS((from_level < 0), (_("BUG: restore from level %d !?")),from_level);
    CHKS((settings->call_level < 0), (_("BUG: restore to level %d !?")),settings->call_level);

    // linearize state
    write_g_codes((block_pointer) NULL, settings);
    write_m_codes((block_pointer) NULL, settings);
    write_settings(settings);

    std::string cmd;

    // construct gcode from the state difference and execute
    // this assures appropriate canon commands are generated if needed -
    // just restoring interp variables is not enough

    // G20/G21 switching is special - it is executed beforehand
    // so restoring feed lateron is interpreted in the correct context

    if (settings->active_g_codes[5] != settings->sub_context[from_level].saved_g_codes[5]) {
        char buf[LINELEN];
	snprintf(buf,sizeof(buf), "G%d",settings->sub_context[from_level].saved_g_codes[5]/10);
	CHKS(execute(buf) != INTERP_OK, _("M7x: restore_settings G20/G21 failed: '%s'"), cmd.c_str());
    }
    gen_settings(
	(int *)settings->active_g_codes,
	(int *)settings->sub_context[from_level].saved_g_codes,
	(double *)settings->active_settings,
	(double *)settings->sub_context[from_level].saved_settings,
	cmd);
    gen_m_codes(
	(int *) settings->active_m_codes,
	(int *)settings->sub_context[from_level].saved_m_codes,
	cmd);

    if (!cmd.empty()) {
	// the sequence can be multiline, separated by nl
	// so split and execute each line
	std::string cpy = cmd;
	char *stateptr = NULL;
	char *s = strtok_r(cpy.data(), "\n", &stateptr);
	while (s != NULL) {
	    int status = execute(s);
	    if (status != INTERP_OK) {
		char currentError[LINELEN+1];
		rtapi_strxcpy(currentError,getSavedError());
		CHKS(status, _("M7x: restore_settings failed executing: '%s': %s"), s, currentError);
	    }
	    s = strtok_r(NULL, "\n", &stateptr);
	}
	write_g_codes((block_pointer) NULL, settings);
	write_m_codes((block_pointer) NULL, settings);
	write_settings(settings);
    }

    // TBD: any state deemed important to restore should be restored here
    // NB: some state changes might generate canon commands so do that here
    // if needed

    return INTERP_OK;
}


/**
 * Variation of restore_settings to pull state from a StateTag.
 */
int Interp::restore_from_tag(StateTag const &tag)
{

    if (!tag.is_valid() || !tag.flags[GM_FLAG_RESTORABLE]) {
        //Invalid line implies a bad tag, don't restore
	logStateTags("restore_from_tag() error:  Invalid tag");
	print_state_tag(tag);
        return INTERP_ERROR;
    }

    // clear queue buster sflags, otherwise the command won't be
    // executed - Tormach *dpr 8/17/15
    _setup.input_flag = false;
    _setup.toolchange_flag = false;
    _setup.probe_flag = false;

    // linearize state
    write_g_codes((block_pointer) NULL, &_setup);
    write_m_codes((block_pointer) NULL, &_setup);
    write_settings(&_setup);

    std::string cmd;

    // construct gcode from the state difference and execute
    // this assures appropriate canon commands are generated if needed -
    // just restoring interp variables is not enough

    int res_unpack = gen_restore_cmd((int *) _setup.active_g_codes,
				     (int *) _setup.active_m_codes,
				     (double *) _setup.active_settings,
				     tag,
				     cmd);
    if (res_unpack != INTERP_OK) {
	logStateTags("restore_from_tag() failed to generate restore command");
	print_state_tag(tag);
        return res_unpack;
    }

    if (!cmd.empty()) {
        // the sequence can be multiline, separated by nl
        // so split and execute each line
        std::string cpy = cmd;
        char *stateptr = NULL;
        char *s = strtok_r(cpy.data(), "\n", &stateptr);
        while (s != NULL) {
            int status = execute(s);
            if (status != INTERP_OK) {
                char currentError[LINELEN+1];
                strcpy(currentError,getSavedError());
                CHKS(status, _("Failed to restore interp state on abort "
			       "'%s': %s"), s, currentError);
            }
            s = strtok_r(NULL, "\n", &stateptr);
        }
        write_g_codes((block_pointer) NULL, &_setup);
        write_m_codes((block_pointer) NULL, &_setup);
        write_settings(&_setup);
	logStateTags("Restored program pre-abort state from tag:  |%s|",
		     cmd.c_str());
    } else {
	logStateTags("Program pre-abort state unchanged; not restoring:");
	print_state_tag(tag);
    }

    return INTERP_OK;
}



/****************************************************************************/

/*! convert_m

Returned Value: int
   If convert_tool_change returns an error code, this returns that code.
   If input-related stuff is needed, it sets the flag input_flag = true.
   Otherwise, it returns INTERP_OK.

Side effects:
   m_codes in the block are executed. For each m_code
   this consists of making a function call(s) to a canonical machining
   function(s) and setting the machine model.

Called by: execute_block.

This handles four separate types of activity in order:
1. changing the tool (m6) - which also retracts and stops the spindle.
2. Turning the spindle on or off (m3, m4, and m5)
3. Turning coolant on and off (m7, m8, and m9)
4. turning a-axis clamping on and off (m26, m27) - commented out.
5. enabling or disabling feed and speed overrides (m49, m49).
6. changing the loaded toolnumber (m61).
Within each group, only the first code encountered will be executed.

This does nothing with m0, m1, m2, m30, or m60 (which are handled in
convert_stop).

*/

int Interp::convert_m(block_pointer block,       //!< pointer to a block of RS274/NGC instructions
		      setup_pointer settings)    //!< pointer to machine settings
{
  int type;
  double timeout;               // timeout for M66

  /* The M62-65 commands are used for DIO */
  /* M62 sets a DIO synched with motion
     M63 clears a DIO synched with motion
     M64 sets a DIO immediately
     M65 clears a DIO immediately
     M66 waits for an input
     M67 reads a digital input
     M68 reads an analog input*/

  if (is_user_defined_m_code(block, settings, 5) &&
      STEP_REMAPPED_IN_BLOCK(block, STEP_M_5) &&
      ONCE_M(5))  {
      return convert_remapped_code(block, settings, STEP_M_5, 'm',
				   block->m_modes[5]);
  } else if ((block->m_modes[5] == 62) && ONCE_M(5)) {
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set motion output with cutter radius compensation on")));  // XXX
      CHKS((!block->p_flag), _("No valid P word with M62"));
      SET_MOTION_OUTPUT_BIT(round_to_int(block->p_number));
  } else if ((block->m_modes[5] == 63) && ONCE_M(5)) {
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set motion digital output with cutter radius compensation on")));  // XXX
      CHKS((!block->p_flag), _("No valid P word with M63"));
      CLEAR_MOTION_OUTPUT_BIT(round_to_int(block->p_number));
  } else if ((block->m_modes[5] == 64) && ONCE_M(5)){
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set auxiliary digital output with cutter radius compensation on")));  // XXX
      CHKS((!block->p_flag), _("No valid P word with M64"));
      SET_AUX_OUTPUT_BIT(round_to_int(block->p_number));
  } else if ((block->m_modes[5] == 65) && ONCE_M(5)) {
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set auxiliary digital output with cutter radius compensation on")));  // XXX
      CHKS((!block->p_flag), _("No valid P word with M65"));
      CLEAR_AUX_OUTPUT_BIT(round_to_int(block->p_number));
  } else if ((block->m_modes[5] == 66) && ONCE_M(5)){

    //P-word = digital channel
    //E-word = analog channel
    //L-word = wait type (immediate, rise, fall, high, low)
    //Q-word = timeout
    // it is an error if:

    // P and E word are specified together
    CHKS(((block->p_flag) && (block->e_flag)),
	NCE_BOTH_DIGITAL_AND_ANALOG_INPUT_SELECTED);

    // L-word not 0, and timeout <= 0
    CHKS(((block->q_number <= 0) && (block->l_flag) && (round_to_int(block->l_number) > 0)),
	NCE_ZERO_TIMEOUT_WITH_WAIT_NOT_IMMEDIATE);

    // E-word specified (analog input) and wait type not immediate
    CHKS(((block->e_flag) && (block->l_flag) && (round_to_int(block->l_number) != 0)),
	NCE_ANALOG_INPUT_WITH_WAIT_NOT_IMMEDIATE);

    // missing P or E (or invalid = negative)
    CHKS( ((block->p_flag) && (round_to_int(block->p_number) < 0)) ||
         ((block->e_flag) && (round_to_int(block->e_number) < 0)) ||
	 ((!block->p_flag) && (!block->e_flag)) ,
	NCE_INVALID_OR_MISSING_P_AND_E_WORDS_FOR_WAIT_INPUT);

    write_canon_state_tag(block, settings);
    if (block->p_flag) { // got a digital input
	if (round_to_int(block->p_number) < 0) // safety check for negative words
	    ERS(_("invalid P-word with M66"));

	if (block->l_flag) {
	    type = round_to_int(block->l_number);
	} else {
	    type = WAIT_MODE_IMMEDIATE;
        }

	if (block->q_number > 0) {
	    timeout = block->q_number;
	} else {
	    timeout = 0;
        }

        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot wait for digital input with cutter radius compensation on")));

	int ret = WAIT(round_to_int(block->p_number), DIGITAL_INPUT, type, timeout);
	//WAIT returns 0 on success, -1 for out of bounds
	CHKS((ret == -1), NCE_DIGITAL_INPUT_INVALID_ON_M66);
	if (ret == 0) {
	    settings->input_flag = true;
	    settings->input_index = round_to_int(block->p_number);
	    settings->input_digital = true;
	}
    } else if (round_to_int(block->e_number) >= 0) { // got an analog input
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot wait for analog input with cutter radius compensation on")));

	int ret = WAIT(round_to_int(block->e_number), ANALOG_INPUT, 0, 0); //WAIT returns 0 on success, -1 for out of bounds
	CHKS((ret == -1), NCE_ANALOG_INPUT_INVALID_ON_M66);
	if (ret == 0) {
	    settings->input_flag = true;
	    settings->input_index = round_to_int(block->e_number);
	    settings->input_digital = false;
	}
    }
  } else if ((block->m_modes[5] == 67) && ONCE_M(5)) {

    //E-word = analog channel
    //Q-word = analog value
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set motion analog output with cutter radius compensation on")));  // XXX
      CHKS((!block->e_flag) || (round_to_int(block->e_number) < 0), (_("Invalid analog index with M67")));
      SET_MOTION_OUTPUT_VALUE(round_to_int(block->e_number), block->q_number);
  } else if ((block->m_modes[5] == 68)  && ONCE_M(5)) {
    //E-word = analog channel
    //Q-word = analog value
      CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
           (_("Cannot set auxiliary analog output with cutter radius compensation on")));  // XXX
      CHKS((!block->e_flag) || (round_to_int(block->e_number) < 0), (_("Invalid analog index with M68")));
      SET_AUX_OUTPUT_VALUE(round_to_int(block->e_number), block->q_number);
  }

  if ((block->m_modes[6] != -1)  && ONCE_M(6)){
      int toolno;
      bool remapped_in_block = STEP_REMAPPED_IN_BLOCK(block, STEP_M_6);

      switch (block->m_modes[6]) {
      case 6:
      	  if (is_user_defined_m_code(block, settings, 6) && remapped_in_block) {
      		  return convert_remapped_code(block,settings,
      					       STEP_M_6,
      					       'm',
      					       block->m_modes[6]);
	  } else {
	      // the code was used in its very remap procedure -
	      // the 'recursion case'; record the fact
	      CONTROLLING_BLOCK(*settings).builtin_used = !remapped_in_block;
	      CHP(convert_tool_change(settings));
	  }
      	  break;

      case 61:
	  if (is_user_defined_m_code(block, settings, 6) && remapped_in_block) {
	      return convert_remapped_code(block, settings, STEP_M_6,'m',
					   block->m_modes[6]);
	  } else {
	      CONTROLLING_BLOCK(*settings).builtin_used = !remapped_in_block;
	      toolno = round_to_int(block->q_number);
	      // now also accept M61 Q0 - unload tool
	      CHKS((toolno < 0), (_("Need non-negative Q-word to specify tool number with M61")));

	      int idx;

	      // make sure selected tool exists
	      CHP((find_tool_index(settings, toolno, &idx)));
	      settings->current_pocket = idx;
	      settings->toolchange_flag = true;
	      CHANGE_TOOL_NUMBER(settings->current_pocket);
	      set_tool_parameters();
	  }
	  break;

      default:
	  if (is_user_defined_m_code(block, settings, 6)) {
	      return convert_remapped_code(block, settings, STEP_M_6,'m',
					   block->m_modes[6]);
	  }
      }
  }

  if (FEATURE(RETAIN_G43)) {

      if ((settings->active_g_codes[9] == G_43) && ONCE(STEP_RETAIN_G43)) {
        if(settings->selected_pocket > 0) {
            struct block_struct g43;
            init_block(&g43);
            block->g_modes[gees[G_43]] = G_43;
            CHP(convert_tool_length_offset(G_43, &g43, settings));
        } else {
            struct block_struct g49;
            init_block(&g49);
            block->g_modes[gees[G_49]] = G_49;
            CHP(convert_tool_length_offset(G_49, &g49, settings));
        }
    }
  }

 if (is_user_defined_m_code(block, settings, 7) && ONCE_M(7)) {
    return convert_remapped_code(block, settings, STEP_M_7, 'm',
				   block->m_modes[7]);
 } else if ((block->m_modes[7] == 3)  && ONCE_M(7)) {
     if (block->dollar_flag){
        CHKS((block->dollar_number >= settings->num_spindles || block->dollar_number < -1),
            (_("Spindle ($) number out of range in M3 Command\nnum_spindles =%i. $=%d\n")),settings->num_spindles,(int)block->dollar_number);
        if (block->dollar_number == -1){ // all spindles
            for (int i = 0; i < settings->num_spindles; i++){
                enqueue_START_SPINDLE_CLOCKWISE(i);
                settings->spindle_turning[i] = CANON_CLOCKWISE;
            }
        } else { // a specific spindle
            enqueue_START_SPINDLE_CLOCKWISE(block->dollar_number);
            settings->spindle_turning[(int)block->dollar_number] = CANON_CLOCKWISE;
        }
     } else { // the default spindle
        enqueue_START_SPINDLE_CLOCKWISE(0);
        settings->spindle_turning[0] = CANON_CLOCKWISE;
     }
 } else if ((block->m_modes[7] == 4) && ONCE_M(7)) {
     if (block->dollar_flag){
        CHKS((block->dollar_number >= settings->num_spindles || block->dollar_number < -1),
            (_("Spindle ($) number out of range in M4 Command\nnum_spindles =%i. $=%d\n")),settings->num_spindles,(int)block->dollar_number);
        if (block->dollar_number == -1){ // all spindles
            for (int i = 0; i < settings->num_spindles; i++){
                 enqueue_START_SPINDLE_COUNTERCLOCKWISE(i);
                 settings->spindle_turning[i] = CANON_COUNTERCLOCKWISE;
             }
         } else { // a specific spindle
            enqueue_START_SPINDLE_COUNTERCLOCKWISE(block->dollar_number);
            settings->spindle_turning[(int)block->dollar_number] = CANON_COUNTERCLOCKWISE;
        }
     } else { // default spindle
         enqueue_START_SPINDLE_COUNTERCLOCKWISE(0);
         settings->spindle_turning[0] = CANON_COUNTERCLOCKWISE;
     }
 } else if ((block->m_modes[7] == 5) && ONCE_M(7)){
    if (block->dollar_flag){
        CHKS((block->dollar_number >= settings->num_spindles || block->dollar_number < -1),
            (_("Spindle ($) number out of range in M5 Command\nnum_spindles =%i. $=%d\n")),settings->num_spindles,(int)block->dollar_number);
        if (block->dollar_number == -1){ // all spindles
            for (int i = 0; i < settings->num_spindles; i++){
                settings->spindle_turning[i] = CANON_STOPPED;
                enqueue_STOP_SPINDLE_TURNING(i);
            }
        } else { // a specific spindle
            settings->spindle_turning[block->dollar_number] = CANON_STOPPED;
            enqueue_STOP_SPINDLE_TURNING(block->dollar_number);
        }
    } else { // the default spindle
      for (int i = 0; i < settings->num_spindles; i++){
        settings->spindle_turning[i] = CANON_STOPPED;
        enqueue_STOP_SPINDLE_TURNING(i);
      }
    }
  } else if ((block->m_modes[7] == 19) && ONCE_M(7)) {
      for (int i = 0; i < settings->num_spindles; i++)
          settings->spindle_turning[i] = CANON_STOPPED;
      if (block->dollar_flag){
         CHKS((block->dollar_number >= settings->num_spindles || block->dollar_number < 0),
             (_("Spindle ($) number out of range in M19 Command")));
      }
      if (block->r_flag || block->p_flag)
      enqueue_ORIENT_SPINDLE(block->dollar_flag ? block->dollar_number : 0,
                             block->r_flag ? (block->r_number + settings->orient_offset) : settings->orient_offset,
                             block->p_flag ? block->p_number : 0);
      if (block->q_flag) {
	  CHKS((block->q_number <= 0.0),(_("Q word with M19 requires a value > 0")));
	  enqueue_WAIT_ORIENT_SPINDLE_COMPLETE(block->dollar_flag ? block->dollar_number : 0,
			  	  	  	  	  	  	  	   block->q_number);
      }
  } else if ((block->m_modes[7] == 70) || (block->m_modes[7] == 73)) {

     // save state in current stack frame. We borrow the o-word call stack
     // and extend it to hold modes & settings.
     save_settings(&_setup);

     // flag this frame as containing a valid context
     _setup.sub_context[_setup.call_level].context_status |= CONTEXT_VALID;

     // mark as auto-restore context
     if (block->m_modes[7] == 73) {
	 if (_setup.call_level == 0) {
	     MSG("Warning - M73 at top level: nothing to return to; storing context anyway\n");
	 } else {
	     _setup.sub_context[_setup.call_level].context_status |= CONTEXT_RESTORE_ON_RETURN;
	 }
     }
 } else if ((block->m_modes[7] == 71) && ONCE_M(7))  {
      // M72 - invalidate context at current level
      _setup.sub_context[_setup.call_level].context_status &= ~CONTEXT_VALID;

 } else if ((block->m_modes[7] == 72)  && ONCE_M(7)) {

      // restore state from current stack frame.
      CHKS((!(_setup.sub_context[_setup.call_level].context_status & CONTEXT_VALID)),
           (_("Cannot restore context from invalid stack frame - missing M70/M73?")));
      CHP(restore_settings(&_setup, _setup.call_level));
  }

  if (is_user_defined_m_code(block, settings, 8) &&
	  STEP_REMAPPED_IN_BLOCK(block, STEP_M_8) && ONCE_M(8))  {
      return convert_remapped_code(block, settings, STEP_M_8, 'm', block->m_modes[8]);
  } else if ((block->m_modes[8] == 7) && ONCE_M(8)){
      enqueue_MIST_ON();
      settings->mist = true;
  } else if ((block->m_modes[8] == 8) && ONCE_M(8)) {
      enqueue_FLOOD_ON();
      settings->flood = true;
  } else if ((block->m_modes[8] == 9) && ONCE_M(8)) {
      enqueue_MIST_OFF();
      settings->mist = false;
      enqueue_FLOOD_OFF();
      settings->flood = false;
  }

/* No axis clamps in this version
  if (block->m_modes[2] == 26)
    {
#ifdef DEBUG_EMC
      COMMENT("interpreter: automatic A-axis clamping turned on");
#endif
      settings->a_axis_clamping = true;
    }
  else if (block->m_modes[2] == 27)
    {
#ifdef DEBUG_EMC
      COMMENT("interpreter: automatic A-axis clamping turned off");
#endif
      settings->a_axis_clamping = false;
    }
*/
if (is_user_defined_m_code(block, settings, 9) && ONCE_M(9)) {
     return convert_remapped_code(block, settings, STEP_M_9, 'm',
				   block->m_modes[9]);
 } else if ((block->m_modes[9] == 48)  && ONCE_M(9)){
    CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
         (_("Cannot enable overrides with cutter radius compensation on")));  // XXX
    ENABLE_FEED_OVERRIDE();
    settings->feed_override = true;
    for (int s = 0; s < settings->num_spindles; s++){
    	settings->speed_override[s] = true;
        ENABLE_SPEED_OVERRIDE(s);
    }
 } else if ((block->m_modes[9] == 49)  && ONCE_M(9)){
    CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
         (_("Cannot disable overrides with cutter radius compensation on")));  // XXX
    DISABLE_FEED_OVERRIDE();
    settings->feed_override = false;
    for (int s = 0; s < settings->num_spindles; s++){
    	settings->speed_override[s] = false;
        DISABLE_SPEED_OVERRIDE(s);
    }
  }

if ((block->m_modes[9] == 50)  && ONCE_M(9)){
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_FEED_OVERRIDE();
	settings->feed_override = true;
    } else {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot disable overrides with cutter radius compensation on")));  // XXX
        DISABLE_FEED_OVERRIDE();
	settings->feed_override = false;
    }
  }

if ((block->m_modes[9] == 51)  && ONCE_M(9)){
	int e = -1;
	if (block->dollar_flag){
		CHKS((block->dollar_number <= 0 || block->dollar_number >= settings-> num_spindles),
				(_("Invalid spindle ($) number in M51 command")));
		e = block->dollar_number;
	}
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot enable overrides with cutter radius compensation on")));  // XXX
		for (int s = 0; s < settings->num_spindles; s++){
			if (e == -1 or s == e){
				ENABLE_SPEED_OVERRIDE(s);
				settings->speed_override[s] = true;
			}
		}
    } else {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot disable overrides with cutter radius compensation on")));  // XXX
		for (int s = 0; s < settings->num_spindles; s++){
			if (e == -1 or s == e){
				DISABLE_SPEED_OVERRIDE(s);
				settings->speed_override[s] = false;
			}
		}
    }
  }

if ((block->m_modes[9] == 52)  && ONCE_M(9)){
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_ADAPTIVE_FEED();
	settings->adaptive_feed = true;
    } else {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot disable overrides with cutter radius compensation on")));  // XXX
	DISABLE_ADAPTIVE_FEED();
	settings->adaptive_feed = false;
    }
  }

if ((block->m_modes[9] == 53)  && ONCE_M(9)){
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_FEED_HOLD();
	settings->feed_hold = true;
    } else {
        CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
             (_("Cannot disable overrides with cutter radius compensation on")));  // XXX
	DISABLE_FEED_HOLD();
	settings->feed_hold = false;
    }
  }

if (is_user_defined_m_code(block, settings, 10) && ONCE_M(10)) {
    return convert_remapped_code(block,settings,STEP_M_10,'m',
				   block->m_modes[10]);

 } else if ((block->m_modes[10] != -1)  && ONCE_M(10)){
     /* user-defined M codes */
    int index = block->m_modes[10];
    if (USER_DEFINED_FUNCTION[index - 100] == 0) {
      CHKS(1, NCE_UNKNOWN_M_CODE_USED,index);
    }
    enqueue_M_USER_COMMAND(index,block->p_number,block->q_number);
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_modal_0

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_axis_offsets
      convert_home
      convert_setup
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. code is not G_4, G_10, G_28, G_30, G_52, G_53, G_92:
      NCE_BUG_CODE_NOT_G4_G10_G28_G30_G52_G53_OR_G92_SERIES

Side effects: See below

Called by: convert_g

If the g_code is g10, g28, g30, g52, g92, g92.1, g92.2, or g92.3 (all are in
modal group 0), it is executed. The other two in modal group 0 (G4 and
G53) are executed elsewhere.

*/

int Interp::convert_modal_0(int code,    						//!< G-code, must be from group 0
                           block_pointer block, 		//!< pointer to a block of RS274/NGC instructions
                           setup_pointer settings)  //!< pointer to machine settings
{

  if (code == G_10) {
      if(block->l_number == 1 || block->l_number == 10 || block->l_number == 11)
          CHP(convert_setup_tool(block, settings));
      else if (block->l_number == 0) {
          int tno = settings->tool_table[0].toolno;
          if (tno > 0) { ERS("G10 L0 not allowed with loaded tool <%d>\n",tno); }
          settings->toolchange_flag = true; // refresh actual pos, sync,etc
          RELOAD_TOOLDATA(); // msg to io
      }
      else
          CHP(convert_setup(block, settings));
  } else if ((code == G_28) || (code == G_30)) {
    CHP(convert_home(code, block, settings));
  } else if ((code == G_28_1) || (code == G_30_1)) {
    CHP(convert_savehome(code, block, settings));
  } else if ((code == G_52) || (code == G_92)) {
    CHP(convert_axis_offsets(code, block, settings));
  } else if ((code == G_5_3)||(code == G_6_3)) { // jjf
    CHP(convert_nurbs(code, block, settings));
  } else if ((code == G_4) || (code == G_53));  // handled elsewhere 
  else
    ERS(NCE_BUG_CODE_NOT_G4_G10_G28_G30_G52_G53_OR_G92_SERIES);
  return INTERP_OK;
}

int Interp::convert_g92_is_applied(int code, block_pointer block,
                                   setup_pointer settings)
{
    CHP(convert_axis_offsets(code, block, settings));
    return INTERP_OK;
}


/****************************************************************************/

/*! convert_motion

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_arc
      convert_cycle
      convert_probe
      convert_straight
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The motion code is not 0,1,2,3,38.2,80,81,82,83,84,85,86,87, 88, or 89:
      NCE_BUG_UNKNOWN_MOTION_CODE

Side effects:
   A g_code from the group causing motion (mode 1) is executed.

Called by: convert_g.

*/

int Interp::convert_motion(int motion,   //!< g_code for a line, arc, canned cycle
                          block_pointer block,  //!< pointer to a block of RS274 instructions
                          setup_pointer settings)       //!< pointer to machine settings
{
  int ai = block->a_flag && (-1 != settings->a_indexer_jnum);
  int bi = block->b_flag && (-1 != settings->b_indexer_jnum);
  int ci = block->c_flag && (-1 != settings->c_indexer_jnum);


  if (motion != G_0) {
      CHKS((ai), (_("Indexing axis %c can only be moved with G0")), 'A');
      CHKS((bi), (_("Indexing axis %c can only be moved with G0")), 'B');
      CHKS((ci), (_("Indexing axis %c can only be moved with G0")), 'C');
  }

  int xyzuvw_flag = (block->x_flag || block->y_flag || block->z_flag ||
                     block->u_flag || block->v_flag || block->w_flag);

  CHKS((ai && (xyzuvw_flag || block->b_flag || block->c_flag)),
       (_("Indexing axis %c can only be moved alone")), 'A');
  CHKS((bi && (xyzuvw_flag || block->a_flag || block->c_flag)),
       (_("Indexing axis %c can only be moved alone")), 'B');
  CHKS((ci && (xyzuvw_flag || block->a_flag || block->b_flag)),
       (_("Indexing axis %c can only be moved alone")), 'C');

  if (!is_a_cycle(motion))
    settings->cycle_il_flag = false;

  if (ai || bi || ci) {
    int anum=-1,jnum=-1;
    if (     ai) {anum = 3; jnum = settings->a_indexer_jnum;}
    else if (bi) {anum = 4; jnum = settings->b_indexer_jnum;}
    else if (ci) {anum = 5; jnum = settings->c_indexer_jnum;}
    CHP(convert_straight_indexer(anum, jnum, block, settings));
  } else if ((motion == G_0) || (motion == G_1) || (motion == G_33) || (motion == G_33_1) || (motion == G_76)) {
    CHP(convert_straight(motion, block, settings));
  } else if ((motion == G_3) || (motion == G_2)) {
    CHP(convert_arc(motion, block, settings));
  } else if (motion == G_38_2 || motion == G_38_3 ||
             motion == G_38_4 || motion == G_38_5) {
    CHP(convert_probe(block, motion, settings));
  } else if (motion == G_80) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: motion mode set to none");
#endif
    settings->motion_mode = G_80;
  } else if (is_user_defined_g_code(motion)) {
      CHP(convert_remapped_code(block, settings, STEP_MOTION, 'g', motion));
  } else if (is_a_cycle(motion)) {

    CHP(convert_cycle(motion, block, settings));
  } else if ((motion == G_5) || (motion == G_5_1) || (motion == G_6) || (motion == G_6_1) ) { // jjf
      write_canon_state_tag(block, settings);
      CHP(convert_spline(motion, block, settings));
  } else if ((motion == G_5_2) || (motion == G_6_2)) { // jjf
    write_canon_state_tag(block, settings);
    CHP(convert_nurbs(motion, block, settings));
  } else if (
    motion == G_70 ||
    motion == G_71 || motion == G_71_1 || motion == G_71_2 ||
    motion == G_72 || motion == G_72_1 || motion == G_72_2
  ) {
    CHP(convert_g7x(motion, block, settings));
  } else {
    ERS(NCE_BUG_UNKNOWN_MOTION_CODE);
  }

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_probe

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. No value is given in the block for any of X, Y, or Z:
      NCE_X_Y_AND_Z_WORDS_ALL_MISSING_WITH_G38_2
   3. cutter radius comp is on: NCE_CANNOT_PROBE_WITH_CUTTER_RADIUS_COMP_ON
   4. Feed rate is zero: NCE_CANNOT_PROBE_WITH_ZERO_FEED_RATE
   5. The move is degenerate (already at the specified point)
      NCE_START_POINT_TOO_CLOSE_TO_PROBE_POINT

Side effects:
   This executes a straight_probe command.
   The probe_flag in the settings is set to true.
   The motion mode in the settings is set to G_38_2.

Called by: convert_motion.

The approach to operating in incremental distance mode (g91) is to
put the the absolute position values into the block before using the
block to generate a move.

After probing is performed, the location of the probe cannot be
predicted. This differs from every other command, all of which have
predictable results. The next call to the interpreter (with either
Interp::read or Interp::execute) will result in updating the
current position by calls to get_external_position_x, etc.

*/

int Interp::convert_probe(block_pointer block,   //!< pointer to a block of RS274 instructions
                          int g_code,
                          setup_pointer settings)        //!< pointer to machine settings
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end;
  double v_end;
  double w_end;

  /* probe_type:
     ~1 = error if probe operation is unsuccessful (ngc default)
     |1 = suppress error, report in # instead
     ~2 = move until probe trips (ngc default)
     |2 = move until probe clears */

  unsigned char probe_type = g_code - G_38_2;

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
      NCE_CANNOT_PROBE_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->feed_rate == 0.0), NCE_CANNOT_PROBE_WITH_ZERO_FEED_RATE);
  CHKS(settings->feed_mode == FEED_MODE::UNITS_PER_REVOLUTION,
	  _("Cannot probe with feed per rev mode"));
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end,
                &u_end, &v_end, &w_end));
  CHKS(((!(probe_type & 1)) &&
        settings->current_x == end_x && settings->current_y == end_y &&
        settings->current_z == end_z && settings->AA_current == AA_end &&
        settings->BB_current == BB_end && settings->CC_current == CC_end &&
        settings->u_current == u_end && settings->v_current == v_end &&
        settings->w_current == w_end),
       NCE_START_POINT_TOO_CLOSE_TO_PROBE_POINT);

  TURN_PROBE_ON();
  STRAIGHT_PROBE(block->line_number, end_x, end_y, end_z,
                 AA_end, BB_end, CC_end,
                 u_end, v_end, w_end, probe_type);

  TURN_PROBE_OFF();
  settings->motion_mode = g_code;
  settings->probe_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_retract_mode

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_98 or G_99: NCE_BUG_CODE_NOT_G98_OR_G99

Side effects:
   The interpreter switches the machine settings to indicate the current
   retract mode for canned cycles (OLD_Z or R_PLANE).

Called by: convert_g.

The canonical machine to which commands are being sent does not have a
retract mode, so no command setting the retract mode is generated in
this function.

*/

int Interp::convert_retract_mode(int g_code,     //!< g_code being executed (must be G_98 or G_99)
                                setup_pointer settings) //!< pointer to machine settings
{
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
       (_("Cannot change retract mode with cutter radius compensation on")));
  if (g_code == G_98) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: retract mode set to old_z");
#endif
    settings->retract_mode = RETRACT_MODE::OLD_Z;
  } else if (g_code == G_99) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: retract mode set to r_plane");
#endif
    settings->retract_mode = RETRACT_MODE::R_PLANE;
  } else
    ERS(NCE_BUG_CODE_NOT_G98_OR_G99);
  return INTERP_OK;
}

// G10 L1  P[tool number] R[radius] X[x offset] Z[z offset] Q[orientation]
// G10 L10 P[tool number] R[radius] X[x offset] Z[z offset] Q[orientation]

int Interp::convert_setup_tool(block_pointer block, setup_pointer settings) {
    int idx = -1, toolno;
    int q;
    double tx, ty, tz, ta, tb, tc, tu, tv, tw;
    int direct = block->l_number == 1;

    is_near_int(&toolno, block->p_number);

    CHP((find_tool_index(settings, toolno, &idx)));

    CHKS(!(block->x_flag || block->y_flag || block->z_flag ||
           block->a_flag || block->b_flag || block->c_flag ||
           block->u_flag || block->v_flag || block->w_flag ||
           block->r_flag || block->q_flag || block->i_flag ||
           block->j_flag),
         _("G10 L1 without offsets has no effect"));

    if(direct) {
        if(block->x_flag)
            settings->tool_table[idx].offset.tran.x = PROGRAM_TO_USER_LEN(block->x_number);
        if(block->y_flag)
            settings->tool_table[idx].offset.tran.y = PROGRAM_TO_USER_LEN(block->y_number);
        if(block->z_flag)
            settings->tool_table[idx].offset.tran.z = PROGRAM_TO_USER_LEN(block->z_number);
        if(block->a_flag)
            settings->tool_table[idx].offset.a = PROGRAM_TO_USER_ANG(block->a_number);
        if(block->b_flag)
            settings->tool_table[idx].offset.b = PROGRAM_TO_USER_ANG(block->b_number);
        if(block->c_flag)
            settings->tool_table[idx].offset.c = PROGRAM_TO_USER_ANG(block->c_number);
        if(block->u_flag)
            settings->tool_table[idx].offset.u = PROGRAM_TO_USER_LEN(block->u_number);
        if(block->v_flag)
            settings->tool_table[idx].offset.v = PROGRAM_TO_USER_LEN(block->v_number);
        if(block->w_flag)
            settings->tool_table[idx].offset.w = PROGRAM_TO_USER_LEN(block->w_number);
    } else {
        int to_fixture = block->l_number == 11;
        int destination_system = to_fixture? 9 : settings->origin_index; // maybe 9 (g59.3) should be user configurable?

        find_current_in_system_without_tlo(settings, destination_system,
                                           &tx, &ty, &tz,
                                           &ta, &tb, &tc,
                                           &tu, &tv, &tw);

        if ( to_fixture && settings->parameters[5210]) {
            // For G10L11, we don't want to move the origin of the
            // fixture according to G92.  Since find_current_in_system
            // did this for us already, undo it.
            tx += USER_TO_PROGRAM_LEN(settings->parameters[5211]);
            ty += USER_TO_PROGRAM_LEN(settings->parameters[5212]);
            tz += USER_TO_PROGRAM_LEN(settings->parameters[5213]);
            ta += USER_TO_PROGRAM_ANG(settings->parameters[5214]);
            tb += USER_TO_PROGRAM_ANG(settings->parameters[5215]);
            tc += USER_TO_PROGRAM_ANG(settings->parameters[5216]);
            tu += USER_TO_PROGRAM_LEN(settings->parameters[5217]);
            tv += USER_TO_PROGRAM_LEN(settings->parameters[5218]);
            tw += USER_TO_PROGRAM_LEN(settings->parameters[5219]);
        }


        if(block->x_flag && block->y_flag) {
            tx -= block->x_number;
            ty -= block->y_number;
            rotate(&tx, &ty, settings->parameters[5210 + destination_system * 20]);
            settings->tool_table[idx].offset.tran.x = PROGRAM_TO_USER_LEN(tx);
            settings->tool_table[idx].offset.tran.y = PROGRAM_TO_USER_LEN(ty);
        } else if(block->x_flag) {
            // keep the component of the tool table's current setting that points
            // along our possibly-rotated Y axis
            double ox, oy;
            ox = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.x);
            oy = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.y);
            rotate(&ox, &oy, -settings->parameters[5210 + destination_system * 20]);
            ox = 0;
            rotate(&ox, &oy, settings->parameters[5210 + destination_system * 20]);


            tx -= block->x_number;
            ty = 0;
            rotate(&tx, &ty, settings->parameters[5210 + destination_system * 20]);

            settings->tool_table[idx].offset.tran.x = PROGRAM_TO_USER_LEN(tx + ox);
            settings->tool_table[idx].offset.tran.y = PROGRAM_TO_USER_LEN(ty + oy);
        } else if(block->y_flag) {
            double ox, oy;
            ox = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.x);
            oy = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.y);
            rotate(&ox, &oy, -settings->parameters[5210 + destination_system * 20]);
            oy = 0;
            rotate(&ox, &oy, settings->parameters[5210 + destination_system * 20]);

            ty -= block->y_number;
            tx = 0;

            rotate(&tx, &ty, settings->parameters[5210 + destination_system * 20]);
            settings->tool_table[idx].offset.tran.x = PROGRAM_TO_USER_LEN(tx + ox);
            settings->tool_table[idx].offset.tran.y = PROGRAM_TO_USER_LEN(ty + oy);
        }


        if(block->z_flag)
            settings->tool_table[idx].offset.tran.z = PROGRAM_TO_USER_LEN(tz - block->z_number);
        if(block->a_flag)
            settings->tool_table[idx].offset.a = PROGRAM_TO_USER_ANG(ta - block->a_number);
        if(block->b_flag)
            settings->tool_table[idx].offset.b = PROGRAM_TO_USER_ANG(tb - block->b_number);
        if(block->c_flag)
            settings->tool_table[idx].offset.c = PROGRAM_TO_USER_ANG(tc - block->c_number);
        if(block->u_flag)
            settings->tool_table[idx].offset.u = PROGRAM_TO_USER_LEN(tu - block->u_number);
        if(block->v_flag)
            settings->tool_table[idx].offset.v = PROGRAM_TO_USER_LEN(tv - block->v_number);
        if(block->w_flag)
            settings->tool_table[idx].offset.w = PROGRAM_TO_USER_LEN(tw - block->w_number);
    }

    if(block->r_flag) settings->tool_table[idx].diameter = PROGRAM_TO_USER_LEN(block->r_number) * 2.;
    if(block->i_flag) settings->tool_table[idx].frontangle = block->i_number;
    if(block->j_flag) settings->tool_table[idx].backangle = block->j_number;
    if(block->q_number != -1.0) {
        CHKS((!is_near_int(&q, block->q_number)), _("Q number in G10 is not an integer"));
        CHKS((q > 9), _("Invalid tool orientation"));
        settings->tool_table[idx].orientation = q;
    }

    SET_TOOL_TABLE_ENTRY(idx,
                         settings->tool_table[idx].toolno,
                         settings->tool_table[idx].offset,
                         settings->tool_table[idx].diameter,
                         settings->tool_table[idx].frontangle,
                         settings->tool_table[idx].backangle,
                         settings->tool_table[idx].orientation);

    //
    // On non-random tool changers we just updated the tool's "home index"
    // in the tool changer carousel, so now, if the tool is currently
    // loaded, we need to copy the new tool information to the spindle
    // (index 0).  This is never needed on random tool changers because
    // there tools don't have a home index, and instead we updated index
    // 0 (the spindle) directly when modifying the loaded tool.
    //
    if ((!settings->random_toolchanger) && (settings->current_pocket == idx)) {
        settings->tool_table[0] = settings->tool_table[idx];
    }

    //
    // Update parameter #5400 with the tool currently in the spindle, or a
    // special "invalid tool number" marker if no tool is in the spindle.
    // Unfortunately, random and nonrandom toolchangers use a different
    // number for "invalid tool number":  nonrandom uses 0, random uses -1.
    //
    if (settings->random_toolchanger) {
        if (settings->tool_table[0].toolno >= 0) {
            settings->parameters[5400] = settings->tool_table[0].toolno;
        } else {
            settings->parameters[5400] = -1;
        }
    } else {
        if (settings->tool_table[0].toolno > 0) {
            settings->parameters[5400] = settings->tool_table[0].toolno;
        } else {
            settings->parameters[5400] = 0;
        }
    }

    settings->parameters[5401] = settings->tool_table[0].offset.tran.x;
    settings->parameters[5402] = settings->tool_table[0].offset.tran.y;
    settings->parameters[5403] = settings->tool_table[0].offset.tran.z;
    settings->parameters[5404] = settings->tool_table[0].offset.a;
    settings->parameters[5405] = settings->tool_table[0].offset.b;
    settings->parameters[5406] = settings->tool_table[0].offset.c;
    settings->parameters[5407] = settings->tool_table[0].offset.u;
    settings->parameters[5408] = settings->tool_table[0].offset.v;
    settings->parameters[5409] = settings->tool_table[0].offset.w;
    settings->parameters[5410] = settings->tool_table[0].diameter;
    settings->parameters[5411] = settings->tool_table[0].frontangle;
    settings->parameters[5412] = settings->tool_table[0].backangle;
    settings->parameters[5413] = settings->tool_table[0].orientation;

    // if the modified tool is currently in the spindle, then copy its
    // information to index 0 of the tool table (which signifies the
    // spindle)
    if (   !_setup.random_toolchanger
           && idx == settings->current_pocket) {
        SET_TOOL_TABLE_ENTRY(0,
                             settings->tool_table[idx].toolno,
                             settings->tool_table[idx].offset,
                             settings->tool_table[idx].diameter,
                             settings->tool_table[idx].frontangle,
                             settings->tool_table[idx].backangle,
                             settings->tool_table[idx].orientation);
    }

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_setup

Returned Value: int (INTERP_OK)

Side effects:
   SET_G5X_OFFSET is called, and the coordinate
   values for the program origin are reset.
   If the program origin is currently in use, the values of the
   the coordinates of the current point are updated.

Called by: convert_modal_0.

This is called only if g10 is called. g10 L2 may be used to alter the
location of coordinate systems as described in [NCMS, pages 9 - 10] and
[Fanuc, page 65]. [Fanuc] has only six coordinate systems, while
[NCMS] has nine (the first six of which are the same as the six [Fanuc]
has). All nine are implemented here.

Being in incremental distance mode has no effect on the action of G10
in this implementation. The manual is not explicit about what is
intended.

If L is 20 instead of 2, the coordinates are relative to the current
position instead of the origin.  Like how G92 offsets are programmed,
the meaning is "set the coordinate system origin such that my current
position becomes the specified value".

See documentation of convert_coordinate_system for more information.

*/

int Interp::convert_setup(block_pointer block,   //!< pointer to a block of RS274/NGC instructions
                         setup_pointer settings)        //!< pointer to machine settings
{
  double x;
  double y;
  double z;
  double a;
  double b;
  double c;
  double u, v, w;
  double r;
  double *parameters;
  int p_int;

  double cx, cy, cz, ca, cb, cc, cu, cv, cw;

  CHKS((block->i_flag || block->j_flag), _("I J words not allowed with G10 L2"));

  parameters = settings->parameters;
  p_int = (int) (block->p_number + 0.0001);

  // if P = 0 then use whatever coordinate system that is currently active
  if (p_int == 0) {
    p_int = settings->origin_index;
  }

  CHKS((block->l_number == 20 && block->a_flag && settings->a_axis_wrapped &&
        (block->a_number <= -360.0 || block->a_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->a_number, 'A');
  CHKS((block->l_number == 20 && block->b_flag && settings->b_axis_wrapped &&
        (block->b_number <= -360.0 || block->b_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->b_number, 'B');
  CHKS((block->l_number == 20 && block->c_flag && settings->c_axis_wrapped &&
        (block->c_number <= -360.0 || block->c_number >= 360.0)),
       (_("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->c_number, 'C');

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF && p_int == settings->origin_index),
       (_("Cannot change the active coordinate system with cutter radius compensation on")));

  find_current_in_system(settings, p_int,
                         &cx, &cy, &cz,
                         &ca, &cb, &cc,
                         &cu, &cv, &cw);

  if (block->r_flag) {
    CHKS((block->l_number == 20), _("R not allowed in G10 L20"));
    r = block->r_number;
    parameters[5210 + (p_int * 20)] = r;
  } else
    r = parameters[5210 + (p_int * 20)];

  if (block->l_number == 20) {
      // old position in rotated system
      double oldx = cx, oldy = cy;

      // find new desired position in rotated system
      x = cx;
      y = cy;

      if (block->x_flag) {
          x = block->x_number;
      }
      if (block->y_flag) {
          y = block->y_number;
      }

      // move old current position into the unrotated system
      rotate(&oldx, &oldy, r);
      // move desired position into the unrotated system
      rotate(&x, &y, r);

      // find new offset
      x = oldx + USER_TO_PROGRAM_LEN(parameters[5201 + (p_int * 20)]) - x;
      y = oldy + USER_TO_PROGRAM_LEN(parameters[5202 + (p_int * 20)]) - y;

      // parameters are not rotated
      parameters[5201 + (p_int * 20)] = PROGRAM_TO_USER_LEN(x);
      parameters[5202 + (p_int * 20)] = PROGRAM_TO_USER_LEN(y);

      if (p_int == settings->origin_index) {
          // let the code below fix up the current coordinates correctly
          rotate(&settings->current_x, &settings->current_y, settings->rotation_xy);
          settings->rotation_xy = 0;
      }
  } else {
      if (block->x_flag) {
          x = block->x_number;
          parameters[5201 + (p_int * 20)] = PROGRAM_TO_USER_LEN(x);
      } else {
          x = USER_TO_PROGRAM_LEN(parameters[5201 + (p_int * 20)]);
      }
      if (block->y_flag) {
          y = block->y_number;
          parameters[5202 + (p_int * 20)] = PROGRAM_TO_USER_LEN(y);
      } else {
          y = USER_TO_PROGRAM_LEN(parameters[5202 + (p_int * 20)]);
      }
  }

  if (block->z_flag) {
    z = block->z_number;
    if (block->l_number == 20) z = cz + USER_TO_PROGRAM_LEN(parameters[5203 + (p_int * 20)]) - z;
    parameters[5203 + (p_int * 20)] = PROGRAM_TO_USER_LEN(z);
  } else
    z = USER_TO_PROGRAM_LEN(parameters[5203 + (p_int * 20)]);

  if (block->a_flag) {
    a = block->a_number;
    if (block->l_number == 20) a = ca + USER_TO_PROGRAM_ANG(parameters[5204 + (p_int * 20)]) - a;
    parameters[5204 + (p_int * 20)] = PROGRAM_TO_USER_ANG(a);
  } else
    a = USER_TO_PROGRAM_ANG(parameters[5204 + (p_int * 20)]);

  if (block->b_flag) {
    b = block->b_number;
    if (block->l_number == 20) b = cb + USER_TO_PROGRAM_ANG(parameters[5205 + (p_int * 20)]) - b;
    parameters[5205 + (p_int * 20)] = PROGRAM_TO_USER_ANG(b);
  } else
    b = USER_TO_PROGRAM_ANG(parameters[5205 + (p_int * 20)]);

  if (block->c_flag) {
    c = block->c_number;
    if (block->l_number == 20) c = cc + USER_TO_PROGRAM_ANG(parameters[5206 + (p_int * 20)]) - c;
    parameters[5206 + (p_int * 20)] = PROGRAM_TO_USER_ANG(c);
  } else
    c = USER_TO_PROGRAM_ANG(parameters[5206 + (p_int * 20)]);

  if (block->u_flag) {
    u = block->u_number;
    if (block->l_number == 20) u = cu + USER_TO_PROGRAM_LEN(parameters[5207 + (p_int * 20)]) - u;
    parameters[5207 + (p_int * 20)] = PROGRAM_TO_USER_LEN(u);
  } else
    u = USER_TO_PROGRAM_LEN(parameters[5207 + (p_int * 20)]);

  if (block->v_flag) {
    v = block->v_number;
    if (block->l_number == 20) v = cv + USER_TO_PROGRAM_LEN(parameters[5208 + (p_int * 20)]) - v;
    parameters[5208 + (p_int * 20)] = PROGRAM_TO_USER_LEN(v);
  } else
    v = USER_TO_PROGRAM_LEN(parameters[5208 + (p_int * 20)]);

  if (block->w_flag) {
    w = block->w_number;
    if (block->l_number == 20) w = cw + USER_TO_PROGRAM_LEN(parameters[5209 + (p_int * 20)]) - w;
    parameters[5209 + (p_int * 20)] = PROGRAM_TO_USER_LEN(w);
  } else
    w = USER_TO_PROGRAM_LEN(parameters[5209 + (p_int * 20)]);

  if (p_int == settings->origin_index) {        /* system is currently used */

    rotate(&settings->current_x, &settings->current_y, settings->rotation_xy);

    settings->current_x += settings->origin_offset_x;
    settings->current_y += settings->origin_offset_y;
    settings->current_z += settings->origin_offset_z;
    settings->AA_current += settings->AA_origin_offset;
    settings->BB_current += settings->BB_origin_offset;
    settings->CC_current += settings->CC_origin_offset;
    settings->u_current += settings->u_origin_offset;
    settings->v_current += settings->v_origin_offset;
    settings->w_current += settings->w_origin_offset;

    settings->origin_offset_x = x;
    settings->origin_offset_y = y;
    settings->origin_offset_z = z;
    settings->AA_origin_offset = a;
    settings->BB_origin_offset = b;
    settings->CC_origin_offset = c;
    settings->u_origin_offset = u;
    settings->v_origin_offset = v;
    settings->w_origin_offset = w;

    settings->current_x -= x;
    settings->current_y -= y;
    settings->current_z -= z;
    settings->AA_current -= a;
    settings->BB_current -= b;
    settings->CC_current -= c;
    settings->u_current -= u;
    settings->v_current -= v;
    settings->w_current -= w;

    SET_G5X_OFFSET(p_int, x, y, z, a, b, c, u, v, w);

    rotate(&settings->current_x, &settings->current_y, - r);
    settings->rotation_xy = r;
    SET_XY_ROTATION(settings->rotation_xy);

  }
#ifdef DEBUG_EMC
  else
    enqueue_COMMENT("interpreter: setting coordinate system origin");
#endif
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_set_plane

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The user tries to change to a different plane while comp is on:
      NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
   2. The g_code is not G_17, G_18, or G_19:
      NCE_BUG_CODE_NOT_G17_G18_OR_G19

Side effects:
   A canonical command setting the current plane is executed.

Called by: convert_g.

*/

int Interp::convert_set_plane(int g_code,        //!< must be G_17, G_18, or G_19
                             setup_pointer settings)    //!< pointer to machine settings
{
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF && g_code == G_17 && settings->plane != CANON_PLANE::XY),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF && g_code == G_18 && settings->plane != CANON_PLANE::XZ),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF && g_code == G_19 && settings->plane != CANON_PLANE::YZ),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF && g_code == G_19),
          NCE_RADIUS_COMP_ONLY_IN_XY_OR_XZ);

  if (g_code == G_17) {
    SELECT_PLANE(CANON_PLANE::XY);
    settings->plane = CANON_PLANE::XY;
  } else if (g_code == G_18) {
    SELECT_PLANE(CANON_PLANE::XZ);
    settings->plane = CANON_PLANE::XZ;
  } else if (g_code == G_19) {
    SELECT_PLANE(CANON_PLANE::YZ);
    settings->plane = CANON_PLANE::YZ;
  } else if (g_code == G_17_1) {
    SELECT_PLANE(CANON_PLANE::UV);
    settings->plane = CANON_PLANE::UV;
  } else if (g_code == G_18_1) {
    SELECT_PLANE(CANON_PLANE::UW);
    settings->plane = CANON_PLANE::UW;
  } else if (g_code == G_19_1) {
    SELECT_PLANE(CANON_PLANE::VW);
    settings->plane = CANON_PLANE::VW;
  } else
    ERS(NCE_BUG_CODE_NOT_G17_G18_OR_G19);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_speed

Returned Value: int (INTERP_OK)

Side effects:
  The machine spindle speed is set to the value of s_number in the
  block by a call to SET_SPINDLE_SPEED.
  The machine model for spindle speed is set to that value.

Called by: execute_block.

*/

int Interp::convert_speed(int spindle,          //The spindle ($) or -1 if none
					      block_pointer block,   //!< pointer to a block of RS274 instructions
                          setup_pointer settings)//!< pointer to machine settings
{
    if (spindle >= 0 && spindle <  settings->num_spindles){
        enqueue_SET_SPINDLE_SPEED(spindle, block->s_number);
        settings->speed[spindle] = block->s_number;
}

  return INTERP_OK;
}

int Interp::convert_spindle_mode(int dollar_number, block_pointer block, setup_pointer settings)
{
	for (int s = 0; s < settings->num_spindles; s++){
		if (dollar_number == -1 || s == dollar_number){
			  if(block->g_modes[GM_SPINDLE_MODE] == G_97) {
				settings->spindle_mode[s] = SPINDLE_MODE::CONSTANT_RPM;
			enqueue_SET_SPINDLE_MODE(s, 0);
			} else { /* G_96 */
				settings->spindle_mode[s] = SPINDLE_MODE::CONSTANT_SURFACE;
			if(block->d_flag)
				enqueue_SET_SPINDLE_MODE(s, fabs(block->d_number_float));
			else
				enqueue_SET_SPINDLE_MODE(s, 1e30);
			}
		}
	}
    return INTERP_OK;
}
/****************************************************************************/

/*! convert_stop

Returned Value: int
   When an m2 or m30 (program_end) is encountered, this returns INTERP_EXIT.
   M99 main program endless loop:
       if looping is disabled (default, not in task), return INTERP_EXIT;
       else in task, return INTERP_EXECUTE_FINISH.
   M99 return from subprogram is not handled here, and raises an error.
   If the code is not m0, m1, m2, m30, m60, or m99 this returns
       NCE_BUG_CODE_NOT_M0_M1_M2_M30_M60_M99
   Otherwise, it returns INTERP_OK.

Side effects:
   An m0, m1, m2, m30, m60 or m99 in the block is executed.

   For m0, m1, and m60, this makes a function call to the PROGRAM_STOP
   canonical machining function (which stops program execution).
   In addition, m60 calls PALLET_SHUTTLE.

   For m2 and m30, this resets the machine and then calls PROGRAM_END.
   In addition, m30 calls PALLET_SHUTTLE.
   Clear g92 offset if DISABLE_G92_PERSISTENCE is set in the .ini file.

   For m99 main program endless looping in task, this returns control
   to the beginning of the file and outputs any linked segments to the
   interp list.  The INTERP_EXECUTE_FINISH return code causes any
   commands in the interp list to be issued so that the endless loop
   doesn't result in an infinite queue.

   For m99 main program endless looping elsewhere, especially preview
   where endless looping is not desired, this behaves as m2 and m30
   below.

Called by: execute_block.

This handles stopping or ending the program (m0, m1, m2, m30, m60)

[NCMS] specifies how the following modes should be reset at m2 or
m30. The descriptions are not collected in one place, so this list
may be incomplete.

G52 offsetting coordinate zero points [NCMS, page 10]
G92 coordinate offset using tool position [NCMS, page 10]

The following should have reset values, but no description of reset
behavior could be found in [NCMS].
G17, G18, G19 selected plane [NCMS, pages 14, 20]
G90, G91 distance mode [NCMS, page 15]
G93, G94 feed mode [NCMS, pages 35 - 37]
M48, M49 overrides enabled, disabled [NCMS, pages 37 - 38]
M3, M4, M5 spindle turning [NCMS, page 7]

The following should be set to some value at machine start-up but
not automatically reset by any of the stopping codes.
1. G20, G21 length units [NCMS, page 15]. This is up to the installer.
2. motion_control_mode. This is set in Interp::init but not reset here.
   Might add it here.

The following resets have been added by calling the appropriate
canonical machining command and/or by resetting interpreter
settings. They occur on M2 or M30.

1. origin offsets are set to the default (like G54)
2. Selected plane is set to CANON_PLANE::XY (like G17) - SELECT_PLANE
3. Distance mode is set to DISTANCE_MODE::ABSOLUTE (like G90)   - no canonical call
4. Feed mode is set to UNITS_PER_MINUTE (like G94)    - no canonical call
5. Feed and speed overrides are set to true (like M48)  - ENABLE_FEED_OVERRIDE
                                                      - ENABLE_SPEED_OVERRIDE
6. Cutter compensation is turned off (like G40)       - no canonical call
7. The spindle is stopped (like M5)                   - STOP_SPINDLE_TURNING
8. The motion mode is set to G_1 (like G1)            - no canonical call
9. Coolant is turned off (like M9)                    - FLOOD_OFF & MIST_OFF
10. G52/G92 is cleared if DISABLE_G92_PERSISTENCE is set in the .ini file

*/

int Interp::convert_stop(block_pointer block,    //!< pointer to a block of RS274/NGC instructions
                        setup_pointer settings) //!< pointer to machine settings
{
  int index;
  char *line;
  int length;

  double cx = 0, cy = 0, cz = 0;
  comp_get_current(settings, &cx, &cy, &cz);
  CHP(move_endpoint_and_flush(settings, cx, cy));
  dequeue_canons(settings);

	nurbs_reset_global_variables(); // jjf 

  // M99 as subroutine return is handled in interp_o_word.cc
  // convert_control_functions()
  CHKS((block->m_modes[4] == 99 && settings->call_level > 0),
        (_("Bug:  Reached convert_stop() from M99 as subprogram return")));
  if (block->m_modes[4] == 0) {
    PROGRAM_STOP();
  } else if (block->m_modes[4] == 60) {
    PALLET_SHUTTLE();
    PROGRAM_STOP();
  } else if (block->m_modes[4] == 1) {
    OPTIONAL_PROGRAM_STOP();
  } else if (block->m_modes[4] == 99 && _setup.loop_on_main_m99) {

    // Fanuc-style M99 main program endless loop
    logDebug("M99 main program endless loop");

    loop_to_beginning(settings);  // return control to beginning of file
    FINISH();  // Output any final linked segments
    return INTERP_EXECUTE_FINISH;  // tell task to issue any queued commands
  } else if ((block->m_modes[4] == 2) || (block->m_modes[4] == 30) ||
            (block->m_modes[4] == 99 && !_setup.loop_on_main_m99)
            ) {   /* reset stuff here */

/*1*/
    rotate(&settings->current_x, &settings->current_y, settings->rotation_xy);
    settings->current_x += settings->origin_offset_x;
    settings->current_y += settings->origin_offset_y;
    settings->current_z += settings->origin_offset_z;
    settings->AA_current += settings->AA_origin_offset;
    settings->BB_current += settings->BB_origin_offset;
    settings->CC_current += settings->CC_origin_offset;
    settings->u_current += settings->u_origin_offset;
    settings->v_current += settings->v_origin_offset;
    settings->w_current += settings->w_origin_offset;

    settings->origin_index = 1;
    settings->parameters[5220] = 1.0;
    settings->origin_offset_x = USER_TO_PROGRAM_LEN(settings->parameters[5221]);
    settings->origin_offset_y = USER_TO_PROGRAM_LEN(settings->parameters[5222]);
    settings->origin_offset_z = USER_TO_PROGRAM_LEN(settings->parameters[5223]);
    settings->AA_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5224]);
    settings->BB_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5225]);
    settings->CC_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5226]);
    settings->u_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5227]);
    settings->v_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5228]);
    settings->w_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5229]);
    settings->rotation_xy = settings->parameters[5230];

    settings->current_x -= settings->origin_offset_x;
    settings->current_y -= settings->origin_offset_y;
    settings->current_z -= settings->origin_offset_z;
    settings->AA_current -= settings->AA_origin_offset;
    settings->BB_current -= settings->BB_origin_offset;
    settings->CC_current -= settings->CC_origin_offset;
    settings->u_current -= settings->u_origin_offset;
    settings->v_current -= settings->v_origin_offset;
    settings->w_current -= settings->w_origin_offset;
    rotate(&settings->current_x, &settings->current_y, -settings->rotation_xy);

    SET_G5X_OFFSET(settings->origin_index,
                   settings->origin_offset_x,
                   settings->origin_offset_y,
                   settings->origin_offset_z,
                   settings->AA_origin_offset,
                   settings->BB_origin_offset,
                   settings->CC_origin_offset,
                   settings->u_origin_offset,
                   settings->v_origin_offset,
                   settings->w_origin_offset);
    SET_XY_ROTATION(settings->rotation_xy);

/*2*/ if (settings->plane != CANON_PLANE::XY) {
      SELECT_PLANE(CANON_PLANE::XY);
      settings->plane = CANON_PLANE::XY;
    }

/*3*/
    settings->distance_mode = DISTANCE_MODE::ABSOLUTE;

/*4*/ settings->feed_mode = FEED_MODE::UNITS_PER_MINUTE;
    SET_FEED_MODE(0, 0);
    settings->feed_rate = block->f_number;
    SET_FEED_RATE(0);

/*5*/ if (!settings->feed_override) {
      ENABLE_FEED_OVERRIDE();
      settings->feed_override = true;
    }

/*6*/
    settings->cutter_comp_side = CUTTER_COMP::OFF;
    settings->cutter_comp_firstmove = true;

/*7*/
    for (int s = 0; s < settings->num_spindles; s++){
        STOP_SPINDLE_TURNING(s);
        settings->spindle_turning[s] = CANON_STOPPED;

        settings->speed_override[s] = true;
        /* turn off FPR */
        SET_SPINDLE_MODE(s, 0);
    }

/*8*/ settings->motion_mode = G_1;

/*9*/ if (settings->mist) {
      MIST_OFF();
      settings->mist = false;
    }
    if (settings->flood) {
      FLOOD_OFF();
      settings->flood = false;
    }

/*10*/
    if (settings->disable_g92_persistence)
      // Clear G92/G52 offset
      for (index=5210; index<=5219; index++)
          settings->parameters[index] = 0;

    if (block->m_modes[4] == 30)
      PALLET_SHUTTLE();
    PROGRAM_END();
    if (_setup.percent_flag && _setup.file_pointer) {
      line = _setup.linetext;
      for (;;) {                /* check for ending percent sign and comment if missing */
        if (fgets(line, LINELEN, _setup.file_pointer) == NULL) {
          enqueue_COMMENT("interpreter: percent sign missing from end of file");
          break;
        }
        length = strlen(line);
        if (length == (LINELEN - 1)) {       // line is too long. need to finish reading the line
          for (; fgetc(_setup.file_pointer) != '\n' && !feof(_setup.file_pointer););
          continue;
        }
        for (index = (length - 1);      // index set on last char
             (index >= 0) && (isspace(line[index])); index--);
        if (line[index] == '%') // found line with % at end
        {
          for (index--; (index >= 0) && (isspace(line[index])); index--);
          if (index == -1)      // found line with only percent sign
            break;
        }
      }
    }
    unwind_call(INTERP_EXIT, __FILE__,__LINE__,__FUNCTION__);
    return INTERP_EXIT;
  } else
    ERS(NCE_BUG_CODE_NOT_M0_M1_M2_M30_M60_M99);
  return INTERP_OK;
}

/*************************************************************************** */

/*! convert_straight

Returned Value: int
   If convert_straight_comp1 or convert_straight_comp2 is called
   and returns an error code, this returns that code.
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The value of move is not G_0 or G_1:
      NCE_BUG_CODE_NOT_G0_OR_G1
   2. A straight feed (g1) move is called with feed rate set to 0:
      NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE
   3. A straight feed (g1) move is called with inverse time feed in effect
      but no f word (feed time) is provided:
      NCE_F_WORD_MISSING_WITH_INVERSE_TIME_G1_MOVE
   4. A move is called with G53 and cutter radius compensation on:
      NCE_CANNOT_USE_G53_WITH_CUTTER_RADIUS_COMP
   5. A G33 move is called without the necessary support compiled in:
      NCE_G33_NOT_SUPPORTED

Side effects:
   This executes a STRAIGHT_FEED command at cutting feed rate
   (if move is G_1) or a STRAIGHT_TRAVERSE command (if move is G_0).
   It also updates the setting of the position of the tool point to the
   end point of the move. If cutter radius compensation is on, it may
   also generate an arc before the straight move. Also, in INVERSE_TIME
   feed mode, SET_FEED_RATE will be called the feed rate setting changed.

Called by: convert_motion.

The approach to operating in incremental distance mode (g91) is to
put the the absolute position values into the block before using the
block to generate a move.

If the destination point is the same as the current point, the feed rate
will be calculated as zero, so a default of 0.1 is applied in this case
(It doesn't matter how wrong the feed rate is on a zero-length move)
 
If cutter compensation is in use, the path's length may increase or
decrease.  Also an arc may be added, to go around a corner, before the
straight move.  For the purpose of calculating the feed rate when in
inverse time mode, this length increase or decrease is ignored.  The
feed is still set to the original programmed straight length divided by
the F number (with the above lower bound).  The new arc (if needed) and
the new longer or shorter straight move are taken at this feed.

*/

int Interp::convert_straight(int move,   //!< either G_0 or G_1
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end, v_end, w_end;
  int status;

  settings->arc_not_allowed = false;

  if (move == G_1) {
    if (settings->feed_mode == FEED_MODE::UNITS_PER_MINUTE) {
      CHKS((settings->feed_rate == 0.0), NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE);
    } else if (settings->feed_mode == FEED_MODE::UNITS_PER_REVOLUTION) {
      CHKS((settings->feed_rate == 0.0), NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE);
      CHKS((settings->speed[settings->active_spindle] == 0.0),
    		  (_("Cannot feed with zero spindle speed in feed per rev mode")));
    } else if (settings->feed_mode == FEED_MODE::INVERSE_TIME) {
      CHKS((!block->f_flag),
          NCE_F_WORD_MISSING_WITH_INVERSE_TIME_G1_MOVE);
    }
  }

  settings->motion_mode = move;
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end, &u_end, &v_end, &w_end));

  if (move == G_1) {
      inverse_time_rate_straight(end_x, end_y, end_z,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end,
                                 block, settings);
  }

  // Create a state tag and dump it to canon
  write_canon_state_tag(block, settings);

  if ((settings->cutter_comp_side != CUTTER_COMP::OFF) &&    /* ! "== true" */
      (settings->cutter_comp_radius > 0.0)) {   /* radius always is >= 0 */

    CHKS((block->g_modes[GM_MODAL_0] == G_53),
        NCE_CANNOT_USE_G53_WITH_CUTTER_RADIUS_COMP);

    if(settings->plane == CANON_PLANE::XZ) {
        if (settings->cutter_comp_firstmove)
            status = convert_straight_comp1(move, block, settings, end_z, end_x, end_y,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
        else
            status = convert_straight_comp2(move, block, settings, end_z, end_x, end_y,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else if(settings->plane == CANON_PLANE::XY) {
        if (settings->cutter_comp_firstmove)
            status = convert_straight_comp1(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
        else
            status = convert_straight_comp2(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else ERS("BUG: Invalid plane for cutter compensation");
    CHP(status);
  } else if (move == G_0) {
    STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                      AA_end, BB_end, CC_end,
                      u_end, v_end, w_end);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_1) {
    STRAIGHT_FEED(block->line_number, end_x, end_y, end_z,
                  AA_end, BB_end, CC_end,
                  u_end, v_end, w_end);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_33) {
	if (block->dollar_flag){
		CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
				(_("Invalid spindle ($) number in G33 move")));
		settings->active_spindle = (int)block->dollar_number;
	}
    CHKS(((settings->spindle_turning[settings->active_spindle] != CANON_CLOCKWISE) &&
           (settings->spindle_turning[settings->active_spindle] != CANON_COUNTERCLOCKWISE)),
          _("Spindle not turning in G33"));
    START_SPEED_FEED_SYNCH(settings->active_spindle, block->k_number, 0);
    STRAIGHT_FEED(block->line_number, end_x, end_y, end_z, AA_end, BB_end, CC_end, u_end, v_end, w_end);
    STOP_SPEED_FEED_SYNCH();
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_33_1) {
	if (block->dollar_flag){
		CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
				(_("Invalid spindle ($) number in G33.1 move")));
		settings->active_spindle = (int)block->dollar_number;
	}
    CHKS(((settings->spindle_turning[settings->active_spindle] != CANON_CLOCKWISE) &&
           (settings->spindle_turning[settings->active_spindle] != CANON_COUNTERCLOCKWISE)),
          _("Spindle not turning in G33.1"));
    START_SPEED_FEED_SYNCH(settings->active_spindle, block->k_number, 0);
    double scale = 1;
    if(block->i_flag){
        scale = block->i_number;
        if(scale < 1){
            scale = 1;
        }
    }
    RIGID_TAP(block->line_number, end_x, end_y, end_z, scale);
    STOP_SPEED_FEED_SYNCH();
    // after the RIGID_TAP cycle we'll be in the same spot
  } else if (move == G_76) {
	if (block->dollar_flag){
		CHKS((block->dollar_number < 0 || block->dollar_number >= settings->num_spindles),
				(_("Invalid D-number in G76 cycle")));
		settings->active_spindle = (int)block->dollar_number;
	}
    CHKS(((settings->spindle_turning[settings->active_spindle] != CANON_CLOCKWISE) &&
           (settings->spindle_turning[settings->active_spindle] != CANON_COUNTERCLOCKWISE)),
          _("Chosen spindle (%i) not turning in G76"), settings->active_spindle);
    CHKS((settings->AA_current != AA_end ||
         settings->BB_current != BB_end ||
         settings->CC_current != CC_end ||
         settings->u_current != u_end ||
         settings->v_current != v_end ||
         settings->w_current != w_end), NCE_CANNOT_MOVE_ROTARY_AXES_WITH_G76);
    int result = convert_threading_cycle(block, settings, end_x, end_y, end_z);
    if(result != INTERP_OK) return result;
  } else
    ERS(NCE_BUG_CODE_NOT_G0_OR_G1);

  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u_end;
  settings->v_current = v_end;
  settings->w_current = w_end;
  return INTERP_OK;
}

int Interp::convert_straight_indexer(int axis, int jnum, block_pointer block, setup_pointer settings) {
    double end_x, end_y, end_z;
    double AA_end, BB_end, CC_end;
    double u_end, v_end, w_end;

    find_ends(block, settings, &end_x, &end_y, &end_z,
              &AA_end, &BB_end, &CC_end, &u_end, &v_end, &w_end);

    CHKS((end_x != settings->current_x ||
          end_y != settings->current_y ||
          end_z != settings->current_z ||
          u_end != settings->u_current ||
          v_end != settings->v_current ||
          w_end != settings->w_current ||
          (axis != 3 && AA_end != settings->AA_current) ||
          (axis != 4 && BB_end != settings->BB_current) ||
          (axis != 5 && CC_end != settings->CC_current)),
         _("BUG: An axis incorrectly moved along with an indexer"));

    switch(axis) {
    case 3:
        issue_straight_index(axis, jnum, AA_end, block->line_number, settings);
        break;
    case 4:
        issue_straight_index(axis, jnum, BB_end, block->line_number, settings);
        break;
    case 5:
        issue_straight_index(axis, jnum, CC_end, block->line_number, settings);
        break;
    default:
        ERS((_("BUG: trying to index incorrect axis")));
    }
    return INTERP_OK;
}

int Interp::issue_straight_index(int axis, int jnum, double target, int lineno, setup_pointer settings) {
    CANON_MOTION_MODE save_mode;
    double save_tolerance, save_cam_tolerance;
    // temporarily switch to exact stop mode for indexing move
    save_mode = GET_EXTERNAL_MOTION_CONTROL_MODE();
    save_tolerance = GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();
    save_cam_tolerance = GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();
    if (save_mode != CANON_EXACT_PATH)
        SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);

    double AA_end = axis == 3? target: settings->AA_current;
    double BB_end = axis == 4? target: settings->BB_current;
    double CC_end = axis == 5? target: settings->CC_current;

    // tell canon that this is a special indexing move
    UNLOCK_ROTARY(lineno, jnum);
    STRAIGHT_TRAVERSE(lineno, settings->current_x, settings->current_y, settings->current_z,
                      AA_end, BB_end, CC_end,
                      settings->u_current, settings->v_current, settings->w_current);
    LOCK_ROTARY(lineno, jnum);

    // restore path mode
    if(save_mode != CANON_EXACT_PATH) {
        SET_MOTION_CONTROL_MODE(save_mode, save_tolerance);
	SET_NAIVECAM_TOLERANCE(save_cam_tolerance);
    }

    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    return INTERP_OK;
}


#define AABBCC settings->AA_current, settings->BB_current, settings->CC_current, settings->u_current, settings->v_current, settings->w_current

// make one threading pass.  only called from convert_threading_cycle.
static void
threading_pass(setup_pointer settings, block_pointer block,
	       int boring, double safe_x, double depth, double end_depth,
	       double start_y, double start_z, double zoff, double taper_dist,
	       int entry_taper, int exit_taper, double taper_pitch,
	       double pitch, double full_threadheight, double target_z) {
    STRAIGHT_TRAVERSE(block->line_number, boring?
		      safe_x + depth - end_depth:
		      safe_x - depth + end_depth,
		      start_y, start_z - zoff, AABBCC); //back
    if(taper_dist && entry_taper) {
	DISABLE_FEED_OVERRIDE();
	START_SPEED_FEED_SYNCH(settings->active_spindle, taper_pitch, 0);
	STRAIGHT_FEED(block->line_number, boring?
		      safe_x + depth - full_threadheight:
		      safe_x - depth + full_threadheight,
		      start_y, start_z - zoff, AABBCC); //in
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth, //angled in
		      start_y, start_z - zoff - taper_dist, AABBCC);
	START_SPEED_FEED_SYNCH(settings->active_spindle, pitch, 0);
    } else {
	STRAIGHT_TRAVERSE(block->line_number, boring? safe_x + depth: safe_x - depth,
			  start_y, start_z - zoff, AABBCC); //in
	DISABLE_FEED_OVERRIDE();
	START_SPEED_FEED_SYNCH(settings->active_spindle, pitch, 0);
    }

    if(taper_dist && exit_taper) {
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth,  //over
		      start_y, target_z - zoff + taper_dist, AABBCC);
	START_SPEED_FEED_SYNCH(settings->active_spindle, taper_pitch, 0);
	STRAIGHT_FEED(block->line_number, boring?
		      safe_x + depth - full_threadheight:
		      safe_x - depth + full_threadheight,
		      start_y, target_z - zoff, AABBCC); //angled out
    } else {
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth,
		      start_y, target_z - zoff, AABBCC); //over
    }
    STOP_SPEED_FEED_SYNCH();
    STRAIGHT_TRAVERSE(block->line_number, boring?
		      safe_x + depth - end_depth:
		      safe_x - depth + end_depth,
		      start_y, target_z - zoff, AABBCC); //out
    ENABLE_FEED_OVERRIDE();
}

int Interp::convert_threading_cycle(block_pointer block,
				    setup_pointer settings,
				    double end_x, double end_y, double end_z) {


    CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
         (_("Cannot use G76 threading cycle with cutter radius compensation on")));

    CHKS((block->i_number == 0),
        (_("In G76, I must not be 0")));
    CHKS((block->j_number <= 0),
        (_("In G76, J must be greater than 0")));
    CHKS((block->k_number <= block->j_number),
        (_("In G76, K must be greater than J")));

    double start_x = settings->current_x;
    double start_y = settings->current_y;
    double start_z = settings->current_z;

    double i_number = block->i_number;
    double j_number = block->j_number;
    double k_number = block->k_number;

    if(_setup.lathe_diameter_mode){
      i_number /= 2;
      j_number /= 2;
      k_number /= 2;
    }


    int boring = 0;

    if (i_number > 0.0)
	boring = 1;

    double safe_x = start_x;
    double full_dia_depth = fabs(i_number);
    double start_depth = fabs(i_number) + fabs(j_number);
    double cut_increment = fabs(j_number);
    double full_threadheight = fabs(k_number);
    double end_depth = fabs(k_number) + fabs(i_number);

    double pitch = block->p_number;
    double compound_angle = block->q_number;
    if(compound_angle == -1) compound_angle = 0;
    compound_angle *= M_PIl/180.0;
    if(end_z > start_z) compound_angle = -compound_angle;

    int spring_cuts = block->h_flag ? block->h_number: 0;

    double degression = block->r_number;
    if(degression < 1.0 || !block->r_flag) degression = 1.0;

    double taper_dist = block->e_flag? block->e_number: 0.0;
    if(taper_dist < 0.0) taper_dist = 0.0;
    double taper_pitch = taper_dist > 0.0?
	pitch * hypot(taper_dist, full_threadheight)/taper_dist: pitch;

    if(end_z > start_z) taper_dist = -taper_dist;

    int taper_flags = block->l_number;
    if(taper_flags < 0) taper_flags = 0;

    int entry_taper = taper_flags & 1;
    int exit_taper = taper_flags & 2;

    double depth, zoff;
    int pass = 1;

    double target_z = end_z + fabs(k_number) * tan(compound_angle);

    depth = start_depth;
    zoff = (depth - full_dia_depth) * tan(compound_angle);
    while (depth < end_depth) {
	threading_pass(settings, block, boring, safe_x, depth, end_depth, start_y,
		       start_z, zoff, taper_dist, entry_taper, exit_taper,
		       taper_pitch, pitch, full_threadheight, target_z);
        depth = full_dia_depth + cut_increment * pow(++pass, 1.0/degression);
        zoff = (depth - full_dia_depth) * tan(compound_angle);
    }
    // full specified depth now
    depth = end_depth;
    zoff = (depth - full_dia_depth) * tan(compound_angle);
    // cut at least once -- more if spring cuts.
    for(int i = 0; i<spring_cuts+1; i++) {
	threading_pass(settings, block, boring, safe_x, depth, end_depth, start_y,
		       start_z, zoff, taper_dist, entry_taper, exit_taper,
		       taper_pitch, pitch, full_threadheight, target_z);
    }
    STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z, AABBCC);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
#undef AABBC
    return INTERP_OK;
}


/****************************************************************************/

/*! convert_straight_comp1

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The side is not RIGHT or LEFT:
      NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT
   2. The destination tangent point is not more than a tool radius
      away (indicating gouging): NCE_CUTTER_GOUGING_WITH_CUTTER_RADIUS_COMP
   3. The value of move is not G_0 or G_1
      NCE_BUG_CODE_NOT_G0_OR_G1

Side effects:
   This executes a STRAIGHT_MOVE command at cutting feed rate
   or a STRAIGHT_TRAVERSE command.
   It also updates the setting of the position of the tool point
   to the end point of the move and updates the programmed point.

Called by: convert_straight.

This is called if cutter radius compensation is on and
settings->cutter_comp_firstmove is true, indicating that this is the
first move after cutter radius compensation is turned on.

The algorithm used here for determining the path is to draw a straight
line from the destination point which is tangent to a circle whose
center is at the current point and whose radius is the radius of the
cutter. The destination point of the cutter tip is then found as the
center of a circle of the same radius tangent to the tangent line at
the destination point.

*/

int Interp::convert_straight_comp1(int move,     //!< either G_0 or G_1
                                   block_pointer block,  //!< pointer to a block of RS274 instructions
                                   setup_pointer settings,       //!< pointer to machine settings
                                   double px,    //!< X coordinate of end point
                                   double py,    //!< Y coordinate of end point
                                   double pz,    //!< Z coordinate of end point
                                   double AA_end,        //!< A coordinate of end point
                                   double BB_end,        //!< B coordinate of end point
                                   double CC_end,        //!< C coordinate of end point
                                   double u_end, double v_end, double w_end)
{
    double alpha;
    double distance;
    double radius = settings->cutter_comp_radius; /* always will be positive */
    double end_x, end_y;

    CUTTER_COMP side = settings->cutter_comp_side;
    double cx, cy, cz;

    comp_get_current(settings, &cx, &cy, &cz);
    distance = hypot((px - cx), (py - cy));

    CHKS(((side != CUTTER_COMP::LEFT) && (side != CUTTER_COMP::RIGHT)), NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT);
    CHKS((distance <= radius), _("Length of cutter compensation entry move is not greater than the tool radius"));

    alpha = atan2(py - cy, px - cx) + (side == CUTTER_COMP::LEFT ? M_PIl/2. : -M_PIl/2.);

    end_x = (px + (radius * cos(alpha)));
    end_y = (py + (radius * sin(alpha)));

    // with these moves we don't need to record the direction vector.
    // they cannot get reversed because they are guaranteed to be long
    // enough.

    set_endpoint(cx, cy);

    if (move == G_0) {
        enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                  cos(alpha), sin(alpha), 0,
                                  end_x, end_y, pz,
                                  AA_end, BB_end, CC_end, u_end, v_end, w_end);
    }
    else if (move == G_1) {
        enqueue_STRAIGHT_FEED(settings, block->line_number,
                              cos(alpha), sin(alpha), 0,
                              end_x, end_y, pz,
                              AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else
        ERS(NCE_BUG_CODE_NOT_G0_OR_G1);

    settings->cutter_comp_firstmove = false;

    comp_set_current(settings, end_x, end_y, pz);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;
    comp_set_programmed(settings, px, py, pz);
    return INTERP_OK;
}
/****************************************************************************/

/*! convert_straight_comp2

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The compensation side is not RIGHT or LEFT:
      NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT
   2. A concave corner is found:
      NCE_CONCAVE_CORNER_WITH_CUTTER_RADIUS_COMP

Side effects:
   This executes a STRAIGHT_FEED command at cutting feed rate
   or a STRAIGHT_TRAVERSE command.
   It also generates an ARC_FEED to go around a corner, if necessary.
   It also updates the setting of the position of the tool point to
   the end point of the move and updates the programmed point.

Called by: convert_straight.

This is called if cutter radius compensation is on and
settings->cutter_comp_firstmove is not true, indicating that this is not
the first move after cutter radius compensation is turned on.

The algorithm used here is:
1. Determine the direction of the last motion. This is done by finding
   the direction of the line from the last programmed point to the
   current tool tip location. This line is a radius of the tool and is
   perpendicular to the direction of motion since the cutter is tangent
   to that direction.
2. Determine the direction of the programmed motion.
3. If there is a convex corner, insert an arc to go around the corner.
4. Find the destination point for the tool tip. The tool will be
   tangent to the line from the last programmed point to the present
   programmed point at the present programmed point.
5. Go in a straight line from the current tool tip location to the
   destination tool tip location.

This uses an angle tolerance of TOLERANCE_CONCAVE_CORNER (0.01 radian)
to determine if:
1) an illegal concave corner exists (tool will not fit into corner),
2) no arc is required to go around the corner (i.e. the current line
   is in the same direction as the end of the previous move), or
3) an arc is required to go around a convex corner and start off in
   a new direction.

If a rotary axis is moved in this block and an extra arc is required
to go around a sharp corner, all the rotary axis motion occurs on the
arc.  An alternative might be to distribute the rotary axis motion
over the arc and the straight move in proportion to their lengths.

If the Z-axis is moved in this block and an extra arc is required to
go around a sharp corner, all the Z-axis motion occurs on the straight
line and none on the extra arc.  An alternative might be to distribute
the Z-axis motion over the extra arc and the straight line in
proportion to their lengths.

This handles the case of there being no XY motion.

This handles G0 moves. Where an arc is inserted to round a corner in a
G1 move, no arc is inserted for a G0 move; a STRAIGHT_TRAVERSE is made
from the current point to the end point. The end point for a G0
move is the same as the end point for a G1 move, however.

*/

int Interp::convert_straight_comp2(int move,     //!< either G_0 or G_1
                                   block_pointer block,  //!< pointer to a block of RS274 instructions
                                   setup_pointer settings,       //!< pointer to machine settings
                                   double px,    //!< X coordinate of programmed end point
                                   double py,    //!< Y coordinate of programmed end point
                                   double pz,    //!< Z coordinate of end point
                                   double AA_end,        //!< A coordinate of end point
                                   double BB_end,        //!< B coordinate of end point
                                   double CC_end,        //!< C coordinate of end point
                                   double u_end, double v_end, double w_end)
{
    double end_x, end_y, end_z;                 /* x-coordinate of actual end point */
    double mid_x, mid_y;                 /* x-coordinate of end of added arc, if needed */
    double small = TOLERANCE_CONCAVE_CORNER;      /* radians, testing corners */
    double opx = 0, opy = 0, opz = 0;      /* old programmed beginning point */
    double cx, cy, cz;
    int concave;

    comp_get_current(settings, &cx, &cy, &cz);
    comp_get_current(settings, &end_x, &end_y, &end_z);
    comp_get_programmed(settings, &opx, &opy, &opz);

    if ((py == opy) && (px == opx)) {     /* no XY motion */
        if (move == G_0) {
            enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                      px - opx, py - opy, pz - opz,
                                      cx, cy, pz,
                                      AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else if (move == G_1) {
            enqueue_STRAIGHT_FEED(settings, block->line_number,
                                  px - opx, py - opy, pz - opz,
                                  cx, cy, pz, AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else
            ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        // end already filled out, above
    } else {
        // some XY motion
        double beta = 0.0;  // initializing to avoid confusion over else branch below
        double gamma = 0.0; // that does not define value but returns form function.
        CUTTER_COMP side = settings->cutter_comp_side;
        double radius = settings->cutter_comp_radius;      /* will always be positive */
        double theta = atan2(cy - opy, cx - opx);
        double alpha = atan2(py - opy, px - opx);

        if (side == CUTTER_COMP::LEFT) {
            if (theta < alpha)
                theta = (theta + (2 * M_PIl));
            beta = ((theta - alpha) - M_PI_2l);
            gamma = M_PI_2l;
        } else if (side == CUTTER_COMP::RIGHT) {
            if (alpha < theta)
                alpha = (alpha + (2 * M_PIl));
            beta = ((alpha - theta) - M_PI_2l);
            gamma = -M_PI_2l;
        } else {
            ERS(NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT);
            // the ERS macro will return from this function
        }
        end_x = (px + (radius * cos(alpha + gamma)));
        end_y = (py + (radius * sin(alpha + gamma)));
        mid_x = (opx + (radius * cos(alpha + gamma)));
        mid_y = (opy + (radius * sin(alpha + gamma)));

        if ((beta < -small) || (beta > (M_PIl + small))) {
            concave = 1;
        } else if (beta > (M_PIl - small) &&
                   (!qc().empty() && qc().front().type == QARC_FEED &&
                    ((side == CUTTER_COMP::RIGHT && qc().front().data.arc_feed.turn > 0) ||
                     (side == CUTTER_COMP::LEFT && qc().front().data.arc_feed.turn < 0)))) {
            // this is an "h" shape, tool on right, going right to left
            // over the hemispherical round part, then up next to the
            // vertical part (or, the mirror case).  there are two ways
            // to stay to the "right", either loop down and around, or
            // stay above and right.  we're forcing above and right.
            concave = 1;
        } else {
            concave = 0;
            mid_x = (opx + (radius * cos(alpha + gamma)));
            mid_y = (opy + (radius * sin(alpha + gamma)));
        }

        if (!concave && (beta > small)) {       /* ARC NEEDED */
            CHP(move_endpoint_and_flush(settings, cx, cy));
            if(move == G_1) {
                enqueue_ARC_FEED(settings, block->line_number,
                                 0.0, // doesn't matter, since we will not move this arc's endpoint
                                 mid_x, mid_y, opx, opy,
                                 ((side == CUTTER_COMP::LEFT) ? -1 : 1), cz,
                                 AA_end, BB_end, CC_end, u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } else if(move == G_0) {
                // we can't go around the corner because there is no
                // arc traverse.  but, if we do this anyway, at least
                // most of our rapid will be parallel to the original
                // programmed one.  if nothing else, this will look a
                // little less confusing in the preview.
                enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                          0.0, 0.0, 0.0,
                                          mid_x, mid_y, cz,
                                          AA_end, BB_end, CC_end,
                                          u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } else ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        } else if (concave) {
            if (qc().front().type != QARC_FEED) {
                // line->line
                double retreat;
                // half the angle of the inside corner
                double halfcorner = (beta + M_PIl) / 2.0;
                CHKS((halfcorner == 0.0), (_("Zero degree inside corner is invalid for cutter compensation")));
                retreat = radius / tan(halfcorner);
                // move back along the compensated path
                // this should replace the endpoint of the previous move
                mid_x = cx + retreat * cos(theta + gamma);
                mid_y = cy + retreat * sin(theta + gamma);
                // we actually want to move the previous line's endpoint here.  That's the same as
                // discarding that line and doing this one instead.
                CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            } else {
                // arc->line
                // beware: the arc we saved is the compensated one.
                arc_feed prev = qc().front().data.arc_feed;
                double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
                double oldrad_uncomp;

                // new line's direction
                double base_dir = atan2(py - opy, px - opx);
                double theta;
                double phi;

                theta = (prev.turn > 0) ? base_dir + M_PI_2l : base_dir - M_PI_2l;
                phi = atan2(prev.center2 - opy, prev.center1 - opx);
                if TOOL_INSIDE_ARC(side, prev.turn) {
                    oldrad_uncomp = oldrad + radius;
                } else {
                    oldrad_uncomp = oldrad - radius;
                }

                double alpha = theta - phi;
                // distance to old arc center perpendicular to the new line
                double d = oldrad_uncomp * cos(alpha);
                double d2;
                double angle_from_center;

                if TOOL_INSIDE_ARC(side, prev.turn) {
                    d2 = d - radius;
                    double l = d2/oldrad;
                    CHKS((l > 1.0 || l < -1.0), _("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn > 0)
                        angle_from_center = - acos(l) + theta + M_PIl;
                    else
                        angle_from_center = acos(l) + theta + M_PIl;
                } else {
                    d2 = d + radius;
                    double l = d2/oldrad;
                    CHKS((l > 1.0 || l < -1.0), _("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn > 0)
                        angle_from_center = acos(l) + theta + M_PIl;
                    else
                        angle_from_center = - acos(l) + theta + M_PIl;
                }
                mid_x = prev.center1 + oldrad * cos(angle_from_center);
                mid_y = prev.center2 + oldrad * sin(angle_from_center);
                CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            }
        } else {
            // no arc needed, also not concave (colinear lines or tangent arc->line)
            dequeue_canons(settings);
            set_endpoint(cx, cy);
        }
        (move == G_0? enqueue_STRAIGHT_TRAVERSE: enqueue_STRAIGHT_FEED)
            (settings, block->line_number,
             px - opx, py - opy, pz - opz,
             end_x, end_y, pz,
             AA_end, BB_end, CC_end,
             u_end, v_end, w_end);
    }

    comp_set_current(settings, end_x, end_y, pz);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;
    comp_set_programmed(settings, px, py, pz);
    return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_change

Returned Value: int (INTERP_OK)

Side effects:
   This makes function calls to canonical machining functions, and sets
   the machine model as described below.

Called by: convert_m

This function carries out an M6 command, which changes the tool.
If M61 is called, the toolnumber gets changed (without causing an actual toolchange).

When the CHANGE_TOOL call completes, the specified tool should be
loaded.  What this means varies by machine.  According to configuration,
the interpreter may also issue commands to do one or more of the
following things before calling CHANGE_TOOL:

1. stop the spindle
2. move the quill up (Z to machine zero, like G0 G53 Z0)
3. move the axes to reference point #2 (like G30)

Further, the interpreter makes no assumptions about the axis positions
after the tool change completes.  This state is queried and the internal
model is resynched before the program continues.  This means CHANGE_TOOL
itself can also issue motion (and it currently may, according to
configuration).

This implements the "Next tool in T word" approach to tool selection.
The tool is selected when the T word is read (and the carousel may
move at that time) but is changed when M6 is read.

Note that if a different tool is put into the spindle, the current_z
location setting will be incorrect. It is assumed the program will
contain an appropriate USE_TOOL_LENGTH_OFFSET (G43) command before any
subsequent motion.  It is also assumed that the program will restart the
spindle and make new entry moves if necessary.

*/

int Interp::convert_tool_change(setup_pointer settings)  //!< pointer to machine settings
{

  if (settings->selected_pocket < 0) {
    ERS(NCE_TXX_MISSING_FOR_M6);
  }

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
       (_("Cannot change tools with cutter radius compensation on")));

  if (!settings->tool_change_with_spindle_on) {
	  for (int s = 0; s < settings->num_spindles; s++){
		  STOP_SPINDLE_TURNING(s);
		  settings->spindle_turning[s] = CANON_STOPPED;
	  }
  }

  if (settings->tool_change_quill_up) {
      double up_z;
      double discard;
      find_relative(0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    &discard, &discard, &up_z,
                    &discard, &discard, &discard,
                    &discard, &discard, &discard,
                    settings);
      COMMENT("AXIS,hide");
      STRAIGHT_TRAVERSE(-1, settings->current_x, settings->current_y, up_z,
                        settings->AA_current, settings->BB_current, settings->CC_current,
                        settings->u_current, settings->v_current, settings->w_current);
      COMMENT("AXIS,show");
      settings->current_z = up_z;
  }

  if (settings->tool_change_at_g30) {
      double end_x;
      double end_y;
      double end_z;
      double AA_end;
      double BB_end;
      double CC_end;
      double u_end;
      double v_end;
      double w_end;

      find_relative(USER_TO_PROGRAM_LEN(settings->parameters[5181]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5182]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5183]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5184]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5185]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5186]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5187]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5188]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5189]),
                    &end_x, &end_y, &end_z,
                    &AA_end, &BB_end, &CC_end,
                    &u_end, &v_end, &w_end, settings);
      COMMENT("AXIS,hide");

      // move indexers first, one at a time
      // JOINTS_AXES settings->*_indexer_jnum == -1 means notused
      if (AA_end != settings->AA_current && (-1 != settings->a_indexer_jnum) )
          issue_straight_index(3,settings->a_indexer_jnum, AA_end, -1, settings);
      if (BB_end != settings->BB_current && (-1 != settings->b_indexer_jnum) )
          issue_straight_index(4,settings->b_indexer_jnum, BB_end, -1, settings);
      if (CC_end != settings->CC_current && (-1 != settings->c_indexer_jnum) )
          issue_straight_index(5,settings->c_indexer_jnum, CC_end, -1, settings);

      STRAIGHT_TRAVERSE(-1, end_x, end_y, end_z,
                        AA_end, BB_end, CC_end,
                        u_end, v_end, w_end);
      COMMENT("AXIS,show");
      settings->current_x = end_x;
      settings->current_y = end_y;
      settings->current_z = end_z;
      settings->AA_current = AA_end;
      settings->BB_current = BB_end;
      settings->CC_current = CC_end;
      settings->u_current = u_end;
      settings->v_current = v_end;
      settings->w_current = w_end;
  }

  CHANGE_TOOL();

  settings->current_pocket = settings->selected_pocket;
  // tool change can move the controlled point.  reread it:
  settings->toolchange_flag = true;
  set_tool_parameters();
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_length_offset

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The block has no offset index (h number): NCE_OFFSET_INDEX_MISSING
   2. The g_code argument is not G_43 or G_49:
      NCE_BUG_CODE_NOT_G43_OR_G49

Side effects:
   A USE_TOOL_LENGTH_OFFSET function call is made. Current_z,
   tool_length_offset, and length_offset_index are reset.

Called by: convert_g

This is called to execute g43 or g49.

The g49 RS274/NGC command translates into a USE_TOOL_LENGTH_OFFSET(0.0)
function call.

The g43 RS274/NGC command translates into a USE_TOOL_LENGTH_OFFSET(length)
function call, where length is the value of the entry in the tool length
offset table whose index is the H number in the block.

The H number in the block (if present) was checked for being a non-negative
integer when it was read, so that check does not need to be repeated.

*/

int Interp::convert_tool_length_offset(int g_code,       //!< g_code being executed (must be G_43 or G_49)
                                      block_pointer block,      //!< pointer to a block of RS274/NGC instructions
                                      setup_pointer settings)   //!< pointer to machine settings
{
  int idx;
  EmcPose tool_offset;
  ZERO_EMC_POSE(tool_offset);

  CHKS((settings->cutter_comp_side != CUTTER_COMP::OFF),
       (_("Cannot change tool offset with cutter radius compensation on")));
  if (g_code == G_49) {
    idx = 0;
  } else if (g_code == G_43) {
      logDebug("convert_tool_length_offset h_flag=%d h_number=%d toolchange_flag=%d current_pocket=%d\n",
	      block->h_flag,block->h_number,settings->toolchange_flag,settings->current_pocket);
    if(block->h_flag) {
        CHP((find_tool_index(settings, block->h_number, &idx)));
         //sequence:
         //  t12m6
         //  g43      should      use t12 offset
         //  g43h12   should also use t12 offset
        if (idx==0 && settings->random_toolchanger) {
           idx=settings->current_pocket;
        }
    } else if (settings->toolchange_flag) {
        // Tool change is in progress, so the "current tool" is in its
        // original index still.
        idx = settings->current_pocket;
    } else {
        // Tool change is done so the current tool is in index 0 (aka the
        // spindle).
        idx = 0;
    }
    logDebug("convert_tool_length_offset: using index=%d spindle_toolno=%d pocket_toolno=%d",
	     idx, settings->tool_table[0].toolno,settings->tool_table[settings->current_pocket].toolno);

    tool_offset.tran.x = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.x);
    tool_offset.tran.y = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.y);
    tool_offset.tran.z = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.z);
    tool_offset.a = USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.a);
    tool_offset.b = USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.b);
    tool_offset.c = USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.c);
    tool_offset.u = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.u);
    tool_offset.v = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.v);
    tool_offset.w = USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.w);
  } else if (g_code == G_43_1) {
    tool_offset = settings->tool_offset;
    idx = -1;
    if(block->x_flag) tool_offset.tran.x = block->x_number;
    if(block->y_flag) tool_offset.tran.y = block->y_number;
    if(block->z_flag) tool_offset.tran.z = block->z_number;
    if(block->a_flag) tool_offset.a = block->a_number;
    if(block->b_flag) tool_offset.b = block->b_number;
    if(block->c_flag) tool_offset.c = block->c_number;
    if(block->u_flag) tool_offset.u = block->u_number;
    if(block->v_flag) tool_offset.v = block->v_number;
    if(block->w_flag) tool_offset.w = block->w_number;
  } else if (g_code == G_43_2) {
    CHKS((block->h_flag && (block->x_flag || block->y_flag || block->z_flag
         || block->a_flag || block->b_flag || block->c_flag || block->u_flag
         || block->v_flag || block->w_flag)), (_("G43.2: Can not have both H and axis words")));
    CHKS((!block->h_flag && !block->x_flag && !block->y_flag && !block->z_flag
         && !block->a_flag &&!block->b_flag && !block->c_flag && !block->u_flag
         && !block->v_flag && !block->w_flag), (_("G43.2: No axes specified and H word missing")));
    tool_offset = settings->tool_offset;
    if (block->h_flag){
        CHP((find_tool_index(settings, block->h_number, &idx)));
        tool_offset.tran.x += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.x);
        tool_offset.tran.y += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.y);
        tool_offset.tran.z += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.tran.z);
        tool_offset.a += USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.a);
        tool_offset.b += USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.b);
        tool_offset.c += USER_TO_PROGRAM_ANG(settings->tool_table[idx].offset.c);
        tool_offset.u += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.u);
        tool_offset.v += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.v);
        tool_offset.w += USER_TO_PROGRAM_LEN(settings->tool_table[idx].offset.w);
    } else {
        if(block->x_flag) tool_offset.tran.x += block->x_number;
        if(block->y_flag) tool_offset.tran.y += block->y_number;
        if(block->z_flag) tool_offset.tran.z += block->z_number;
        if(block->a_flag) tool_offset.a += block->a_number;
        if(block->b_flag) tool_offset.b += block->b_number;
        if(block->c_flag) tool_offset.c += block->c_number;
        if(block->u_flag) tool_offset.u += block->u_number;
        if(block->v_flag) tool_offset.v += block->v_number;
        if(block->w_flag) tool_offset.w += block->w_number;
    }
  } else {
    ERS("BUG: Code not G43, G43.1, G43.2, or G49");
  }
  USE_TOOL_LENGTH_OFFSET(tool_offset);

  double dx, dy;

  dx = settings->tool_offset.tran.x - tool_offset.tran.x;
  dy = settings->tool_offset.tran.y - tool_offset.tran.y;

  rotate(&dx, &dy, -settings->rotation_xy);

  settings->current_x += dx;
  settings->current_y += dy;
  settings->current_z += settings->tool_offset.tran.z - tool_offset.tran.z;
  settings->AA_current += settings->tool_offset.a - tool_offset.a;
  settings->BB_current += settings->tool_offset.b - tool_offset.b;
  settings->CC_current += settings->tool_offset.c - tool_offset.c;
  settings->u_current += settings->tool_offset.u - tool_offset.u;
  settings->v_current += settings->tool_offset.v - tool_offset.v;
  settings->w_current += settings->tool_offset.w - tool_offset.w;

  settings->tool_offset = tool_offset;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_select

Returned Value: int
   If the tool number given in the block is not found in the tool table,
   it returns INTERP_ERROR.  Otherwise (if the tool *is* found) it returns
   INTERP_OK.

Side effects: See below

Called by: execute_block

A select tool command is given, which causes the changer chain to move
so that the slot with the tool identified by the t_number given in the
block is next to the tool changer, ready for a tool change.  The
settings->selected_tool_slot is set to the given slot.

A check that the t_number is not negative has already been made in read_t.
A zero t_number is allowed and means no tool should be selected.

*/

// OK to select tool in a concave corner, I think?

int Interp::convert_tool_select(block_pointer block,     //!< pointer to a block of RS274 instructions
                               setup_pointer settings)  //!< pointer to machine settings
{
  int idx;
  CHP((find_tool_index(settings, block->t_number, &idx)));
  SELECT_TOOL(block->t_number);
  settings->selected_pocket = idx;
  settings->selected_tool = block->t_number;
  return INTERP_OK;
}


int Interp::update_tag(StateTag &tag)
{
    UPDATE_TAG(tag);
    return INTERP_OK;
}
