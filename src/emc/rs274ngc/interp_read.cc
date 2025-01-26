/********************************************************************
* Description: interp_read.cc
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
#include <sstream>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"
#include "rtapi_math.h"
#include <cmath>
#include <rtapi_string.h>	// rtapi_strlcpy()

using namespace interp_param_global;

/****************************************************************************/

/*! read_a

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not a:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An a_coordinate has already been inserted in the block:
      NCE_MULTIPLE_A_WORDS_ON_ONE_LINE.
   3. A values are not allowed: NCE_CANNOT_USE_A_WORD.

Side effects:
   counter is reset.
   The a_flag in the block is turned on.
   An a_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'a', indicating an a_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
The counter is then set to point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

If the AA compiler flag is defined, the a_flag in the block is turned
on and the a_number in the block is set to the value read. If the
AA flag is not defined, (i) if the AXIS_ERROR flag is defined, that means
A values are not allowed, and an error value is returned, (ii) if the
AXIS_ERROR flag is not defined, nothing is done.

*/

int Interp::read_a(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'a'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->a_flag), NCE_MULTIPLE_A_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->a_flag = true;
  block->a_number = value;
  return INTERP_OK;
}


/****************************************************************************/

/*! read_atan

Returned Value: int
   If read_real_expression returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character to read is not a slash:
      NCE_SLASH_MISSING_AFTER_FIRST_ATAN_ARGUMENT
   2. The second character to read is not a left bracket:
      NCE_LEFT_BRACKET_MISSING_AFTER_SLASH_WITH_ATAN

Side effects:
   The computed value is put into what double_ptr points at.
   The counter is reset to point to the first character after the
   characters which make up the value.

Called by:
   read_unary

When this function is called, the characters "atan" and the first
argument have already been read, and the value of the first argument
is stored in double_ptr.  This function attempts to read a slash and
the second argument to the atan function, starting at the index given
by the counter and then to compute the value of the atan operation
applied to the two arguments.  The computed value is inserted into
what double_ptr points at.

The computed value is in the range from -180 degrees to +180 degrees.
The range is not specified in the RS274/NGC manual [NCMS, page 51],
although using degrees (not radians) is specified.

*/

int Interp::read_atan(char *line,        //!< string: line of RS274/NGC code being processed
                     int *counter,      //!< pointer to a counter for position on line     
                     double *double_ptr,        //!< pointer to double to be read                  
                     double *parameters)        //!< array of system parameters                    
{
  double argument2;

  CHKS((line[*counter] != '/'), NCE_SLASH_MISSING_AFTER_FIRST_ATAN_ARGUMENT);
  *counter = (*counter + 1);
  CHKS((line[*counter] != '['),
      NCE_LEFT_BRACKET_MISSING_AFTER_SLASH_WITH_ATAN);
  CHP(read_real_expression(line, counter, &argument2, parameters));
  *double_ptr = atan2(*double_ptr, argument2);  /* value in radians */
  *double_ptr = ((*double_ptr * 180.0) / M_PIl);   /* convert to degrees */
  return INTERP_OK;
}

/****************************************************************************/

/*! read_b

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not b:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A b_coordinate has already been inserted in the block:
      NCE_MULTIPLE_B_WORDS_ON_ONE_LINE.
   3. B values are not allowed: NCE_CANNOT_USE_B_WORD

Side effects:
   counter is reset.
   The b_flag in the block is turned on.
   A b_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'b', indicating a b_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
The counter is then set to point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

If the BB compiler flag is defined, the b_flag in the block is turned
on and the b_number in the block is set to the value read. If the
BB flag is not defined, (i) if the AXIS_ERROR flag is defined, that means
B values are not allowed, and an error value is returned, (ii) if the
AXIS_ERROR flag is not defined, nothing is done.

*/

int Interp::read_b(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'b'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->b_flag), NCE_MULTIPLE_B_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->b_flag = true;
  block->b_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_c

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not c:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An c_coordinate has already been inserted in the block:
      NCE_MULTIPLE_C_WORDS_ON_ONE_LINE
   3. C values are not allowed: NCE_CANNOT_USE_C_WORD

Side effects:
   counter is reset.
   The c_flag in the block is turned on.
   A c_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'c', indicating an c_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
The counter is then set to point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

If the CC compiler flag is defined, the c_flag in the block is turned
on and the c_number in the block is set to the value read. If the
CC flag is not defined, (i) if the AXIS_ERROR flag is defined, that means
C values are not allowed, and an error value is returned, (ii) if the
AXIS_ERROR flag is not defined, nothing is done.

*/

int Interp::read_c(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'c'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->c_flag), NCE_MULTIPLE_C_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->c_flag = true;
  block->c_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_comment

Returned Value: int
  If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not '(' ,
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   The counter is reset to point to the character following the comment.
   The comment string, without parentheses, is copied into the comment
   area of the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character '(', indicating a comment is
beginning. The function reads characters of the comment, up to and
including the comment closer ')'.

It is expected that the format of a comment will have been checked (by
read_text or read_keyboard_line) and bad format comments will
have prevented the system from getting this far, so that this function
can assume a close parenthesis will be found when an open parenthesis
has been found, and that comments are not nested.

The "parameters" argument is not used in this function. That argument is
present only so that this will have the same argument list as the other
"read_XXX" functions called using a function pointer by read_one_item.

*/

int Interp::read_comment(char *line,     //!< string: line of RS274 code being processed   
                        int *counter,   //!< pointer to a counter for position on the line
                        block_pointer block,    //!< pointer to a block being filled from the line
                        double * /*parameters*/)     //!< array of system parameters
{
  int n;

  CHKS((line[*counter] != '('), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  (*counter)++;
  for (n = 0; line[*counter] != ')'; (*counter)++, n++) {
    block->comment[n] = line[*counter];
  }
  block->comment[n] = 0;
  (*counter)++;
  return INTERP_OK;
}

// With the exception of lines starting with ';py,',
// Everything after a semicolon will be ignored/treated as a comment until the end of the line.

int Interp::read_semicolon(char *line,     //!< string: line of RS274 code being processed   
                           int *counter,   //!< pointer to a counter for position on the line
                           block_pointer /*block*/,    //!< pointer to a block being filled from the line
                           double * /*parameters*/)     //!< array of system parameters
{
    CHKS((line[*counter] != ';'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
    (*counter) = strlen(line);
    // pass unmutilated line starting with ';py,' to convert_comment - FIXME access to _setup
    if (strncmp (_setup.linetext, ";py,", 4) == 0)
	CHP(convert_comment(_setup.linetext+1, false));
    return INTERP_OK;
}

/****************************************************************************/

/*! read_d

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not d:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A d_number has already been inserted in the block:
      NCE_MULTIPLE_D_WORDS_ON_ONE_LINE

Side effects:
   counter is reset to the character following the tool number.
   A d_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'd', indicating an index into a
table of tool diameters.  The function reads characters which give the
(integer) value of the index. The value may not be more than
_setup.tool_max and may not be negative, but it may be zero. The range
is checked here.

read_integer_value allows a minus sign, so a check for a negative value
is made here, and the parameters argument is also needed.

*/

int Interp::read_d(char *line,   //!< string: line of RS274 code being processed   
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters                   
{
  double value;

  CHKS((line[*counter] != 'd'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->d_flag), NCE_MULTIPLE_D_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->d_number_float = value;
  block->d_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_dollar

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not d:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A dollar_number has already been inserted in the block:
      NCE_MULTIPLE_$_WORDS_ON_ONE_LINE

Side effects:
   counter is reset to the character following the tool number.
   A d_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'd', indicating an index into a
table of tool diameters.  The function reads characters which give the
(integer) value of the index. The value may not be more than
_setup.tool_max and may not be negative, but it may be zero. The range
is checked here.

read_integer_value allows a minus sign, so a check for a negative value
is made here, and the parameters argument is also needed.

*/

int Interp::read_dollar(char *line,   //!< string: line of RS274 code being processed
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters
{
  double value;
  CHKS((line[*counter] != '$'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->dollar_flag), NCE_MULTIPLE_$_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->dollar_number = value;
  block->dollar_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_e

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not e:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A e value has already been inserted in the block:
      NCE_MULTIPLE_E_WORDS_ON_ONE_LINE

Side effects:
   counter is reset to point to the first character following the e value.
   The e value setting is inserted in block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'e', indicating a e value
setting. The function reads characters which tell how to set the e
value, up to the start of the next item or the end of the line. This
information is inserted in the block.

E codes are used for:
Infeed/Outfeed angle specification with G76

*/

int Interp::read_e(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'e'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->e_flag), NCE_MULTIPLE_E_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->e_flag = true;
  block->e_number = value;
  return INTERP_OK;
}


/****************************************************************************/

/*! read_f

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not f:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An f_number has already been inserted in the block:
      NCE_MULTIPLE_F_WORDS_ON_ONE_LINE
   3. The f_number is negative: NCE_NEGATIVE_F_WORD_USED

Side effects:
   counter is reset to point to the first character following the f_number.
   The f_number is inserted in block and the f_flag is set.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'f'. The function reads characters
which tell how to set the f_number, up to the start of the next item
or the end of the line. This information is inserted in the block.

The value may be a real number or something that evaluates to a real
number, so read_real_value is used to read it. Parameters may be
involved, so the parameters argument is required. The value is always
a feed rate.

*/

int Interp::read_f(char *line,   //!< string: line of RS274 code being processed   
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters                   
{
  double value;

  CHKS((line[*counter] != 'f'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->f_flag), NCE_MULTIPLE_F_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  CHKS((value < 0.0), NCE_NEGATIVE_F_WORD_USED);
  block->f_number = value;
  block->f_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_g

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not g:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The value is negative: NCE_NEGATIVE_G_CODE_USED
   3. The value differs from a number ending in an even tenth by more
      than 0.0001: NCE_G_CODE_OUT_OF_RANGE
   4. The value is greater than 99.9: NCE_G_CODE_OUT_OF_RANGE
   5. The value is not the number of a valid g code: NCE_UNKNOWN_G_CODE_USED
   6. Another g code from the same modal group has already been
      inserted in the block: NCE_TWO_G_CODES_USED_FROM_SAME_MODAL_GROUP

Side effects:
   counter is reset to the character following the end of the g_code.
   A g code is inserted as the value of the appropriate mode in the
   g_modes array in the block.
   The g code counter in the block is increased by 1.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'g', indicating a g_code.  The
function reads characters which tell how to set the g_code.

The RS274/NGC manual [NCMS, page 51] allows g_codes to be represented
by expressions and provide [NCMS, 71 - 73] that a g_code must evaluate
to to a number of the form XX.X (59.1, for example). The manual does not
say how close an expression must come to one of the allowed values for
it to be legitimate. Is 59.099999 allowed to mean 59.1, for example?
In the interpreter, we adopt the convention that the evaluated number
for the g_code must be within 0.0001 of a value of the form XX.X

To simplify the handling of g_codes, we convert them to integers by
multiplying by 10 and rounding down or up if within 0.001 of an
integer. Other functions that deal with g_codes handle them
symbolically, however. The symbols are defined in rs274NGC.hh
where G_1 is 10, G_83 is 830, etc.

This allows any number of g_codes on one line, provided that no two
are in the same modal group.

This allows G80 on the same line as one other g_code with the same
mode. If this happens, the G80 is simply ignored.

*/

int Interp::read_g(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value_read;
  int value;
  int mode;

  CHKS((line[*counter] != 'g'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHP(read_real_value(line, counter, &value_read, parameters));
  value_read = (10.0 * value_read);
  value = (int) floor(value_read);

  if ((value_read - value) > 0.999)
    value = (int) ceil(value_read);
  else if ((value_read - value) > 0.001)
    ERS(NCE_G_CODE_OUT_OF_RANGE);

  CHKS((value > 999), NCE_G_CODE_OUT_OF_RANGE);
  CHKS((value < 0), NCE_NEGATIVE_G_CODE_USED);
  // mode = usercode_mgroup(&(_setup),value);
  // if (mode != -1) {

 remap_pointer r = _setup.g_remapped[value];
  if (r) {
      mode =  r->modal_group;
      CHKS ((mode < 0),"BUG: G remapping: modal group < 0"); // real bad

      CHKS((block->g_modes[mode] != -1),
	   NCE_TWO_G_CODES_USED_FROM_SAME_MODAL_GROUP);
      block->g_modes[mode] = value;
      return INTERP_OK;
  }
  mode = gees[value];
  CHKS((mode == -1), NCE_UNKNOWN_G_CODE_USED);
  if ((value == G_80) && (block->g_modes[mode] != -1));
  else {
    if (block->g_modes[mode] == G_80);
    else {
      CHKS((block->g_modes[mode] != -1),
          NCE_TWO_G_CODES_USED_FROM_SAME_MODAL_GROUP);
    }
    block->g_modes[mode] = value;
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_h

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not h:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An h_number has already been inserted in the block:
      NCE_MULTIPLE_H_WORDS_ON_ONE_LINE
   3. The value is negative: NCE_NEGATIVE_H_WORD_TOOL_LENGTH_OFFSET_INDEX_USED
   4. The value is greater than _setup.tool_max:
      NCE_TOOL_LENGTH_OFFSET_INDEX_TOO_BIG

Side effects:
   counter is reset to the character following the h_number.
   An h_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'h', indicating a tool length
offset index.  The function reads characters which give the (integer)
value of the tool length offset index (not the actual distance of the
offset).

*/

int Interp::read_h(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  int value;

  CHKS((line[*counter] != 'h'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->h_flag), NCE_MULTIPLE_H_WORDS_ON_ONE_LINE);
  CHP(read_integer_value(line, counter, &value, parameters));
  CHKS((value < -1), NCE_NEGATIVE_H_WORD_USED);
  block->h_flag = true;
  block->h_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_i

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not i:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An i_coordinate has already been inserted in the block:
      NCE_MULTIPLE_I_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The i_flag in the block is turned on.
   An i_coordinate setting is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'i', indicating a i_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_i(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'i'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->i_flag), NCE_MULTIPLE_I_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->i_flag = true;
  block->i_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_integer_unsigned

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, INTERP_OK is returned.
   1. The first character is not a digit: NCE_BAD_FORMAT_UNSIGNED_INTEGER
   2. sscanf fails: NCE_SSCANF_FAILED

Side effects:
   The number read from the line is put into what integer_ptr points at.

Called by: read_n_number

This reads an explicit unsigned (positive) integer from a string,
starting from the position given by *counter. It expects to find one
or more digits. Any character other than a digit terminates reading
the integer. Note that if the first character is a sign (+ or -),
an error will be reported (since a sign is not a digit).

*/

int Interp::read_integer_unsigned(char *line,    //!< string: line of RS274 code being processed   
                                 int *counter,  //!< pointer to a counter for position on the line
                                 int *integer_ptr)      //!< pointer to the value being read              
{
  int n;
  char c;

  for (n = *counter;; n++) {
    c = line[n];
    if ((c < 48) || (c > 57))
      break;
  }
  CHKS((n == *counter), NCE_BAD_FORMAT_UNSIGNED_INTEGER);
  if (sscanf(line + *counter, "%d", integer_ptr) == 0)
    ERS(NCE_SSCANF_FAILED);
  *counter = n;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_integer_value

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The returned value is not close to an integer:
      NCE_NON_INTEGER_VALUE_FOR_INTEGER

Side effects:
   The number read from the line is put into what integer_ptr points at.

Called by:
   read_d
   read_l
   read_h
   read_m
   read_parameter
   read_parameter_setting
   read_t

This reads an integer (positive, negative or zero) from a string,
starting from the position given by *counter. The value being
read may be written with a decimal point or it may be an expression
involving non-integers, as long as the result comes out within 0.0001
of an integer.

This proceeds by calling read_real_value and checking that it is
close to an integer, then returning the integer it is close to.

*/

int Interp::read_integer_value(char *line,       //!< string: line of RS274/NGC code being processed
                              int *counter,     //!< pointer to a counter for position on the line 
                              int *integer_ptr, //!< pointer to the value being read               
                              double *parameters)       //!< array of system parameters                    
{
  double float_value;

  CHP(read_real_value(line, counter, &float_value, parameters));
  *integer_ptr = (int) floor(float_value);
  if ((float_value - *integer_ptr) > 0.9999) {
    *integer_ptr = (int) ceil(float_value);
  } else if ((float_value - *integer_ptr) > 0.0001)
    ERS(NCE_NON_INTEGER_VALUE_FOR_INTEGER);
  return INTERP_OK;
}

/****************************************************************************/

/*! read_items

Returned Value: int
   If read_n_number or read_one_item returns an error code,
   this returns that code.
   Otherwise, it returns INTERP_OK.

Side effects:
   One line of RS274 code is read and data inserted into a block.
   The counter which is passed around among the readers is initialized.
   System parameters may be reset.

Called by: parse_line

*/

int Interp::read_items(block_pointer block,      //!< pointer to a block being filled from the line 
                      char *line,       //!< string: line of RS274/NGC code being processed
                      double *parameters)   //!< array of system parameters 
{
  int counter;
  int m_number, m_counter;  // for checking m98/m99 as o-words
  int length;

  length = strlen(line);
  counter = 0;

  if (line[counter] == '/')     /* skip the slash character if first */
    counter++;

  if (line[counter] == 'n') {
    CHP(read_n_number(line, &counter, block));
  }

  // Pre-check for M code, used in following logic
  if (! (line[counter] == 'm' &&
	 read_integer_value(line, &(m_counter=counter+1), &m_number,
			    parameters) == INTERP_OK))
      m_number = -1;

  if (line[counter] == 'o' || m_number == 98 ||
      (m_number == 99 && _setup.call_level > 0))

 /* Handle 'o', 'm98' and 'm99' sub return (but not 'm99' endless
    program) explicitly here. Default is to read letters via pointer
    calls to related reader functions. 'o' control lines have their
    own commands and command handlers. */
  {
      CHP(read_o(line, &counter, block, parameters));

      // if skipping, the conditionals are not evaluated and are therefore unconsumed
      // so we can't check the rest of the line.  but don't worry, we'll get it later
      if(_setup.skipping_o) return INTERP_OK;

      // after if [...], etc., nothing is allowed except comments
      for (; counter < length;) {
          if(line[counter] == ';') read_semicolon(line, &counter, block, parameters);
          else if (line[counter] == '(') read_comment(line, &counter, block, parameters);
          else ERS("Unexpected character after O-word");
      }
      return INTERP_OK;
  }

  // non O-lines

  if(_setup.skipping_o)
  {
      // if we are skipping, do NOT evaluate non-olines
      return INTERP_OK;
  }

  for (; counter < length;) {
    CHP(read_one_item(line, &counter, block, parameters));
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_j

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not j:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A j_coordinate has already been inserted in the block.
      NCE_MULTIPLE_J_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The j_flag in the block is turned on.
   A j_coordinate setting is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'j', indicating a j_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a real
number, so read_real_value is used to read it. Parameters may be
involved.

*/

int Interp::read_j(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'j'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->j_flag), NCE_MULTIPLE_J_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->j_flag = true;
  block->j_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_k

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not k:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A k_coordinate has already been inserted in the block:
      NCE_MULTIPLE_K_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The k_flag in the block is turned on.
   A k_coordinate setting is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'k', indicating a k_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a real
number, so read_real_value is used to read it. Parameters may be
involved.

*/

int Interp::read_k(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'k'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->k_flag), NCE_MULTIPLE_K_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->k_flag = true;
  block->k_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_l

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not l:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An l_number has already been inserted in the block:
      NCE_MULTIPLE_L_WORDS_ON_ONE_LINE
   3. the l_number is negative: NCE_NEGATIVE_L_WORD_USED

Side effects:
   counter is reset to the character following the l number.
   An l code is inserted in the block as the value of l.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'l', indicating an L code.
The function reads characters which give the (integer) value of the
L code.

L codes are used for:
1. the number of times a canned cycle should be repeated.
2. a key with G10.

*/

int Interp::read_l(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  int value;

  CHKS((line[*counter] != 'l'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->l_number > -1), NCE_MULTIPLE_L_WORDS_ON_ONE_LINE);
  CHP(read_integer_value(line, counter, &value, parameters));
  CHKS((value < 0), NCE_NEGATIVE_L_WORD_USED);
  block->l_number = value;
  block->l_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_n_number

Returned Value: int
   If read_integer_unsigned returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not n:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The line number is too large (more than 99999):
      NCE_LINE_NUMBER_GREATER_THAN_99999

Side effects:
   counter is reset to the character following the line number.
   A line number is inserted in the block.

Called by: read_items

When this function is called, counter is pointing at an item on the
line that starts with the character 'n', indicating a line number.
The function reads characters which give the (integer) value of the
line number.

Note that extra initial zeros in a line number will not cause the
line number to be too large.

*/

int Interp::read_n_number(char *line, //!< string: line of RS274    code being processed 
                          int *counter,       //!< pointer to a counter for position on the line 
                          block_pointer block)        //!< pointer to a block being filled from the line 
{
  int value;

  CHKS(((line[*counter] != 'n') && (line[*counter] != 'o')),
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHP(read_integer_unsigned(line, counter, &value));
  /* This next test is problematic as many CAM systems will exceed this !
  CHKS((value > 99999), NCE_LINE_NUMBER_GREATER_THAN_99999); */
  block->n_number = value;

  // accept & ignore fractional line numbers
  if (line[*counter] == '.') {
      *counter = (*counter + 1);
      CHP(read_integer_unsigned(line, counter, &value));
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_m

Returned Value:
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not m:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The value is negative: NCE_NEGATIVE_M_CODE_USED
   3. The value is greater than 199: NCE_M_CODE_GREATER_THAN_199
   4. The m code is not known to the system: NCE_UNKNOWN_M_CODE_USED
   5. Another m code in the same modal group has already been read:
      NCE_TWO_M_CODES_USED_FROM_SAME_MODAL_GROUP

Side effects:
   counter is reset to the character following the m number.
   An m code is inserted as the value of the appropriate mode in the
   m_modes array in the block.
   The m code counter in the block is increased by 1.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'm', indicating an m code.
The function reads characters which give the (integer) value of the
m code.

read_integer_value allows a minus sign, so a check for a negative value
is needed here, and the parameters argument is also needed.

*/

int Interp::read_m(char *line,   //!< string: line of RS274 code being processed   
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters                   
{
  int value;
  int mode;

  CHKS((line[*counter] != 'm'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHP(read_integer_value(line, counter, &value, parameters));
  CHKS((value < 0), NCE_NEGATIVE_M_CODE_USED);

  remap_pointer r = _setup.m_remapped[value];
  if (r) {
      mode =  r->modal_group;
      CHKS ((mode < 0),"BUG: M remapping: modal group < 0");
      CHKS ((mode > 10),"BUG: M remapping: modal group > 10");

      CHKS((block->m_modes[mode] != -1),
	   NCE_TWO_M_CODES_USED_FROM_SAME_MODAL_GROUP);
      block->m_modes[mode] = value;
      block->m_count++;
      return INTERP_OK;
  }

  CHKS((value > 199), NCE_M_CODE_GREATER_THAN_199,value);
  mode = ems[value];
  CHKS((mode == -1), NCE_UNKNOWN_M_CODE_USED,value);
  CHKS((block->m_modes[mode] != -1),
      NCE_TWO_M_CODES_USED_FROM_SAME_MODAL_GROUP);
  block->m_modes[mode] = value;
  block->m_count++;
  if (value >= 100 && value < 200) {
    block->user_m = 1;
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_one_item

Returned Value: int
   If a reader function which is called returns an error code, that
   error code is returned.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. the first character read is not a known character for starting a
   word: NCE_BAD_CHARACTER_USED

Side effects:
   This function reads one item from a line of RS274/NGC code and inserts
   the information in a block. System parameters may be reset.

Called by: read_items.

When this function is called, the counter is set so that the position
being considered is the first position of a word. The character at
that position must be one known to the system.  In this version those
characters are: a,b,c,d,f,g,h,i,j,k,l,m,n,p,q,r,s,t,x,y,z,(,#,;.
However, read_items calls read_n_number directly if the first word
begins with n, so no read function is included in the "_readers" array
for the letter n. Thus, if an n word is encountered in the middle of
a line, this function reports NCE_BAD_CHARACTER_USED.

The function looks for a letter or special character and matches it to
a pointer to a function in _readers[] - The position of the function
pointers in the array match their ASCII number.
Once the character has been matched, the function calls a selected function
according to what the letter or character is.  The selected function will
be responsible to consider all the characters that comprise the remainder
of the item, and reset the pointer so that it points to the next character
after the end of the item (which may be the end of the line or the first
character of another item).

After an item is read, the counter is set at the index of the
next unread character. The item data is stored in the block.

It is expected that the format of a comment will have been checked;
this is being done by close_and_downcase. Bad format comments will
have prevented the system from getting this far, so that this function
can assume a close parenthesis will be found when an open parenthesis
has been found, and that comments are not nested.

*/

int Interp::read_one_item(
    char *line,    //!< string: line of RS274/NGC code being processed
    int *counter,  //!< pointer to a counter for position on the line 
    block_pointer block,   //!< pointer to a block being filled from the line 
    double * parameters) /* array of system parameters  */
{
  read_function_pointer function_pointer;
  char letter;

  letter = line[*counter];      /* check if in array range */
  CHKS(((letter < ' ') || (letter > 'z')),
	_("Bad character '\\%03o' used"), (unsigned char)letter);
  function_pointer = _readers[(int) letter]; /* Find the function pointer in the array */
  CHKS((function_pointer == 0),
	(!isprint(letter) || isspace(letter)) ?
	    _("Bad character '\\%03o' used") : _("Bad character '%c' used"), letter);
  CHP((*this.*function_pointer)(line, counter, block, parameters)); /* Call the function */ 
  return INTERP_OK;
}

/****************************************************************************/

/*! read_operation

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The operation is unknown:
      NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_A
      NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_M
      NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_O
      NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_X
      NCE_UNKNOWN_OPERATION
   2. The line ends without closing the expression: NCE_UNCLOSED_EXPRESSION

Side effects:
   An integer representing the operation is put into what operation points
   at.  The counter is reset to point to the first character after the
   operation.

Called by: read_real_expression

This expects to be reading a binary operation (+, -, /, *, **, and,
mod, or, xor) or a right bracket (]). If one of these is found, the
value of operation is set to the symbolic value for that operation.
If not, an error is reported as described above.

*/

int Interp::read_operation(char *line,   //!< string: line of RS274/NGC code being processed
                          int *counter, //!< pointer to a counter for position on the line 
                          int *operation)       //!< pointer to operation to be read               
{
  char c;

  c = line[*counter];
  *counter = (*counter + 1);
  switch (c) {
  case '+':
    *operation = PLUS;
    break;
  case '-':
    *operation = MINUS;
    break;
  case '/':
    *operation = DIVIDED_BY;
    break;
  case '*':
    if (line[*counter] == '*') {
      *operation = POWER;
      *counter = (*counter + 1);
    } else
      *operation = TIMES;
    break;
  case ']':
    *operation = RIGHT_BRACKET;
    break;
  case 'a':
    if ((line[*counter] == 'n') && (line[(*counter) + 1] == 'd')) {
      *operation = AND2;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_A);
    break;
  case 'm':
    if ((line[*counter] == 'o') && (line[(*counter) + 1] == 'd')) {
      *operation = MODULO;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_M);
    break;
  case 'o':
    if (line[*counter] == 'r') {
      *operation = NON_EXCLUSIVE_OR;
      *counter = (*counter + 1);
    } else
      ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_O);
    break;
  case 'x':
    if ((line[*counter] == 'o') && (line[(*counter) + 1] == 'r')) {
      *operation = EXCLUSIVE_OR;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_X);
    break;

      /* relational operators */
    case 'e':
      if(line[*counter] == 'q')
	{
	  *operation = EQ;
	  *counter = (*counter + 1);
	}
      else
        ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_E);
      break;
    case 'n':
      if(line[*counter] == 'e')
	{
	  *operation = NE;
	  *counter = (*counter + 1);
	}
      else
        ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_N);
      break;
    case 'g':
      if(line[*counter] == 'e')
	{
	  *operation = GE;
	  *counter = (*counter + 1);
	}
      else if(line[*counter] == 't')
	{
	  *operation = GT;
	  *counter = (*counter + 1);
	}
      else
        ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_G);
      break;
    case 'l':
      if(line[*counter] == 'e')
	{
	  *operation = LE;
	  *counter = (*counter + 1);
	}
      else if(line[*counter] == 't')
	{
	  *operation = LT;
	  *counter = (*counter + 1);
	}
      else
        ERS(NCE_UNKNOWN_OPERATION_NAME_STARTING_WITH_L);
      break;

  case 0:
    ERS(NCE_UNCLOSED_EXPRESSION);
  default:
    ERS(NCE_UNKNOWN_OPERATION);
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_operation_unary

Returned Value: int
   If the operation is not a known unary operation, this returns one of
   the following error codes:
   NCE_UNKNOWN_WORD_STARTING_WITH_A
   NCE_UNKNOWN_WORD_STARTING_WITH_C
   NCE_UNKNOWN_WORD_STARTING_WITH_E
   NCE_UNKNOWN_WORD_STARTING_WITH_F
   NCE_UNKNOWN_WORD_STARTING_WITH_L
   NCE_UNKNOWN_WORD_STARTING_WITH_R
   NCE_UNKNOWN_WORD_STARTING_WITH_S
   NCE_UNKNOWN_WORD_STARTING_WITH_T
   NCE_UNKNOWN_WORD_WHERE_UNARY_OPERATION_COULD_BE
   Otherwise, this returns INTERP_OK.

Side effects:
   An integer code for the name of the operation read from the
   line is put into what operation points at.
   The counter is reset to point to the first character after the
   characters which make up the operation name.

Called by:
   read_unary

This attempts to read the name of a unary operation out of the line,
starting at the index given by the counter. Known operations are:
abs, acos, asin, atan, cos, exp, fix, fup, ln, round, sin, sqrt, tan.

*/

int Interp::read_operation_unary(char *line,     //!< string: line of RS274/NGC code being processed
                                int *counter,   //!< pointer to a counter for position on the line 
                                int *operation) //!< pointer to operation to be read               
{
  char c;

  c = line[*counter];
  *counter = (*counter + 1);
  switch (c) {
  case 'a':
    if ((line[*counter] == 'b') && (line[(*counter) + 1] == 's')) {
      *operation = ABS;
      *counter = (*counter + 2);
    } else if (strncmp((line + *counter), "cos", 3) == 0) {
      *operation = ACOS;
      *counter = (*counter + 3);
    } else if (strncmp((line + *counter), "sin", 3) == 0) {
      *operation = ASIN;
      *counter = (*counter + 3);
    } else if (strncmp((line + *counter), "tan", 3) == 0) {
      *operation = ATAN;
      *counter = (*counter + 3);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_A);
    break;
  case 'c':
    if ((line[*counter] == 'o') && (line[(*counter) + 1] == 's')) {
      *operation = COS;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_C);
    break;
  case 'e':
    if ((line[*counter] == 'x') && (line[(*counter) + 1] == 'p')) {
      *operation = EXP;
      *counter = (*counter + 2);
    } else if (    (line[*counter]     == 'x')
                && (line[*counter + 1] == 'i')
                && (line[*counter + 2] == 's')
                && (line[*counter + 3] == 't')
                && (line[*counter + 4] == 's')
             ) {
      *counter = (*counter + 5);
      *operation = EXISTS;
    } else {
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_E);
    }
    break;
  case 'f':
    if ((line[*counter] == 'i') && (line[(*counter) + 1] == 'x')) {
      *operation = FIX;
      *counter = (*counter + 2);
    } else if ((line[*counter] == 'u') && (line[(*counter) + 1] == 'p')) {
      *operation = FUP;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_F);
    break;
  case 'l':
    if (line[*counter] == 'n') {
      *operation = LN;
      *counter = (*counter + 1);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_L);
    break;
  case 'r':
    if (strncmp((line + *counter), "ound", 4) == 0) {
      *operation = ROUND;
      *counter = (*counter + 4);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_R);
    break;
  case 's':
    if ((line[*counter] == 'i') && (line[(*counter) + 1] == 'n')) {
      *operation = SIN;
      *counter = (*counter + 2);
    } else if (strncmp((line + *counter), "qrt", 3) == 0) {
      *operation = SQRT;
      *counter = (*counter + 3);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_S);
    break;
  case 't':
    if ((line[*counter] == 'a') && (line[(*counter) + 1] == 'n')) {
      *operation = TAN;
      *counter = (*counter + 2);
    } else
      ERS(NCE_UNKNOWN_WORD_STARTING_WITH_T);
    break;
  default:
    ERS(NCE_UNKNOWN_WORD_WHERE_UNARY_OPERATION_COULD_BE);
  }
  return INTERP_OK;
}

/****************************************************************************/

/* read_o

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns RS274NGC_OK.
   1. The first character read is not o:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   counter is reset to point to the first character following the p value.
   The p value setting is inserted in block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'o', indicating a o value
setting. The function reads characters which tell how to set the o
value, up to the start of the next item or the end of the line. This
information is inserted in the block.

O codes are used for:
1.  sub
2.  endsub
3.  call
4.  do
5.  while
6.  if
7.  elseif
8.  else
9.  endif
10. break
11. continue
12. endwhile
13. return

Labels (o-words) are of local scope except for sub, endsub, and call which
are of global scope.

Named o-words and scoping of o-words are implemented now.
Numeric o-words get converted to strings. Global o-words are stored as
just the string. Local o-words are stored as the name (o-word) of the
sub containing the o-word, followed by a '#', followed by the local o-word.
If the local o-word is not contained within a sub, then the sub name is null
and the o-word just begins with a '#'.

It has been a pain to do this because o-words are used as integers all over
the place.
!!!KL the code could use some cleanup.

*/

int Interp::read_o(    /* ARGUMENTS                                     */
 char * line,         /* string: line of RS274/NGC code being processed */
 int * counter,       /* pointer to a counter for position on the line  */
 block_pointer block, /* pointer to a block being filled from the line  */
 double * parameters) /* array of system parameters                     */
{
  static char name[] = "read_o";
  double value;
  int param_cnt;
  char oNameBuf[LINELEN+1];
  const char *subName;
  char fullNameBuf[2*LINELEN+1];
  int oNumber, n;
  extern const char *o_ops[];

  if (line[*counter] == 'm' &&
      read_integer_value(line, &(n=*counter+1), &oNumber,
			 parameters) == INTERP_OK) {
      // m98 or m99 found
      if (oNumber == 98) {
	  CHKS(_setup.disable_fanuc_style_sub,
	       "DISABLE_FANUC_STYLE_SUB set in INI file, but found m98");

	  // Fanuc-style subroutine call with loop: "m98"
	  block->o_type = M_98;
	  *counter += 3;

	  // Read P-word and L-word now
	  n = strlen(line);
	  while (*counter < n)
	      CHP(read_one_item(line, counter, block, parameters));
	  // P-word:  convert to int and put in oNameBuf
	  CHKS(! block->p_flag, "Found 'm98' code with no P-word");
	  // (conversion code from read_integer_value)
	  n = (int) floor(block->p_number);
	  if ((block->p_number - n) > 0.9999) {
	      n = (int) ceil(block->p_number);
	  } else
	      CHKS((block->p_number - n) > 0.0001,
		   NCE_NON_INTEGER_VALUE_FOR_INTEGER);
	  snprintf(oNameBuf, sizeof(oNameBuf), "%d", n);
      } else if (oNumber == 99) {
	  // Fanuc-style subroutine return: "m99"

	  // Error checks:
	  // - Fanuc-style subs disabled
	  CHKS(_setup.disable_fanuc_style_sub,
	       "DISABLE_FANUC_STYLE_SUB set in INI file, but found m99");
	  // - Empty stack M99 (endless program) handled in read_m()
	  CHKS(_setup.defining_sub,
	       "Found 'M99' instead of 'O endsub' after 'O sub'");

	  // Fanuc-style subroutine return: "m99"
	  block->o_type = M_99;
	  *counter += 3;

	  // Subroutine name not provided in Fanuc syntax, so pull from
	  // context
          if (strlen(_setup.sub_context[_setup.call_level].subName) >= sizeof(oNameBuf))
              ERS(NCE_UNABLE_TO_OPEN_FILE, _setup.sub_context[_setup.call_level].subName);
	  rtapi_strlcpy(oNameBuf, _setup.sub_context[_setup.call_level].subName,
                  sizeof(oNameBuf));
      } else
	  // any other m-code should have been handled by read_m()
	  OERR(_("%d: Bug:  Non-m98/m99 M-code passed to read_o(): '%s'"),
	       _setup.sequence_number, _setup.linetext);

  } else {
      CHKS((line[*counter] != 'o'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);

      // rs274-style O-word

      *counter += 1;
      if(line[*counter] == '<')
	  {
	      read_name(line, counter, oNameBuf);
	  }
      else
	  {
	      CHP(read_integer_value(line, counter, &oNumber,
				     parameters));
	      snprintf(oNameBuf, sizeof(oNameBuf), "%d", oNumber);
	  }

      // We stash the text the offset part of setup

#define CMP(txt) (strncmp(line+*counter, txt, strlen(txt)) == 0 && (*counter += strlen(txt)))
      // characterize the type of o-word

      if(CMP("sub"))
	  block->o_type = O_sub;
      else if(CMP("endsub"))
	  block->o_type = O_endsub;
      else if(CMP("call"))
	  block->o_type = O_call;
      else if(CMP("do"))
	  block->o_type = O_do;
      else if(CMP("while"))
	  block->o_type = O_while;
      else if(CMP("repeat"))
	  block->o_type = O_repeat;
      else if(CMP("if"))
	  block->o_type = O_if;
      else if(CMP("elseif"))
	  block->o_type = O_elseif;
      else if(CMP("else"))
	  block->o_type = O_else;
      else if(CMP("endif"))
	  block->o_type = O_endif;
      else if(CMP("break"))
	  block->o_type = O_break;
      else if(CMP("continue"))
	  block->o_type = O_continue;
      else if(CMP("endwhile"))
	  block->o_type = O_endwhile;
      else if(CMP("endrepeat"))
	  block->o_type = O_endrepeat;
      else if(CMP("return"))
	  block->o_type = O_return;
      else if((line+*counter)[0] == '(' || (line+*counter)[0] == ';'
	      || (line+*counter)[0] == 0) {
	  // Fanuc-style subroutine definition:  "O2000" with no following args
	  CHKS(_setup.disable_fanuc_style_sub,
	       "DISABLE_FANUC_STYLE_SUB disabled in INI file, but found "
	       "bare O-word");

	  block->o_type = O_;
      } else
	  block->o_type = O_none;
  }

  logDebug("In: %s line:%d |%s| subroutine=|%s|",
	   name, block->line_number, line, oNameBuf);

  // we now have it characterized
  // now create the text of the oword

  switch(block->o_type)
    {
      // the global cases first
    case O_sub:
    case O_:
    case O_endsub:
    case O_call:
    case M_98:
    case O_return:
    case M_99:
	block->o_name = strstore(oNameBuf);
	logDebug("global case:|%s|", block->o_name);
	break;

      // the remainder are local cases
    default:
      if(_setup.call_level)
	{
	  subName = _setup.sub_context[_setup.call_level].subName;
	  logDebug("inside a call[%d]:|%s|", _setup.call_level, subName);
	}
      else if(_setup.defining_sub)
	{
	  subName = _setup.sub_name;
	  logDebug("defining_sub:|%s|", subName);
	}
      else
	{
	  subName = "";
	  logDebug("not defining_sub:|%s|", subName);
	}
      snprintf(fullNameBuf, sizeof(fullNameBuf), "%s#%s", subName, oNameBuf);
      block->o_name = strstore(fullNameBuf);
      logDebug("local case:|%s|", block->o_name);
    }
  logDebug("o_type:%s o_name: %s  line:%d %s", o_ops[block->o_type], block->o_name,
	   block->line_number, line);

  if (block->o_type == O_sub || block->o_type == O_)
    {
	// Check we're not already defining a main- or sub-program
	CHKS((_setup.defining_sub == 1), NCE_NESTED_SUBROUTINE_DEFN);
    }
  // in terms of execution endsub and return do the same thing
  else if ((block->o_type == O_endsub) || (block->o_type == O_return) ||
	   (block->o_type == M_99))
    {
	if ((_setup.skipping_o != 0) &&
	    (0 != strcmp(_setup.skipping_o, block->o_name))) {
	    return INTERP_OK;
	}

	// optional return value expression
	if (block->o_type != M_99 && line[*counter] == '[') {
	    CHP(read_real_expression(line, counter, &value, parameters));
	    logOword("%s %s value %lf",
		     (block->o_type == O_endsub) ? "endsub" : "return",
		     block->o_name,
		     value);
	    _setup.return_value = value;
	    _setup.value_returned = 1;
	} else {
	    _setup.return_value = 0;
	    _setup.value_returned = 0;
	}
    }
  else if(_setup.defining_sub == 1)
    {
      // we can not evaluate expressions -- so just skip on out
      block->o_type = O_none;
    }
  else if(block->o_type == O_call)
    {
      // we need to NOT evaluate parameters if skipping
      // skipping never ends on a "call"
      if(_setup.skipping_o != 0)
      {
          block->o_type = O_none;
          return INTERP_OK;
      }

      // convey starting state for call_fsm() to handle this call
      // convert_remapped_code() might change this to CS_REMAP 
      block->call_type = is_pycallable(&_setup,  OWORD_MODULE, block->o_name) ?
	  CT_PYTHON_OWORD_SUB : CT_NGC_OWORD_SUB;

      for(param_cnt=0;(line[*counter] == '[') || (line[*counter] == '(');)
	{
	  if(line[*counter] == '(')
	    {
	      CHP(read_comment(line, counter, block, parameters));
	      continue;
	    }
	  logDebug("counter[%d] rest of line:|%s|", *counter,
		   line+*counter);
	  CHKS((param_cnt >= INTERP_SUB_PARAMS),
	      NCE_TOO_MANY_SUBROUTINE_PARAMETERS);
	  CHP(read_real_expression(line, counter, &value, parameters));
	  block->params[param_cnt] = value;
	  param_cnt++;
	}
      logDebug("set arg params:%d", param_cnt);
      block->param_cnt = param_cnt;

      // zero the remaining params
      for(;param_cnt < INTERP_SUB_PARAMS; param_cnt++)
	{
	  block->params[param_cnt] = 0.0;
	}
    }
  else if(block->o_type == M_98) {
      // No params in M98 block (this could also be 30!)
      block->param_cnt = 0;
      // Distinguish from 'O.... call'
      block->call_type = CT_NGC_M98_SUB;
  }
  else if(block->o_type == O_do)
    {
      block->o_type = O_do;
    }
  else if(block->o_type == O_while)
    {
      // TESTME !!!KL -- should not eval expressions if skipping ???
      if((_setup.skipping_o != 0) &&
	 (0 != strcmp(_setup.skipping_o, block->o_name)))
      {
	    return INTERP_OK;
      }

      block->o_type = O_while;
      CHKS((line[*counter] != '['),
	    _("Left bracket missing after 'while'"));
      CHP(read_real_expression(line, counter, &value, parameters));
      _setup.test_value = value;
    }
  else if(block->o_type == O_repeat)
      {
          // TESTME !!!KL -- should not eval expressions if skipping ???
          if((_setup.skipping_o != 0) &&
	     (0 != strcmp(_setup.skipping_o, block->o_name)))
          {
	    return INTERP_OK;
          }

          block->o_type = O_repeat;
          CHKS((line[*counter] != '['),
               _("Left bracket missing after 'repeat'"));
          CHP(read_real_expression(line, counter, &value, parameters));
          _setup.test_value = value;
      }
  else if(block->o_type == O_if)
    {
      // TESTME !!!KL -- should not eval expressions if skipping ???
      if((_setup.skipping_o != 0) &&
	 (0 != strcmp(_setup.skipping_o, block->o_name)))
      {
	    return INTERP_OK;
      }

      block->o_type = O_if;
      CHKS((line[*counter] != '['),
	    _("Left bracket missing after 'if'"));
      CHP(read_real_expression(line, counter, &value, parameters));
      _setup.test_value = value;
    }
  else if(block->o_type == O_elseif)
    {
      // TESTME !!!KL -- should not eval expressions if skipping ???
      if((_setup.skipping_o != 0) &&
	 (0 != strcmp(_setup.skipping_o, block->o_name)))
      {
	    return INTERP_OK;
      }

      block->o_type = O_elseif;
      CHKS((line[*counter] != '['),
	    _("Left bracket missing after 'elseif'"));
      CHP(read_real_expression(line, counter, &value, parameters));
      _setup.test_value = value;
    }
  else if(block->o_type == O_else)
    {
      block->o_type = O_else;
    }
  else if(block->o_type == O_endif)
    {
      block->o_type = O_endif;
    }
  else if(block->o_type == O_break)
    {
      block->o_type = O_break;
    }
  else if(block->o_type == O_continue)
    {
      block->o_type = O_continue;
    }
  else if(block->o_type == O_endwhile)
    {
      block->o_type = O_endwhile;
    }
  else if(block->o_type == O_endrepeat)
      {
          block->o_type = O_endrepeat;
      }
  else
    {
      // not legal
      block->o_type = O_none;
      ERS(NCE_UNKNOWN_COMMAND_IN_O_LINE);
    }

  return INTERP_OK;
}


/****************************************************************************/

/*! read_p

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not p:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A p value has already been inserted in the block:
      NCE_MULTIPLE_P_WORDS_ON_ONE_LINE

Side effects:
   counter is reset to point to the first character following the p value.
   The p value setting is inserted in block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'p', indicating a p value
setting. The function reads characters which tell how to set the p
value, up to the start of the next item or the end of the line. This
information is inserted in the block.

P codes are used for:
1. Dwell time in canned cycles g82, G86, G88, G89 [NCMS pages 98 - 100].
2. A key with G10 [NCMS, pages 9, 10].

*/

int Interp::read_p(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'p'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->p_number > -1.0), NCE_MULTIPLE_P_WORDS_ON_ONE_LINE);

  CHP(read_real_value(line, counter, &value, parameters));
  // FMP removed check for negatives, since we may want them for
  // user-defined codes
  // CHKS((value < 0.0), NCE_NEGATIVE_P_WORD_USED);
  block->p_number = value;
  block->p_flag = true;
  return INTERP_OK;
}

int Interp::read_name(
    char *line,   //!< string: line of RS274/NGC code being processed
    int *counter, //!< pointer to a counter for position on the line 
    char *nameBuf)   //!< pointer to name to be read
{

  int done = 0;
  int i;

  CHKS((line[*counter] != '<'),
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);

  // skip over the '<'
  *counter = (*counter + 1);

  for(i=0; (i<LINELEN) && (line[*counter]); i++)
  {
      if(line[*counter] == '>')
      {
          nameBuf[i] = 0; // terminate the name
          *counter = (*counter + 1);
          done = 1;
          break;
      }
      nameBuf[i] = line[*counter];
      *counter = (*counter + 1);
  }

  // !!!KL need to rename the error message and change text
  CHKS((!done), NCE_NAMED_PARAMETER_NOT_TERMINATED);

  return INTERP_OK;
}




/****************************************************************************/

/*! read_parameter

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, this returns INTERP_OK.
   1. The first character read is not # :
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The parameter number is out of bounds:
      NCE_PARAMETER_NUMBER_OUT_OF_RANGE

Side effects:
   The value of the given parameter is put into what double_ptr points at.
   The counter is reset to point to the first character after the
   characters which make up the value.

Called by:  read_real_value

This attempts to read the value of a parameter out of the line,
starting at the index given by the counter.

According to the RS274/NGC manual [NCMS, p. 62], the characters following
# may be any "parameter expression". Thus, the following are legal
and mean the same thing (the value of the parameter whose number is
stored in parameter 2):
  ##2
  #[#2]

Parameter setting is done in parallel, not sequentially. For example
if #1 is 5 before the line "#1=10 #2=#1" is read, then after the line
is executed, #1 is 10 and #2 is 5. If parameter setting were done
sequentially, the value of #2 would be 10 after the line was executed.

ADDED by K. Lerman
Named parameters are now supported.
#[abcd] is a parameter with name "abcd"
#[#2] is NOT a named parameter.
When a [ is seen after a #, if the next char is not a #, it is a named
parameter.

*/

int Interp::read_parameter(
    char *line,   //!< string: line of RS274/NGC code being processed
    int *counter, //!< pointer to a counter for position on the line 
    double *double_ptr,   //!< pointer to double to be read                  
    double *parameters,   //!< array of system parameters
    bool check_exists)    //!< test for existence, not value
{
  int index;

  CHKS((line[*counter] != '#'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);

  *counter = (*counter + 1);

  // named parameters look like '<letter...>' or '<_.....>'
  if(line[*counter] == '<')
  {
      //_setup.stack_index = 0; -- reminder of variable we need
      // find the matching string (if any) -- or create it
      // then get the value and store it
      CHP(read_named_parameter(line, counter, double_ptr, parameters,
	      check_exists));
  }
  else
  {
      CHP(read_integer_value(line, counter, &index, parameters));
      if(check_exists)
      {
      *double_ptr = index >= 1 && index < RS274NGC_MAX_PARAMETERS;
	  return INTERP_OK;
      }
      CHKS(((index < 1) || (index >= RS274NGC_MAX_PARAMETERS)),
          NCE_PARAMETER_NUMBER_OUT_OF_RANGE);
      CHKS(((index >= 5420) && (index <= 5428) && (_setup.cutter_comp_side != CUTTER_COMP::OFF)),
           _("Cannot read current position with cutter radius compensation on"));
      *double_ptr = parameters[index];
  }
  return INTERP_OK;
}

int Interp::read_bracketed_parameter(
    char *line,   //!< string: line of RS274/NGC code being processed
    int *counter, //!< pointer to a counter for position on the line
    double *double_ptr,   //!< pointer to double to be read
    double *parameters,   //!< array of system parameters
    bool check_exists)    //!< test for existence, not value
{
  CHKS((line[*counter] != '['), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((line[*counter] != '#'), _("Expected # reading parameter"));
  CHP(read_parameter(line, counter, double_ptr, parameters, check_exists));
  CHKS((line[*counter] != ']'), _("Expected ] reading bracketed parameter"));
  *counter = (*counter + 1);
  return INTERP_OK;
}


/****************************************************************************/

/*! read_parameter_setting

Returned Value: int
   If read_real_value or read_integer_value returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not # :
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The parameter index is out of range: PARAMETER_NUMBER_OUT_OF_RANGE
   3. An equal sign does not follow the parameter expression:
      NCE_EQUAL_SIGN_MISSING_IN_PARAMETER_SETTING

Side effects:
   counter is reset to the character following the end of the parameter
   setting. The parameter whose index follows "#" is set to the
   real value following "=".

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character '#', indicating a parameter
setting when found by read_one_item.  The function reads characters
which tell how to set the parameter.

Any number of parameters may be set on a line. If parameters set early
on the line are used in expressions farther down the line, the
parameters have their old values, not their new values. This is
usually called setting parameters in parallel.

Parameter setting is not clearly described in [NCMS, pp. 51 - 62]: it is
not clear if more than one parameter setting per line is allowed (any
number is OK in this implementation). The characters immediately following
the "#" must constitute a "parameter expression", but it is not clear
what that is. Here we allow any expression as long as it evaluates to
an integer.

Parameters are handled in the interpreter by having a parameter table
and a parameter buffer as part of the machine settings. The parameter
table is passed to the reading functions which need it. The parameter
buffer is used directly by functions that need it. Reading functions
may set parameter values in the parameter buffer. Reading functions
may obtain parameter values; these come from parameter table.

The parameter buffer has three parts: (i) a counter for how many
parameters have been set while reading the current line (ii) an array
of the indexes of parameters that have been set while reading the
current line, and (iii) an array of the values for the parameters that
have been set while reading the current line; the nth value
corresponds to the nth index. Any given index will appear once in the
index number array for each time the parameter with that index is set
on a line. There is no point in setting the same parameter more than
one on a line because only the last setting of that parameter will
take effect.

The syntax recognized by this this function is # followed by an
integer expression (explicit integer or expression evaluating to an
integer) followed by = followed by a real value (number or
expression).

Note that # also starts a bunch of characters which represent a parameter
to be evaluated. That situation is handled by read_parameter.

*/

int Interp::read_parameter_setting(
    char *line,   //!< string: line of RS274/NGC code being processed
    int *counter, //!< pointer to a counter for position on the line 
    block_pointer /*block*/,  //!< pointer to a block being filled from the line
    double *parameters)   //!< array of system parameters
{
  static char name[] = "read_parameter_setting";
  int index;
  double value;
  char *param;
  const char *dup;

  CHKS((line[*counter] != '#'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);

  // named parameters look like '<letter...>' or '<_letter.....>'
  if(line[*counter] == '<')
  {
      CHP(read_named_parameter_setting(line, counter, &param, parameters));

      CHKS((line[*counter] != '='),
          NCE_EQUAL_SIGN_MISSING_IN_PARAMETER_SETTING);
      *counter = (*counter + 1);
      CHP(read_real_value(line, counter, &value, parameters));

      logDebug("setting up named param[%d]:|%s| value:%lf",
               _setup.named_parameter_occurrence, param, value);

      dup = strstore(param); // no more need to free this
      if(dup == 0)
      {
          ERS(NCE_OUT_OF_MEMORY);
      }
      logDebug("%s |%s|", name,  dup);
      _setup.named_parameters[_setup.named_parameter_occurrence] = dup;

      _setup.named_parameter_values[_setup.named_parameter_occurrence] = value;
      _setup.named_parameter_occurrence++;
      logDebug("done setting up named param[%d]:|%s| value:%lf",
               _setup.named_parameter_occurrence, param, value);
  }
  else
  {
      CHP(read_integer_value(line, counter, &index, parameters));
      CHKS(((index < 1) || (index >= RS274NGC_MAX_PARAMETERS)),
          NCE_PARAMETER_NUMBER_OUT_OF_RANGE);
      CHKS((is_parameter_readonly(index)), NCE_PARAMETER_NUMBER_READONLY);
      CHKS((line[*counter] != '='),
          NCE_EQUAL_SIGN_MISSING_IN_PARAMETER_SETTING);
      *counter = (*counter + 1);
      CHP(read_real_value(line, counter, &value, parameters));
      _setup.parameter_numbers[_setup.parameter_occurrence] = index;
      _setup.parameter_values[_setup.parameter_occurrence] = value;
      _setup.parameter_occurrence++;
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! read_named_parameter_setting

Returned Value: int
   If read_real_value or read_integer_value returns an error code,
   this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not # :
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The parameter index is out of range: PARAMETER_NUMBER_OUT_OF_RANGE
   3. An equal sign does not follow the parameter expression:
      NCE_EQUAL_SIGN_MISSING_IN_PARAMETER_SETTING

Side effects:
   counter is reset to the character following the end of the parameter
   setting. The parameter whose index follows "#" is set to the
   real value following "=".

Called by: read_parameter_setting

When this function is called, counter is pointing at an item on the
line that starts with the character '#', indicating a parameter
setting when found by read_one_item.  The function reads characters
which tell how to set the parameter.

Any number of parameters may be set on a line. If parameters set early
on the line are used in expressions farther down the line, the
parameters have their old values, not their new values. This is
usually called setting parameters in parallel.

Parameter setting is not clearly described in [NCMS, pp. 51 - 62]: it is
not clear if more than one parameter setting per line is allowed (any
number is OK in this implementation). The characters immediately following
the "#" must constitute a "parameter expression", but it is not clear
what that is. Here we allow any expression as long as it evaluates to
an integer.

Parameters are handled in the interpreter by having a parameter table
and a parameter buffer as part of the machine settings. The parameter
table is passed to the reading functions which need it. The parameter
buffer is used directly by functions that need it. Reading functions
may set parameter values in the parameter buffer. Reading functions
may obtain parameter values; these come from parameter table.

The parameter buffer has three parts: (i) a counter for how many
parameters have been set while reading the current line (ii) an array
of the indexes of parameters that have been set while reading the
current line, and (iii) an array of the values for the parameters that
have been set while reading the current line; the nth value
corresponds to the nth index. Any given index will appear once in the
index number array for each time the parameter with that index is set
on a line. There is no point in setting the same parameter more than
one on a line because only the last setting of that parameter will
take effect.

The syntax recognized by this this function is # followed by an
integer expression (explicit integer or expression evaluating to an
integer) followed by = followed by a real value (number or
expression).

Note that # also starts a bunch of characters which represent a parameter
to be evaluated. That situation is handled by read_parameter.

*/

int Interp::read_named_parameter_setting(
    char *line,   //!< string: line of RS274/NGC code being processed
    int *counter, //!< pointer to a counter for position on the line 
    char **param,  //!< pointer to the char * to be returned 
    double * /*parameters*/)   //!< array of system parameters
{
  static char name[] = "read_named_parameter_setting";
  int status;
  static char paramNameBuf[LINELEN+1];

  *param = paramNameBuf;

  logDebug("entered %s", name);
  CHKS((line[*counter] != '<'),
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);

  status=read_name(line, counter, paramNameBuf);
  CHP(status);

  logDebug("%s: returned(%d) from read_name:|%s|", name, status, paramNameBuf);

  status = add_named_param(paramNameBuf);
  CHP(status);
  logDebug("%s: returned(%d) from add_named_param:|%s|", name, status, paramNameBuf);

  // the rest of the work is done in read_parameter_setting

  return INTERP_OK;
}

/****************************************************************************/

/*! read_q

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not q:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A q value has already been inserted in the block:
      NCE_MULTIPLE_Q_WORDS_ON_ONE_LINE

Side effects:
   counter is reset to point to the first character following the q value.
   The q value setting is inserted in block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'q', indicating a q value
setting. The function reads characters which tell how to set the q
value, up to the start of the next item or the end of the line. This
information is inserted in the block.

Q is used only in the G87 canned cycle [NCMS, page 98], where it must
be positive.

*/

int Interp::read_q(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'q'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->q_number > -1.0), NCE_MULTIPLE_Q_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  // FMP removed check for negatives, since we may want them for
  // user-defined codes
  // CHKS((value <= 0.0), NCE_NEGATIVE_OR_ZERO_Q_VALUE_USED);
  block->q_number = value;
  block->q_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_r

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not r:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. An r_number has already been inserted in the block:
      NCE_MULTIPLE_R_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The r_flag in the block is turned on.
   The r_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'r'. The function reads characters
which tell how to set the coordinate, up to the start of the next item
or the end of the line. This information is inserted in the block. The
counter is then set to point to the character following.

An r number indicates the clearance plane in canned cycles.
An r number may also be the radius of an arc.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_r(char *line,   //!< string: line of RS274 code being processed   
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters                   
{
  double value;

  CHKS((line[*counter] != 'r'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->r_flag), NCE_MULTIPLE_R_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->r_flag = true;
  block->r_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_real_expression

Returned Value: int
   If any of the following functions returns an error code,
   this returns that code.
     read_real_value
     read_operation
     execute_binary
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The first character is not [ :
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED

Side effects:
   The number read from the line is put into what value_ptr points at.
   The counter is reset to point to the first character after the real
   expression.

Called by:
 read_atan
 read_real_value
 read_unary

Example 1: [2 - 3 * 4 / 5] means [2 - [[3 * 4] / 5]] and equals -0.4.

Segmenting Expressions -

The RS274/NGC manual, section 3.5.1.1 [NCMS, page 50], provides for
using square brackets to segment expressions.

Binary Operations -

The RS274/NGC manual, section 3.5.1.1, discusses expression evaluation.
The manual provides for eight binary operations: the four basic
mathematical operations (addition, subtraction, multiplication,
division), three logical operations (non-exclusive ||, exclusive ||,
and AND2) and the modulus operation. The manual does not explicitly call
these "binary" operations, but implicitly recognizes that they are
binary. We have added the "power" operation of raising the number
on the left of the operation to the power on the right; this is
needed for many basic machining calculations.

There are two groups of binary operations given in the manual. If
operations are strung together as shown in Example 1, operations in
the first group are to be performed before operations in the second
group. If an expression contains more than one operation from the same
group (such as * and / in Example 1), the operation on the left is
performed first. The first group is: multiplication (*), division (/),
and modulus (MOD). The second group is: addition(+), subtraction (-),
logical non-exclusive || (||), logical exclusive || (XOR), and logical
&& (AND2). We have added a third group with higher precedence than
the first group. The third group contains only the power (**)
operation.

The logical operations and modulus are apparently to be performed on
any real numbers, not just on integers or on some other data type.

Unary Operations -

The RS274/NGC manual, section 3.5.1.2, provides for fifteen unary
mathematical operations. Two of these, BIN and BCD, are apparently for
converting between decimal and hexadecimal number representation,
although the text is not clear. These have not been implemented, since
we are not using any hexadecimal numbers. The other thirteen unary
operations have been implemented: absolute_value, arc_cosine, arc_sine,
arc_tangent, cosine, e_raised_to, fix_down, fix_up, natural_log_of,
round, sine, square_root, tangent.

The manual section 3.5.1.2 [NCMS, page 51] requires the argument to
all unary operations (except atan) to be in square brackets.  Thus,
for example "sin[90]" is allowed in the interpreter, but "sin 90" is
not. The atan operation must be in the format "atan[..]/[..]".

Production Rule Definitions in Terms of Tokens -

The following is a production rule definition of what this RS274NGC
interpreter recognizes as valid combinations of symbols which form a
recognized real_value (the top of this production hierarchy).

The notion of "integer_value" is used in the interpreter. Below it is
defined as a synonym for real_value, but in fact a constraint is added
which cannot be readily written in a production language.  An
integer_value is a real_value which is very close to an integer.
Integer_values are needed for array and table indices and (when
divided by 10) for the values of M-codes and G-codes. All numbers
(including integers) are read as real numbers and stored as doubles.
If an integer_value is required in some situation, a test for being
close to an integer is applied to the number after it is read.


arc_tangent_combo = arc_tangent expression divided_by expression .

binary_operation1 = divided_by | modulo | power | times .

binary_operation2 = and | exclusive_or | minus |  non_exclusive_or | plus .

combo1 = real_value { binary_operation1 real_value } .

digit = zero | one | two | three | four | five | six | seven |eight | nine .

expression =
   left_bracket
   (unary_combo | (combo1 { binary_operation2 combo1 }))
   right_bracket .

integer_value = real_value .

ordinary_unary_combo =  ordinary_unary_operation expression .

ordinary_unary_operation =
   absolute_value | arc_cosine | arc_sine | cosine | e_raised_to |
   fix_down | fix_up | natural_log_of | round | sine | square_root | tangent .

parameter_index = integer_value .

parameter_value = parameter_sign  parameter_index .

real_number =
   [ plus | minus ]
   (( digit { digit } decimal_point {digit}) | ( decimal_point digit {digit})).

real_value =
   real_number | expression | parameter_value | unary_combo.

unary_combo = ordinary_unary_combo | arc_tangent_combo .


Production Tokens in Terms of Characters -

absolute_value   = 'abs'
and              = 'and'
arc_cosine       = 'acos'
arc_sine         = 'asin'
arc_tangent      = 'atan'
cosine           = 'cos'
decimal_point    = '.'
divided_by       = '/'
eight            = '8'
exclusive_or     = 'xor'
e_raised_to      = 'exp'
five             = '5'
fix_down         = 'fix'
fix_up           = 'fup'
four             = '4'
left_bracket     = '['
minus            = '-'
modulo           = 'mod'
natural_log_of   = 'ln'
nine             = '9'
non_exclusive_or = 'or'
one              = '1'
parameter_sign   = '#'
plus             = '+'
power            = '**'
right_bracket    = ']'
round            = 'round'
seven            = '7'
sine             = 'sin'
six              = '6'
square_root      = 'sqrt'
tangent          = 'tan'
three            = '3'
times            = '*'
two              = '2'
zero             = '0'

When this function is called, the counter should be set at a left
bracket. The function reads up to and including the right bracket
which closes the expression.

The basic form of an expression is: [v1 bop v2 bop ... vn], where the
vi are real_values and the bops are binary operations. The vi may be
numbers, parameters, expressions, or unary functions. Because some
bops are to be evaluated before others, for understanding the order of
evaluation, it is useful to rewrite the general form collecting any
subsequences of bops of the same precedence. For example, suppose the
expression is: [9+8*7/6+5-4*3**2+1]. It may be rewritten as:
[9+[8*7/6]+5-[4*[3**2]]+1] to show how it should be evaluated.

The manual provides that operations of the same precedence should be
processed left to right.

The first version of this function is commented out. It is suitable
for when there are only two precedence levels. It is an improvement
over the version used in interpreters before 2000, but not as general
as the second version given here.

The first version of this function reads the first value and the first
operation in the expression. Then it calls either read_rest_bop1 or
read_rest_bop2 according to whether the first operation is a bop1 or a
bop2.  Read_rest_bop1 resets the next_operation to either a right
bracket or a bop2. If it is reset to a bop2, read_rest_bop2 is called
when read_rest_bop1 returns.

*/

#ifdef UNDEFINED
int Interp::read_real_expression(char *line,     //!< string: line of RS274/NGC code being processed
                                int *counter,   //!< pointer to a counter for position on the line 
                                double *value,  //!< pointer to double to be read                  
                                double *parameters)     //!< array of system parameters                    
{
  static char name[] = "read_real_expression";
  int next_operation;
  int status;

  CHKS((line[*counter] != '['), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHP(read_real_value(line, counter, value, parameters));
  CHP(read_operation(line, counter, &next_operation));
  if (next_operation == RIGHT_BRACKET); /* nothing to do */
  else if (next_operation < AND2) {     /* next operation is a bop1, times-like */
    CHP(read_rest_bop1(line, counter, value, &next_operation, parameters));
    if (next_operation == RIGHT_BRACKET);       /* next_operation has been reset */
    else                        /* next_operation is now a bop2, plus-like */
      CHP(read_rest_bop2(line, counter, value, next_operation, parameters));
  } else                        /* next operation is a bop2, plus-like */
    CHP(read_rest_bop2(line, counter, value, next_operation, parameters));
  return INTERP_OK;
}
#endif

/****************************************************************************/

/*! read_real_expression

The following version is stack-based and fully general. It is the
classical stack-based version with left-to-right evaluation of
operations of the same precedence. Separate stacks are used for
operations and values, and the stacks are made with arrays
rather than lists, but those are implementation details. Pushing
and popping are implemented by increasing or decreasing the
stack index.

Additional levels of precedence may be defined easily by changing the
precedence function. The size of MAX_STACK should always be at least
as large as the number of precedence levels used. We are currently
using four precedence levels (for right-bracket, plus-like operations,
times-like operations, and power).

N.B.: We are now using six levels (right-bracket, logical operations,
relational operations, plus-like operations, times-like operations, and
power).

*/

#define MAX_STACK 7

int Interp::read_real_expression(char *line,     //!< string: line of RS274/NGC code being processed
                                int *counter,   //!< pointer to a counter for position on the line 
                                double *value,  //!< pointer to double to be computed              
                                double *parameters)     //!< array of system parameters                    
{
  double values[MAX_STACK];
  int operators[MAX_STACK];
  int stack_index;

  CHKS((line[*counter] != '['), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHP(read_real_value(line, counter, values, parameters));
  CHP(read_operation(line, counter, operators));
  stack_index = 1;
  for (; operators[0] != RIGHT_BRACKET;) {
    CHP(read_real_value(line, counter, values + stack_index, parameters));
    CHP(read_operation(line, counter, operators + stack_index));
    if (precedence(operators[stack_index]) >
        precedence(operators[stack_index - 1]))
      stack_index++;
    else {                      /* precedence of latest operator is <= previous precedence */

      for (; precedence(operators[stack_index]) <=
           precedence(operators[stack_index - 1]);) {
        CHP(execute_binary((values + stack_index - 1),
                           operators[stack_index - 1],
                           (values + stack_index)));
        operators[stack_index - 1] = operators[stack_index];
        if ((stack_index > 1) &&
            (precedence(operators[stack_index - 1]) <=
             precedence(operators[stack_index - 2])))
          stack_index--;
        else
          break;
      }
    }
  }
  *value = values[0];
  return INTERP_OK;
}


/****************************************************************************/

/*! read_real_number

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The first character is not "+", "-", "." or a digit:
      NCE_BAD_NUMBER_FORMAT
   2. No digits are found after the first character and before the
      end of the line or the next character that cannot be part of a real:
      NCE_NO_DIGITS_FOUND_WHERE_REAL_NUMBER_SHOULD_BE
   3. sscanf fails: NCE_SSCANF_FAILED

Side effects:
   The number read from the line is put into what double_ptr points at.
   The counter is reset to point to the first character after the real.

Called by:  read_real_value

This attempts to read a number out of the line, starting at the index
given by the counter. It stops when the first character that cannot
be part of the number is found.

The first character may be a digit, "+", "-", or "."
Every following character must be a digit or "." up to anything
that is not a digit or "." (a second "." terminates reading).

This function is not called if the first character is NULL, so it is
not necessary to check that.

The temporary insertion of a NULL character on the line is to avoid
making a format string like "%3lf" which the LynxOS compiler cannot
handle.

*/

int Interp::read_real_number(char *line, //!< string: line of RS274/NGC code being processed
                            int *counter,       //!< pointer to a counter for position on the line 
                            double *double_ptr) //!< pointer to double to be read                  
{
  char *start;
  size_t after;

  start = line + *counter;

  after = strspn(start, "+-");
  after = strspn(start+after, "0123456789.") + after;

  std::string st(start, start+after);
  std::stringstream s(st);
  double val;
  if(!(s >> val)) ERS(_("bad number format (conversion failed) parsing '%s'"), st.c_str());
  if(s.get() != std::char_traits<char>::eof()) ERS(_("bad number format (trailing characters) parsing '%s'"), st.c_str());

  *double_ptr = val;
  *counter = start + after - line;
  //fprintf(stderr, "got %f   rest of line=%s\n", val, line+*counter);
  return INTERP_OK;
}

/****************************************************************************/

/*! read_real_value

Returned Value: int
   If one of the following functions returns an error code,
   this returns that code.
      read_real_expression
      read_parameter
      read_unary
      read_real_number
   If no characters are found before the end of the line this
   returns NCE_NO_CHARACTERS_FOUND_IN_READING_REAL_VALUE.
   Otherwise, this returns INTERP_OK.

Side effects:
   The value read from the line is put into what double_ptr points at.
   The counter is reset to point to the first character after the
   characters which make up the value.

Called by:
   read_a
   read_b
   read_c
   read_f
   read_g
   read_i
   read_integer_value
   read_j
   read_k
   read_p
   read_parameter_setting
   read_q
   read_r
   read_real_expression
   read_s
   read_x
   read_y
   read_z

This attempts to read a real value out of the line, starting at the
index given by the counter. The value may be a number, a parameter
value, a unary function, or an expression. It calls one of four
other readers, depending upon the first character.

*/

int Interp::read_real_value(char *line,  //!< string: line of RS274/NGC code being processed
                           int *counter,        //!< pointer to a counter for position on the line 
                           double *double_ptr,  //!< pointer to double to be read                  
                           double *parameters)  //!< array of system parameters                    
{
  char c, c1;

  c = line[*counter];
  CHKS((c == 0), NCE_NO_CHARACTERS_FOUND_IN_READING_REAL_VALUE);

  c1 = line[*counter+1];

  if (c == '[')
    CHP(read_real_expression(line, counter, double_ptr, parameters));
  else if (c == '#')
  {
    CHP(read_parameter(line, counter, double_ptr, parameters, false));
  }
  else if (c == '+' && c1 && !isdigit(c1) && c1 != '.')
  {
    (*counter)++;
    CHP(read_real_value(line, counter, double_ptr, parameters));
  }
  else if (c == '-' && c1 && !isdigit(c1) && c1 != '.')
  {
    (*counter)++;
    CHP(read_real_value(line, counter, double_ptr, parameters));
    *double_ptr = -*double_ptr;
  }
  else if ((c >= 'a') && (c <= 'z'))
    CHP(read_unary(line, counter, double_ptr, parameters));
  else
    CHP(read_real_number(line, counter, double_ptr));

  CHKS(std::isnan(*double_ptr),
          _("Calculation resulted in 'not a number'"));
  CHKS(std::isinf(*double_ptr),
          _("Calculation resulted in 'infinity'"));

  return INTERP_OK;
}

/****************************************************************************/

/*! read_rest_bop1

Returned Value: int
  If any of the following functions returns an error code,
  this returns that code.
     execute_binary1
     read_real_value
     read_operation
  Otherwise, it returns INTERP_OK.

Side effects:
   The value argument is set to the value of the expression.
   The counter is reset to point to the first character after the real
   expression.

Called by:
  read_real_expression
  read_rest_bop2

The value argument has a value in it when this is called. This repeatedly
gets the next_value and the next_operation, performs the last_operation
on the value and the next_value and resets the last_operation to the
next_operation. Observe that both the value and the last_operation
are passed back to the caller.

This is commented out since it is not used in the uncommented version
of read_real_expression. It has been tested.

*/

#ifdef UNDEFINED
int Interp::read_rest_bop1(char *line,   //!< string: line of RS274/NGC code being processed
                          int *counter, //!< pointer to a counter for position on the line 
                          double *value,        //!< pointer to double to be calculated            
                          int *last_operation,  //!< last operation read, reset to next operation  
                          double *parameters)   //!< array of system parameters                    
{
  static char name[] = "read_rest_bop1";
  double next_value;
  int next_operation;
  int status;

  for (;;) {
    CHP(read_real_value(line, counter, &next_value, parameters));
    CHP(read_operation(line, counter, &next_operation));
    CHP(execute_binary1(value, *last_operation, &next_value));
    *last_operation = next_operation;
    if (next_operation >= AND2) /* next op is a bop2 or right bracket */
      break;
  }
  return INTERP_OK;
}
#endif

/****************************************************************************/

/*! read_rest_bop2

Returned Value: int
  If any of the following functions returns an error code,
  this returns that code.
     execute_binary2
     read_real_value
     read_operation
     read_rest_bop1
  Otherwise, it returns INTERP_OK.

Side effects:
   The value argument is set to the value of the expression.
   The counter is reset to point to the first character after the real
   expression.

Called by:  read_real_expression

The value argument has a value in it when this is called. This repeatedly
gets the next_value and the next_operation, performs the last_operation
on the value and the next_value and resets the last_operation to the
next_operation. If the next_operation is ever a bop1 read_rest_bop1 is
called to set the next_value.

This is commented out since it is not used in the uncommented version
of read_real_expression. It has been tested.

*/

#ifdef UNDEFINED
int Interp::read_rest_bop2(char *line,   //!< string: line of RS274/NGC code being processed
                          int *counter, //!< pointer to a counter for position on the line 
                          double *value,        //!< pointer to double to be calculated            
                          int last_operation,   //!< last operation read                           
                          double *parameters)   //!< array of system parameters                    
{
  static char name[] = "read_rest_bop2";
  double next_value;
  int next_operation;
  int status;

  for (;; last_operation = next_operation) {
    CHP(read_real_value(line, counter, &next_value, parameters));
    CHP(read_operation(line, counter, &next_operation));
    if (next_operation < AND2) {        /* next operation is a bop1 */
      CHP(read_rest_bop1(line, counter, &next_value,
                         &next_operation, parameters));
    }
    CHP(execute_binary2(value, last_operation, &next_value));
    if (next_operation == RIGHT_BRACKET)
      break;
  }
  return INTERP_OK;
}
#endif

/****************************************************************************/

/*! read_s

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not s:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A spindle speed has already been inserted in the block:
      NCE_MULTIPLE_S_WORDS_ON_ONE_LINE
   3. The spindle speed is negative: NCE_NEGATIVE_SPINDLE_SPEED_USED

Side effects:
   counter is reset to the character following the spindle speed.
   A spindle speed setting is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 's', indicating a spindle speed
setting. The function reads characters which tell how to set the spindle
speed.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_s(char *line,   //!< string: line of RS274NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line
                  block_pointer block,  //!< pointer to a block being filled from the line
                  double *parameters)   //!< array of system parameters                   
{
  double value;

  CHKS((line[*counter] != 's'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->s_flag), NCE_MULTIPLE_S_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  CHKS((value < 0.0), NCE_NEGATIVE_SPINDLE_SPEED_USED);
  block->s_number = value;
  block->s_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_t

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not t:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A t_number has already been inserted in the block:
      NCE_MULTIPLE_T_WORDS_ON_ONE_LINE
   3. The t_number is negative: NCE_NEGATIVE_TOOL_ID_USED

Side effects:
   counter is reset to the character following the t_number.
   A t_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 't', indicating a tool.
The function reads characters which give the (integer) value of the
tool code.

The value must be an integer or something that evaluates to a
real number, so read_integer_value is used to read it. Parameters
may be involved.

*/

int Interp::read_t(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  int value;

  CHKS((line[*counter] != 't'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->t_flag), NCE_MULTIPLE_T_WORDS_ON_ONE_LINE);
  CHP(read_integer_value(line, counter, &value, parameters));
  CHKS((value < 0), NCE_NEGATIVE_TOOL_ID_USED);
  block->t_number = value;
  block->t_flag = true;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_text

Returned Value: int
   If close_and_downcase returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, this returns:
       a. INTERP_ENDFILE if the percent_flag is true and the only
          non-white character on the line is %,
       b. INTERP_EXECUTE_FINISH if the first character of the
          close_and_downcased line is a slash, and
       c. INTERP_OK otherwise.
   1. The end of the file is found and the percent_flag is true:
      NCE_FILE_ENDED_WITH_NO_PERCENT_SIGN
   2. The end of the file is found and the percent_flag is false:
      NCE_FILE_ENDED_WITH_NO_PERCENT_SIGN_OR_PROGRAM_END
   3. The command argument is not null and is too long or the command
      argument is null and the line read from the file is too long:
      NCE_COMMAND_TOO_LONG

Side effects: See below

Called by:  Interp::read

This reads a line of RS274 code from a command string or a file into
the line array. If the command string is not null, the file is ignored.

If the end of file is reached, an error is returned as described
above. The end of the file should not be reached because (a) if the
file started with a percent line, it must end with a percent line, and
no more reading of the file should occur after that, and (b) if the
file did not start with a percent line, it must have a program ending
command (M2 or M30) in it, and no more reading of the file should
occur after that.

All blank space at the end of a line read from a file is removed and
replaced here with NULL characters.

This then calls close_and_downcase to downcase and remove tabs and
spaces from everything on the line that is not part of a comment. Any
comment is left as is.

The length is set to zero if any of the following occur:
1. The line now starts with a slash, but the second character is NULL.
2. The first character is NULL.
Otherwise, length is set to the length of the line.

An input line is blank if the first character is NULL or it consists
entirely of tabs and spaces and, possibly, a newline before the first
NULL.

Block delete is discussed in [NCMS, page 3] but the discussion makes
no sense. Block delete is handled by having this function return
INTERP_EXECUTE_FINISH if the first character of the
close_and_downcased line is a slash. When the caller sees this,
the caller is expected not to call Interp::execute if the switch
is on, but rather call Interp::read again to overwrite and ignore
what is read here.

The value of the length argument is set to the number of characters on
the reduced line.

*/

int Interp::read_text(
    const char *command,       //!< a string which may have input text, or null
    FILE * inport,     //!< a file pointer for an input file, or null
    char *raw_line,    //!< array to write raw input line into
    char *line,        //!< array for input line to be processed in
    int *length)       //!< a pointer to an integer to be set
{
  int index;

  if (command == NULL) {
    if (fgets(raw_line, LINELEN, inport) == NULL) {
      if(_setup.skipping_to_sub)
      {
        ERS(_("EOF in file:%s seeking o-word: o<%s> from line: %d"),
                 _setup.filename,
                 _setup.skipping_to_sub,
                 _setup.skipping_start);
      }
      if (_setup.percent_flag)
      {
        ERS(NCE_FILE_ENDED_WITH_NO_PERCENT_SIGN);
      }
      else
      {
        ERS(NCE_FILE_ENDED_WITH_NO_PERCENT_SIGN_OR_PROGRAM_END);
      }
    }
    _setup.sequence_number++;   /* moved from version1, was outside if */
    if (strlen(raw_line) == (LINELEN - 1)) { // line is too long. need to finish reading the line to recover
      for (; fgetc(inport) != '\n' && !feof(inport) ;) {
      }
      ERS(NCE_COMMAND_TOO_LONG);
    }
    for (index = (strlen(raw_line) - 1);        // index set on last char
         (index >= 0) && (isspace(raw_line[index]));
         index--) { // remove space at end of raw_line, especially CR & LF
      raw_line[index] = 0;
    }
    rtapi_strlcpy(line, raw_line, LINELEN);
    CHP(close_and_downcase(line));
    if ((line[0] == '%') && (line[1] == 0) && (_setup.percent_flag)) {
        FINISH();
        return INTERP_ENDFILE;
    }
  } else {
    CHKS((strlen(command) >= LINELEN), NCE_COMMAND_TOO_LONG);
    rtapi_strlcpy(raw_line, command, LINELEN);
    rtapi_strlcpy(line, command, LINELEN);
    CHP(close_and_downcase(line));
  }

  _setup.parameter_occurrence = 0;      /* initialize parameter buffer */

  if ((line[0] == 0) || ((line[0] == '/') && (GET_BLOCK_DELETE())))
    *length = 0;
  else
    *length = strlen(line);

  return INTERP_OK;
}

/****************************************************************************/

/*! read_unary

Returned Value: int
   If any of the following functions returns an error code,
   this returns that code.
     execute_unary
     read_atan
     read_operation_unary
     read_real_expression
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. the name of the unary operation is not followed by a left bracket:
      NCE_LEFT_BRACKET_MISSING_AFTER_UNARY_OPERATION_NAME

Side effects:
   The value read from the line is put into what double_ptr points at.
   The counter is reset to point to the first character after the
   characters which make up the value.

Called by:  read_real_value

This attempts to read the value of a unary operation out of the line,
starting at the index given by the counter. The atan operation is
handled specially because it is followed by two arguments.

*/

int Interp::read_unary(char *line,       //!< string: line of RS274/NGC code being processed
                      int *counter,     //!< pointer to a counter for position on the line 
                      double *double_ptr,       //!< pointer to double to be read                  
                      double *parameters)       //!< array of system parameters                    
{
  int operation;

  CHP(read_operation_unary(line, counter, &operation));
  CHKS((line[*counter] != '['),
      NCE_LEFT_BRACKET_MISSING_AFTER_UNARY_OPERATION_NAME);

  if (operation == EXISTS)
  {
      CHP(read_bracketed_parameter(line, counter, double_ptr, parameters, true));
      return INTERP_OK;
  }

  CHP(read_real_expression(line, counter, double_ptr, parameters));

  if (operation == ATAN)
    CHP(read_atan(line, counter, double_ptr, parameters));
  else
    CHP(execute_unary(double_ptr, operation));
  return INTERP_OK;
}

int Interp::read_u(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'u'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->u_flag), _("Multiple U words on one line"));
  CHP(read_real_value(line, counter, &value, parameters));
  block->u_flag = true;
  block->u_number = value;
  return INTERP_OK;
}

int Interp::read_v(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'v'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->v_flag), _("Multiple V words on one line"));
  CHP(read_real_value(line, counter, &value, parameters));
  block->v_flag = true;
  block->v_number = value;
  return INTERP_OK;
}

int Interp::read_w(char *line,   //!< string: line of RS274/NGC code being processed
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'w'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->w_flag), _("Multiple W words on one line"));
  CHP(read_real_value(line, counter, &value, parameters));
  block->w_flag = true;
  block->w_number = value;
  return INTERP_OK;
}


/****************************************************************************/

/*! read_x

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not x:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A x_coordinate has already been inserted in the block:
      NCE_MULTIPLE_X_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The x_flag in the block is turned on.
   An x_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'x', indicating a x_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_x(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'x'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->x_flag), NCE_MULTIPLE_X_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->x_flag = true;
  if(_setup.lathe_diameter_mode)
  {
    block->x_number = value / 2;
  }else
  {
  block->x_number = value;
  }
  return INTERP_OK;
}

int Interp::read_atsign(char *line, int *counter, block_pointer block,
                        double *parameters) {
    CHKS((line[*counter] != '@'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
    (*counter)++;
    CHP(read_real_value(line, counter, &block->radius, parameters));
    block->radius_flag = true;
    return INTERP_OK;
}

int Interp::read_carat(char *line, int *counter, block_pointer block,
                       double *parameters) {
    CHKS((line[*counter] != '^'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
    (*counter)++;
    CHP(read_real_value(line, counter, &block->theta, parameters));
    block->theta_flag = true;
    return INTERP_OK;
}
    
    

/****************************************************************************/

/*! read_y

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not y:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A y_coordinate has already been inserted in the block:
      NCE_MULTIPLE_Y_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The y_flag in the block is turned on.
   A y_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'y', indicating a y_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_y(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'y'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->y_flag), NCE_MULTIPLE_Y_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->y_flag = true;
  block->y_number = value;
  return INTERP_OK;
}

/****************************************************************************/

/*! read_z

Returned Value: int
   If read_real_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The first character read is not z:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. A z_coordinate has already been inserted in the block:
      NCE_MULTIPLE_Z_WORDS_ON_ONE_LINE

Side effects:
   counter is reset.
   The z_flag in the block is turned on.
   A z_number is inserted in the block.

Called by: read_one_item

When this function is called, counter is pointing at an item on the
line that starts with the character 'z', indicating a z_coordinate
setting. The function reads characters which tell how to set the
coordinate, up to the start of the next item or the end of the line.
This information is inserted in the block. The counter is then set to
point to the character following.

The value may be a real number or something that evaluates to a
real number, so read_real_value is used to read it. Parameters
may be involved.

*/

int Interp::read_z(char *line,   //!< string: line of RS274 code being processed    
                  int *counter, //!< pointer to a counter for position on the line 
                  block_pointer block,  //!< pointer to a block being filled from the line 
                  double *parameters)   //!< array of system parameters                    
{
  double value;

  CHKS((line[*counter] != 'z'), NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
  *counter = (*counter + 1);
  CHKS((block->z_flag), NCE_MULTIPLE_Z_WORDS_ON_ONE_LINE);
  CHP(read_real_value(line, counter, &value, parameters));
  block->z_flag = true;
  block->z_number = value;
  return INTERP_OK;
}

bool Interp::is_parameter_readonly(int index)
{
  for (int i = 0; i < n_readonly_parameters; i++) {
    if (readonly_parameters[i] == index) return true;
  }
  return false;
}
