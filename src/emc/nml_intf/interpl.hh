/********************************************************************
* Description: interpl.hh
*   Mechanism for building lists of arbitrary NML messages, used by
*   the canonical interface and interpreter to pass planned sequences
*   to the HME.
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
#ifndef INTERP_LIST_HH
#define INTERP_LIST_HH

#include <stdint.h>

struct NMLmsg;

#define MAX_NML_COMMAND_SIZE 1000

// these go on the interp list
struct NML_INTERP_LIST_NODE {
    int line_number;		// line number it was on
    union _dummy_union {
	int32_t i;
	int32_t l;
	double d;
	float f;
	int64_t ll;
	long double ld;
    } dummy;			// paranoid alignment variable.

    union _command_union {
	char commandbuf[MAX_NML_COMMAND_SIZE];	// the NML command;
	int32_t i;
	int32_t l;
	double d;
	float f;
	int64_t ll;
	long double ld;
    } command;
};

// here's the interp list itself
class NML_INTERP_LIST {
  public:
    NML_INTERP_LIST();
    ~NML_INTERP_LIST();

    int set_line_number(int line);
    int get_line_number();
    int append(NMLmsg &);
    int append(NMLmsg *);
    NMLmsg *get();
    void clear();
    void print();
    int len();

  private:
    class LinkedList * linked_list_ptr;
    NML_INTERP_LIST_NODE temp_node;	// filled in and put on the list
    int next_line_number;	// line number used to fill temp_node
    int line_number;		// line number of node from get()
};

extern NML_INTERP_LIST interp_list;	/* NML Union, for interpreter */

#endif
