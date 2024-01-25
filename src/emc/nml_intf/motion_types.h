//    Copyright 2008, Chris Radek <chris@timeguy.com>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

typedef enum {
    EMC_MOTION_TYPE_TRAVERSE = 1,
    EMC_MOTION_TYPE_FEED = 2,
    EMC_MOTION_TYPE_ARC = 3,
    EMC_MOTION_TYPE_TOOLCHANGE = 4,
    EMC_MOTION_TYPE_PROBING = 5,
    EMC_MOTION_TYPE_INDEXROTARY = 6,
} EMCMotionTypes;

#endif
