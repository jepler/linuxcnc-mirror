//    Copyright 2005-2008, various authors
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
#include "sequential.h"
#include "gdk/gdktypes.h"

void DrawSeqStep(GdkPixmap * DrawPixmap,int x,int y,int Size,StrStep * pStep,char DrawingOption);
void DrawSeqTransition(GdkPixmap * DrawPixmap,int x,int y,int Size,StrTransition * pTransi,char DrawingOption);

void DrawSequentialPage( GdkPixmap * DrawPixmap, int PageNbr, char DrawingOption );

void DrawSeqElementForToolBar( GdkPixmap * DrawPixmap, int x, int y, int Size, int NumElement );
