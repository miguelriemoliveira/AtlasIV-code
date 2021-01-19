/***************************************************************************
 *   Copyright (C) 2005 by Miguel oliveira                                 *
 *   mike@linux                                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef _IMAGEFUNCTIONS_CPP_
#define _IMAGEFUNCTIONS_CPP_
 
#include <ImageFunctions.h>

ClassImageFunctions::ClassImageFunctions()
{
	MaxLimit = 255;
}

char ClassImageFunctions::IF_Threshold(IplImage *src,IplImage *dst,unsigned char Limit)
{
	if(! src || ! dst ) return -1;
	cvThreshold(src,dst,Limit,MaxLimit,CV_THRESH_BINARY);
	return 1;
}	


char ClassImageFunctions::IF_Threshold(IplImage *src,IplImage *dst,unsigned int Limit)
{
	return IF_Threshold(src,dst,(unsigned char) Limit);
};


char ClassImageFunctions::SetMaxLimit(int val)
{

	if(val >=0 && val<256)
	{	
		MaxLimit = val;
		return 1;
	}
	else
	{	
		return -1;
	}

}
#endif
