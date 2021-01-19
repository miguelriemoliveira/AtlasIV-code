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
 
 #ifndef _IMAGEFUNCTIONS_H_
 #define _IMAGEFUNCTIONS_H_
 
 #include <cv.h>
 
 
 class ClassImageFunctions
 {
 public:
 	//Constructor
 	/**
 	 *       This is the constructor
 	 * @return 
 	 */
 	ClassImageFunctions();  
	
	//Destructor
	~ClassImageFunctions(){}; 
 
	//threshold procedure
	/**
	 * This funtion threshold src image and gives the result in dst image
	 * @param src ->source image
	 * @param dst ->destination image
	 * @param Limit ->threshol limit
	 * @return 
	 */
	char IF_Threshold(IplImage *src,IplImage *dst,unsigned char Limit); 
	
	//overload procedure in case of type mismatch change type to unsigned char
	char IF_Threshold(IplImage *src,IplImage *dst,unsigned int Limit); 
	
	
	//sets private variable
	/**
	 *        ola gosto muito de ti
	 * @param val 
	 * @return 
	 */
	char SetMaxLimit(int val); 
	
 private:
 	unsigned char MaxLimit;
 
 
 
 
 
 
 };
 #endif

   