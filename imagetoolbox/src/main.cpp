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

 
 #ifdef HAVE_CONFIG_H
#include <config.h>
#endif

//System Includes
#include <stdio.h>
#include <stdlib.h>

//opencv includes
#include <cv.h>
#include <highgui.h>

//my incudes
#include <mycameraclass_v1394.h>

//Project includes
#include <ImageFunctions.h>


/**
 * 
 * @param argc 
 * @param argv[] 
 * @return 
 */
int main(int argc, char *argv[])
{
  
  IplImage *Cam1Image = NULL;
  IplImage *Cam2Image = NULL;
  IplImage *Cam3Image = NULL;
  Cam1Image= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,3);
  Cam2Image= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,3);
  Cam3Image= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,3);
  
  mycameraclass_v1394 *Cam1Handle;
  mycameraclass_v1394 *Cam2Handle;
  mycameraclass_v1394 *Cam3Handle;
  
  Cam1Handle = new mycameraclass_v1394(0);
  Cam2Handle = new mycameraclass_v1394(1);
  Cam3Handle = new mycameraclass_v1394(2);
  
  
  
  
  
  
  while(1)
  {
 *Cam1Image = Cam1Handle->myGetImageColor();
 *Cam2Image = Cam2Handle->myGetImageColor();
 *Cam3Image = Cam3Handle->myGetImageColor();
 
  cvWaitKey(10);
  
  cvNamedWindow("Cam1Win",1);
  cvShowImage("Cam1Win",Cam1Image);
  cvNamedWindow("Cam2Win",1);
  cvShowImage("Cam2Win",Cam2Image);
  cvNamedWindow("Cam3Win",1);
  cvShowImage("Cam3Win",Cam3Image);
  
  }
  

  
  
  
  
  return EXIT_SUCCESS;
}
