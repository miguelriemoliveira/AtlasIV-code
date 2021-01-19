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
  char name0[]="MyImages/example1.jpg";
  IplImage *image=NULL, *ThreshImage=NULL;
  unsigned int a=128;
  CvSize ImgSize;
  
  
  mycameraclass_v1394 *NavCam;
  
  NavCam = new mycameraclass_v1394(1);
  
  
  image = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,3);
  
  
  
  printf("Hello, world!\n");

  ClassImageFunctions ImageFunctions; //create class object
  
  ClassImageFunctions &IF=ImageFunctions; //create reference
  
  IF.SetMaxLimit(255); //call class procedure
  
  while(1)
  {
 *image = NavCam->myGetImageColor();
 // image = cvLoadImage(name0,-1);
  
  ImgSize.width = image->width;
  ImgSize.height = image->height;
  
  ThreshImage=cvCreateImage(ImgSize,image->depth,1);
  
  cvCvtPixToPlane(image,ThreshImage,NULL,NULL,NULL);
  
  
  IF.IF_Threshold(ThreshImage,ThreshImage,a);
  
  
  cvWaitKey(10);
  
  cvNamedWindow("Imagem Inicial",1);
  cvShowImage("Imagem Inicial",image);
  cvNamedWindow("Imagem Final",1);
  cvShowImage("Imagem Final",ThreshImage);
  }
  

  
  
  
  
  return EXIT_SUCCESS;
}
