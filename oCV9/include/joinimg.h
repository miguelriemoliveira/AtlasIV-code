/*!
 * @file joinimg.h File Description...
 *
 */

#ifndef _JOINIMG_H_
#define _JOINIMG_H_
#include "cv.h"

#define SHOWCOLORMASK	0xFF00
#define	SHOWCOLORALL	0x0000
#define SHOWCOLORBLUE	0x0100
#define	SHOWCOLORGREEN	0x0200
#define SHOWCOLORRED	0x0400
#define SHOWCOLORWHITE	0x0800
#define SHOWCOLORCYAN	0x1000
#define SHOWCOLORMAGENTA 0x2000
#define SHOWCOLORYELLOW 0x4000

IplImage *joinImages( IplImage *img1,IplImage *img2, IplImage *img3, int hOver, int bin, void (*binOper)(const CvArr*, const CvArr*, CvArr*, const CvArr*) );
IplImage *UndistortImage( IplImage *img1, IplImage *img2, int cp[8], CvMat **mapx=NULL, CvMat **mapy=NULL );
void ChangeBaseDistortionParameters( CvMat **intrMatrix, CvMat **distCoeffs, int cp[8] );

#endif
