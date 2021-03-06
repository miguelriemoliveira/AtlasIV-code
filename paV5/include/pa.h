/*!
 * @file pa.h File Description...
 *
 */

#ifndef _PA_H_
#define _PA_H_
#include "cv.h"

int GetRowPixelWhiteWidth(IplImage *row, int *barycenter, int halfRowFill=0);
CvMat *GetOccupancyHistogram( IplImage *img, CvMat *vals, int chan, CvMat *baryC);
void CleanIsolatedPoints(IplImage *src, IplImage *dest, int mode);
void CleanIsolatedSmallFeatures7x7(IplImage *src, IplImage *dest, int mode);
void CleanIsolatedSmallFeatures(IplImage *src, IplImage *dest, int mode, int size);



#endif
