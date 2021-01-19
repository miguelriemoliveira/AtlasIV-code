/*!
 * @file pa.c 
 *
 */

#include "cv.h"
#include "pa.h"

#include <stdio.h>

#ifndef min
#   define min(a,b) ((a)>(b) ? (b) : (a))
#endif

void ppp(IplImage *src);
//void ppp(CvMat *src);

/*!
 * @brief Get occupancy line histogram in percentual value of track width
 * @param img Image to analyze. ROI and COI are ignored.
 * @param vals Matrix vector to fill with as many values as img height
 * @param chan Channel 0-4 to consider in image. If 0 channel is set, channel 1 will be used
 * @param bary POinter to a vector where to put image rows bayrcenters. Not used if NULL
 * @return The passed vals vector pointer or NULL in case of error
 */
CvMat *GetOccupancyHistogram( IplImage *img, CvMat *vals, int chan, CvMat *baryC) 
{
	int line;
	int count, rowWhiteWidth;
	int bC;
	IplImage *rowImg;
	if( ! img || ! vals ) return NULL;  //Invalid pointers
	if( img->nChannels < chan ) return NULL;

	rowImg=cvCreateImage( cvSize(img->width, 1), img->depth, 1);   //1 line high image

	//cvResetImageROI(imgDup);

	if(img->nChannels == 1 || chan == 0 ) 
		chan = 1; //If one channel only, no big deal... impose channel one
	else
		cvSetImageCOI( img, chan);  //OK can set the COI since it's multichannel and cvSplit will use it below...

	for(line=0; line < min(img->height, vals->width); line++)
	{
		cvSetImageROI( img, cvRect(0,line,img->width,1) ); //ROI for one line!

		if( img->nChannels > 1)
			cvSplit(img,rowImg,NULL,NULL,NULL); //This uses the COI to obtain the right channel, and ROI for one line
		else  //since cvSplit does not work with one channel-only images
			cvCopy( img,rowImg);

       		count = cvCountNonZero(rowImg); //And this one works only with 1 channel images (does not use COI)
		if(count) 
		{
			if(count > 1) //2 or more white pixels in the row
			{
				if(baryC)
				{
					rowWhiteWidth = GetRowPixelWhiteWidth( rowImg, & bC , 1);
	                                //printf("Line %d", line);
					cvCopy( rowImg,img); //Put image back
				}
				else
				{
					rowWhiteWidth = GetRowPixelWhiteWidth( rowImg, NULL, 0 );
				}

				count = (int)((count-1)*100/rowWhiteWidth);   //Percentual value
			}
			else count = 0;  //Only one pixel in the row. No need to verify distances...
		}
		cvmSet( vals, 0, line, (double)count );  //put the percentual value of track width into the array for return
		if(baryC) cvmSet( baryC, 0, line, (double)bC );  //put the barycenter percentual value into the array for return
	}


	cvResetImageROI(img);
	cvSetImageCOI(img, 0);
	cvReleaseImage( & rowImg);
	return vals;
}

/*!
 * @brief Calculate pixel width between the two extreme white pixels of the image row and optionally fill the half of track where barycenter is located
 * @param row Image to analyze. It must be one pixel high.
 * @param barycenter Track barycenter location between extreme white pixels. Ignored if NULL. Value between 0 (left) to 100 % (right)
 * @param baryLimit Barycenter limit value (in 5) to consider location between extreme white pixels. Ignored if NULL. Value between 0 (left) to 100 % (right)
 * @param halfRowFill Logical value indicating whether half row is to be filled with white pixel or not!
 * @return positive integer with pixel ditance betwwen extremes...
 */
int GetRowPixelWhiteWidth(IplImage *row, int *barycenter, int halfRowFill)
{
	int n, val, pL=0, pR=0;
	int sum = 0;
	if( row->height != 1 ) return -1;
	CvScalar pix;

	for(n = 0 ; n < row->width; n++)  //Start from begining of row
	{
		pix = cvGet1D( row, n );
		val = (int)pix.val[0];

		if(val && !pL )  { pL = n; break; } //Get leftmost white pixel
	}
	for(n = row->width-1; n>=0; n--)  //Start from end of row
	{
		pix = cvGet1D( row, n );
		val = (int)pix.val[0];
		if(val && pR<n) { pR = n;  break; } //Get rightmost white pixel
	}

	if(barycenter && (pL != pR) )  //Calculate line barycenter if more than 1 white pixel in the line
	{
		int nn = 0;
		for(n = pL ; n <= pR; n++)  
		{
			pix = cvGet1D( row, n );
			val = (int)pix.val[0];
			if (val) { sum += (n-pL); nn++;}
		}
		//*barycenter = sum/((pR-pL)*(pR-pL))/2/nn/100;
		//*barycenter = sum*200/nn/((pR-pL)*(pR-pL));
		//*barycenter = sum*100/(pR-pL)/(pR-pL);
		*barycenter = sum*100/nn/(pR-pL);
	}
	if( halfRowFill  ) //Now fill the row up to the middle or from the middle up to the end
	{
	    int midP = pL+((pR-pL) >> 1);
	    if (*barycenter>5 && *barycenter<15)
	        for(n=pL;n<midP;n++) cvSet1D( row, n , cvScalar(255));
	    else 
	      if(*barycenter>75 && *barycenter<95) //
	        for(n=midP;n<pR;n++) cvSet1D( row, n , cvScalar(255));

	    //printf("--- %d pL=%d pR=%d midP=%d\n", *barycenter, pL, pR, midP);
	}

	return (pR-pL);
}


/*!
 * @brief Clean isolated points. This is a particular of CleanIsolatedSmallFeatures with 3X3 size, but perhaps this one is more efficient :-)
 * @param src Image to process. If NULL, put the result on the src image.
 * @param dest Processed image. 
 * @param mode 0 eliminate white isolated pixels only; 1 eliminate all types of isolated pixels. 
 * @return Yet to define...
 */
void CleanIsolatedPoints(IplImage *src, IplImage *dest, int mode) 
{
	IplImage *dst;
        IplImage *tmp1=cvCloneImage(src);
        IplImage *tmp2;
	if(mode)  tmp2 = cvCloneImage(src);

	double se[] = { -1, -1, -1,
		        -1, +8, -1,
			-1, -1, -1};

	CvMat kernel=cvMat(3, 3, CV_64FC1, se);
	cvConvertScale( &kernel, &kernel, 1.0/255.0, 0 ); //Multiply per 1/255 and add 0

	cvFilter2D( src, tmp1, &kernel); //The 8s are isolated white pixels.
	cvThreshold( tmp1, tmp1, 7, 255, CV_THRESH_BINARY_INV ); //Make isolated points into a mask

	if(mode) //Perform also elimination of isolated black pixels.
	{
		cvNot(src, tmp2); //negate
		cvFilter2D( tmp2, tmp2, &kernel); //The 8s correspond now to isolated black pixels in white background
		cvThreshold( tmp2, tmp2, 7, 255, CV_THRESH_BINARY ); //Make isolated points into a mask
	}

	if(! dest) dst = src; else dst = dest;

	cvAnd( src, tmp1, tmp1);
	if(mode) cvOr( tmp1, tmp2, dst);

	cvReleaseImage( & tmp1);
	if(mode) cvReleaseImage( & tmp2);
}

/*!
 * @brief Clean isolated small features 7X7. Aperticular case just for initial testing
 * @param src Image to process. If NULL, put the result on the src image.
 * @param dest Processed image. 
 * @param mode 0 eliminate white isolated pixels only; 1 eliminate all types of isolated pixels. Not yet implemented.
 * @return Yet to define...
 */
void CleanIsolatedSmallFeatures7x7(IplImage *src, IplImage *dest, int mode) 
{
	IplImage *dst;
        IplImage *mask=cvCloneImage(src);
        IplImage *tmp2=cvCloneImage(src);

	double se[] = { 
		        +100, +100, +100, +100, +100, +100, +100,
		        +100,   +1,   +1,   +1,   +1,   +1, +100,
		        +100,   +1,   +1,   +1,   +1,   +1, +100,
		        +100,   +1,   +1,   +1,   +1,   +1, +100,
		        +100,   +1,   +1,   +1,   +1,   +1, +100,
		        +100,   +1,   +1,   +1,   +1,   +1, +100,
	                +100, +100, +100, +100, +100, +100, +100
		       };

	CvMat kernel=cvMat(7, 7, CV_64FC1, se);
	cvConvertScale( &kernel, &kernel, 1.0/255.0, 0 ); //Multiply per 1/255 and add 0

	cvFilter2D(src, mask, &kernel); 
	//ppp(mask);
	
	//Any pixel with number <10 is to become background. Other cases ingored.
	cvThreshold( mask, mask, 25, 255, CV_THRESH_BINARY ); // if<10 put it zero (make it background

	cvAnd(src, mask, tmp2);
	if(! dest) dst = src; else dst = dest;

	cvCopy( tmp2, dst);

	cvReleaseImage( & mask);
	cvReleaseImage( & tmp2);
}

/*!
 * @brief Clean isolated small features on a square up to a given size. Note this fumction cleans clusters of points up to a given size viben that a backgound border exists aroud them. a 11X11 filter may ocasionally not eliminate isolated points since and a 3X3 can! For full guaranty, several calls should be issued: 3X3, 5X5, 7X7, 9X9, 11X11 to properly ensure that all point formation from one single point up to clusters of 81=9X9 point are erased...
 * @param src Image to process. If NULL, put the result on the src image.
 * @param dest Processed image. 
 * @param mode 0 eliminate white isolated pixels only; 1 eliminate all types of isolated pixels. NOT YET implemented; 2 recursively aplly filters from size and below up to 3x3. In mode 2 the filtering occurs with one less filter than required in parameters :-((
 * @param size Size of square (including border) where feature to eliminate must fit (only odd numbers accepted) min=3 & max=21.
 * @return Yet to define...
 */
void CleanIsolatedSmallFeatures(IplImage *src, IplImage *dest, int mode, int size) 
{
	int l, c;
	IplImage *dst;

	if( size > 17) size = 17;  //Cannot go beyond...
	if( ! size%2 ) size ++;    //Only odd numbers, and use the following odd

	if( size < 3 ) 
		return;  //Nothing to do. Function returns without any processing
	else if( mode == 2)
	{
		//printf("Executing filtering for %d\n", size-2);
		CleanIsolatedSmallFeatures(src, dest, 2, size-2);
	}


        IplImage *mask=cvCreateImage( cvSize(src->width, src->height), IPL_DEPTH_8U, 1);  
        IplImage *src1=cvCloneImage(src);
	CvMat *kernel = cvCreateMat( size, size, CV_64FC1 );
	cvSet( kernel, cvScalar(1), NULL );  //Force all to one
	cvThreshold( src, src1, 128 , 1, CV_THRESH_BINARY ); //Turn image into zeros and ones for easiear operation


#define NNN	((double)((size-2)*(size-2)+1))
	//Now force last and first row and colums to NNN
	//cvCopyMakeBorder( kernel, kernel, cvPoint(0,0), IPL_BORDER_CONSTANT, cvScalarAll(NNN) );
	//The previous did nor work so I used the following sequence...
	for(l=0; l<size; l++)
		for( c=0; c < size; c++) 
			if(c == 0 || c == size-1 || l==0 || l==size-1) cvmSet( kernel, l,c, NNN ) ;

	//cvConvertScale( kernel, kernel, 1.0/NNN, 0 ); //Multiply per 1/100 and add 0 to have a normalised kernel 0-1

	////////// Just to print out the kernel.....
	/*
	for(l=0; l<size; l++) { for( c=0; c < size; c++) printf("%3d  ", (int)cvmGet( kernel, l, c ) ) ; printf("\n"); }
	*/

	cvFilter2D(src1, mask, kernel); 
	cvMul(src1, mask, mask); //Only white pixels should influence the mask... Got you :-))))
	cvSubRS( mask, cvScalar(255), mask);  //performs (255 - filter result)
	//If value greater or equal than 255-NNN then force mask to Zero since it's a pixel with blak in the neibourhood
	cvThreshold( mask, mask, 255-NNN-1, 255, CV_THRESH_BINARY_INV ); 

	//ppp(mask);
	
	if(! dest) dst = src; else dst = dest;
	cvConvert( mask, dst); //transfer the result to destination and convert if necessary.

	cvReleaseMat(&kernel);
	cvReleaseImage( & mask);
	cvReleaseImage( & src1);

}



//Just a function for debugging feedback
void ppp(IplImage *src)
//void ppp(CvMat *src)
{
	int n;
	const int rr = 110;
	CvScalar pix;
	int val;
	for(n=0; n < src->width; n++ )
	{
		pix = cvGet1D(src, rr*src->width+n );  //A row at line rr
		val = (int)pix.val[0];
		printf("%d ", val);
		//pix.val[0]=255; cvSet1D(src, rr*src->width+n, pix );  //Force a row to be white
	}
	printf("\n-----------------\n");
}
