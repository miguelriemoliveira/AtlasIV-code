/*!
 * @file joinimg.c Functions to join images and auxiliary actions
 *
 */

#include "cv.h"
#include "joinimg.h"

#include <stdio.h>

#ifndef abs
#    define abs(X) ((X)>=0 ? (X) : (-(X)))
#endif

/*!
 * @brief Join two images with a given overlapping region
 * @param img1 Left image (type IplImage*)
 * @param img2 Right image (type IplImage*). Must be the same size of img1
 * @param img3 Allocated space for joint image (type IplImage*). Must be large enough for images side by side.
 * @param hOver Integer with overlaping dimension (when made negative the fusion image is centered in the joint image frame)
 * @param bin Output binary images with threshold bin, or 0 for no thresholding. May be bitwise-ORed with SHOWCOLOR**** #define's for special effects or pure color thresholding. See joinimg.h header file for alternatives.
 * @param binOper open CV function to process binary images (cvOr, cvAnd) NULL uses cvOR. If output is not to be binary, a NULL imply image averaging; if cvOR is passed it performs a max on the 3 RGB components, and if cvAnd is passed a min on RGBs is done... a custom function is also possible (at user's runtime risk)
 * @return Pointer to joint image or NULL on error
 */
IplImage *joinImages( IplImage *img1,
		      IplImage *img2, 
		      IplImage *img3, 
		      int hOver, 
		      int bin, 
		      void (*binOper)(const CvArr*, const CvArr*, CvArr*, const CvArr*)
		    )
{
    IplImage* imgJoin=img3;
    IplImage* img1W, *img2W;
    int centerImg=0;

    if (bin < 0 ) return NULL; //Non negative thresholds unrecognized

    //Evaluate image properties and return if not matching the fundamental properties
    if( img1->width  != img2->width  || 
	img1->height != img2->height || 
	img1->depth  != img2->depth 
      ) return NULL;

    if( imgJoin->width < (img1->width+img2->width-abs(hOver)) ||
        imgJoin->height != img1->height ||
	imgJoin->depth  != img1->depth 
      ) return NULL;

    if( hOver < 0 ) centerImg=1;
    hOver = abs(hOver);

    if( bin & 0xFF )  /*if non-null perform bynary display*/
    {
    	img1W = cvCreateImage( cvSize(img1->width,img1->height), img1->depth, img1->nChannels );
    	img2W = cvCreateImage( cvSize(img1->width,img1->height), img1->depth, img1->nChannels );
    	//Threshold images
    	//Normal
    	cvThreshold( img1, img1W, bin & 0xFF, 255, CV_THRESH_BINARY );
    	cvThreshold( img2, img2W, bin & 0xFF, 255, CV_THRESH_BINARY );
    	//Adaptative -  not valid on multi-channel ?
    	//cvAdaptiveThreshold( img1, img1W, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 10 );
    	//cvAdaptiveThreshold( img2, img2W, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 10 );
    }
    else
    {
    	img1W = img1;
    	img2W = img2;
    }

    cvZero(imgJoin);

    if( img1->width-hOver > 0 ) //If overlap is total ther are no left or right parts
    {
    	//Copy left ROI
    	cvSetImageROI( img1W,   cvRect( 0,                   0, img1->width-hOver, img1->height) );
    	cvSetImageROI( imgJoin, cvRect( (hOver>>1)*centerImg,0, img1->width-hOver, img1->height) );
    	cvCopy(img1W, imgJoin, NULL );

    	//Copy right ROI
    	cvSetImageROI( img2W,   cvRect( hOver,                                            0, img2->width - hOver, img2->height)    );
    	cvSetImageROI( imgJoin, cvRect( img2->width+(hOver-(hOver-(hOver>>1)))*centerImg, 0, img2->width - hOver, imgJoin->height) );
    	cvCopy(img2W, imgJoin, NULL );
    }
    else if( img1->width-hOver < 0 ) //Impossible to have negative size overlaping zone
    {
    	if( bin & 0xFF )  /*if non-null perform bynary display*/
    	{
        	cvReleaseImage( & img1W );  //Free space created...
        	cvReleaseImage( & img2W );  //Free space created...
    	}
    	return imgJoin; // a black filled image...
    }


    //Copy central ROI with mixture.... (average or otyher special operations )
    if( hOver != 0)  //Only if there is overlap, otherwise no need to do anything
    {
        cvSetImageROI( img1W,    cvRect( img1->width-hOver,                      0, hOver, img1->height) );
        cvSetImageROI( img2W,    cvRect( 0,                                      0, hOver, img2->height) );
        cvSetImageROI( imgJoin,  cvRect( img1->width-hOver+(hOver>>1)*centerImg, 0, hOver, imgJoin->height) );

    	if( bin & 0xFF )  /*if non-null, perform binary display*/
        {
	    if( ! binOper) cvOr(img1W, img2W, imgJoin );
	    else binOper(img1W, img2W, imgJoin, NULL );
        }
        else //Well, the threshold level is zero. Perform some math on the common (overlapped area)
	{
		if( ! binOper)  //No operator is passed as bin operator. Do a simple average on both images
			cvAddWeighted( img1W, 0.5, img2W, 0.5, 0, imgJoin );  
		else   //Well. An operator is passed. Must perform single channel manipulation
		{
			//NB. Operations below count with the ROI set above. That is true for the following functions:
			// cvSplit, cvMerge
			if( img1W->nChannels == 3 )
			{
				IplImage* iA[3], *iB[3], *iC[3];  //to put the three channels of the 3 images (2 orgs and the joint)
				int i;
				for( i=0; i<3; i++)  //create 3 tmp single-ch images for each of the 3 img but only with the min size
				{
					CvRect rr = cvGetImageROI(img1W); //Size of ROI is the same for all so...
					iA[i] = cvCreateImage( cvSize(rr.width,rr.height), img1W->depth, 1 );
					iB[i] = cvCreateImage( cvSize(rr.width,rr.height), img1W->depth, 1 );
					iC[i] = cvCreateImage( cvSize(rr.width,rr.height), img1W->depth, 1 );
				}
				cvSplit( img1W, iA[0], iA[1], iA[2], NULL);  //split image 1 into 3 channels
				cvSplit( img2W, iB[0], iB[1], iB[2], NULL);  //split image 2 into 3 channels
				
				if( binOper == cvOr)  
					for( i=0; i<3; i++) 
						cvMax( iA[i], iB[i], iC[i]); //for the 2 images, find the max and put in joint
				else if( binOper == cvAnd) 
					for( i=0; i<3; i++) 
						cvMin( iA[i], iB[i], iC[i]); //for the 2 images, find the max and put in joint
				else for( i=0; i<3; i++) 
					binOper( iA[i], iB[i], iC[i], NULL); //A custom user operator performed at ch level!


				cvMerge( iC[0], iC[1], iC[2], NULL, imgJoin); //Finally combine all the channels

				//And release temporary storage of 3X3=9 smaller single ch images...
				for( i=0; i<3; i++)
				{
					cvReleaseImage(&iA[i]);
					cvReleaseImage(&iB[i]);
					cvReleaseImage(&iC[i]);
				}

			}
			else
			{ /*# of channels is not 3! Bad luck... let's suppose that tjis never happens :-) */ ;}
		}
	}
    }

    //Reset images ROIs so you can return them untouched...
    cvResetImageROI(img1);
    cvResetImageROI(img2);
    cvResetImageROI(imgJoin);

    if( bin & 0xFF )  /*if non-null means images were created...*/
    {
        cvReleaseImage( & img1W );  //Free space created...
        cvReleaseImage( & img2W );  //Free space created...
    }

    int bb = bin & 0x00FF & (SHOWCOLORMASK>>8) ;
    if(bb && img1->nChannels == 3 ) // IF special treatment on bin then do it here before leavin. Pure colors thresholding and selection
    {
    	IplImage *chR = cvCreateImage( cvSize(imgJoin->width,imgJoin->height), imgJoin->depth, 1 );
    	IplImage *chG = cvCreateImage( cvSize(imgJoin->width,imgJoin->height), imgJoin->depth, 1 );
    	IplImage *chB = cvCreateImage( cvSize(imgJoin->width,imgJoin->height), imgJoin->depth, 1 );
    	IplImage *mask = cvCreateImage( cvSize(imgJoin->width,imgJoin->height), imgJoin->depth, 1 );
	cvSplit( imgJoin, chB, chG, chR, NULL);

	if( bin & SHOWCOLORWHITE ) 
	{
		//printf("white\n");
		cvAnd( chR, chG, mask, NULL); cvAnd( chB, mask, chR, NULL); //R holds the And(R,G,B)
		cvCopy( chR, chB, NULL);
		cvCopy( chR, chG, NULL);
	}

	if( bin & SHOWCOLORBLUE ) 
	{
		//printf("blue\n");
		cvOr( chR, chG , mask, NULL); //mask holds the Or(R,G) = ~And(~R,~G)
		cvNot(chR, mask ); //mask holds the ~And(~R,~G)
		cvAnd( chB, mask, chB, NULL); //B holds the And(~R,~G,B)
		cvZero(chR); cvZero(chG);
	}
	if( bin & SHOWCOLORRED ) 
	{
		//printf("red\n");
		cvOr( chB, chG , mask, NULL); 
		cvNot(chB, mask );
		cvAnd( chR, mask, chR, NULL);
		cvZero(chB); cvZero(chG);
	}
	if( bin & SHOWCOLORGREEN ) 
	{
		//printf("green\n");
		cvOr( chB, chR , mask, NULL); 
		cvNot(chB, mask );
		cvAnd( chG, mask, chG, NULL);
		cvZero(chB); cvZero(chR);
	}
	if( bin & SHOWCOLORCYAN ) 
	{
		//printf("cyan\n");
		cvAnd(chG, chB , chG, NULL); 
		cvNot(chR, mask );
		cvAnd(chG, mask, chG, NULL);
		cvCopy(chG, chB); 
		cvZero(chR);
	}
	if( bin & SHOWCOLORYELLOW ) 
	{
		//printf("yellow\n");
		cvAnd(chR, chG , chR, NULL); 
		cvNot(chB, mask );
		cvAnd(chR, mask, chR, NULL);
		cvCopy(chR, chG); 
		cvZero(chB);
	}
	if( bin & SHOWCOLORMAGENTA ) 
	{
		//printf("magenta\n");
		cvAnd(chR, chB , chR, NULL); 
		cvNot(chG, mask );
		cvAnd(chR, mask, chR, NULL);
		cvCopy(chR, chB); 
		cvZero(chG);
	}

	cvMerge (chB, chG, chR, NULL, imgJoin);
        cvReleaseImage( & chR );  //Free created space 
        cvReleaseImage( & chG );  //id..
        cvReleaseImage( & chB );  //id..
        cvReleaseImage( & mask );  //id..
    }

    return imgJoin;
}

/*!
 * @brief Undistort an image using default or passed parameters
 * @param img1 Original image (type IplImage*) 
 * @param img2 Undistorted image (type IplImage*) and must have allocated space. Must be the same size of img1. If NULL, initial calculations are done according to parameter mats (below)
 * @param cp Array of 8 ints  with numbers between 0 and 100% where 50% is the default. If NULL default values are used. The order of params is fx, fy, cx, cy, k1, k2, k3, k4
 * @param mapx address of pointer to mapx. If NULL content uses default undistort on each call. 
 * @param mapy address of pointer to mapx. If NULL content uses default undistort on each call. 
 * @return Pointer to joint image or NULL on error
 */
IplImage *UndistortImage( IplImage *img1, IplImage *img2, int cp[8], CvMat **mapx, CvMat **mapy ) 
{
	//Pointers to be initialised somewhere else
	CvMat *intrMatrix, *distCoeffs;

	if( ! mapx || ! mapy )  //Normal repetition of undistort calculation at each call
	{
		if( ! img1 || ! img2 ) return NULL;
		//printf("slow!\n");
		ChangeBaseDistortionParameters( &intrMatrix, &distCoeffs, cp ) ; //And creates the matrices...
		cvUndistort2( img1, img2, intrMatrix, distCoeffs );
		cvReleaseMat( & intrMatrix);
		cvReleaseMat( & distCoeffs);
		return img2;
	}
	else // one or two maps are not NULL... there are things to be done...
	{
		if( ! img2) //OK. This is a sign that undistort maps have to be (re)computed
		{
			//printf("configure!\n");
			ChangeBaseDistortionParameters( &intrMatrix, &distCoeffs, cp ) ;
			//First free maps space and then recreate space but only free them if non NULL
			if( *mapx) cvReleaseMat( mapx); if( *mapy) cvReleaseMat( mapy);
			*mapx = cvCreateMat(img1->height, img1->width, CV_32FC1 );  // could use previous maps dims instead of image!
			*mapy = cvCreateMat(img1->height, img1->width, CV_32FC1 );
			cvInitUndistortMap( intrMatrix, distCoeffs, *mapx, *mapy );
			return img2;
		}
		else //Don't recompute maps, simply use them. This should be the fastest operation
		{
			if( ! *mapx || ! *mapy ) return NULL;
			//printf("fast!\n");
			cvRemap( img1, img2, *mapx, *mapy, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
			return img2;
		}
	}
}

/*!
 * @brief Readjust base distortion parameters
 * @param intrMatrix container to pointer to intrinsic matrix not previously allocated
 * @param distCoeffs container to pointer to distortion coefficients not previsouly allocated
 * @param cp Array of 8 ints  with numbers between 0 and 100% where 50% is the default. If NULL default values are used. The order of params is fx, fy, cx, cy, k1, k2, k3, k4
 */
void ChangeBaseDistortionParameters( CvMat **intrMatrix, CvMat **distCoeffs, int cp[8] ) 
{

    //Data for wide angle image lens for a given dimension
    //static const int locWidth  =160, locHeight =120;

    double  
	/*Calibration factors - must be scaled for other igm dims*/
	fx    =793.23653,
 	fy    =792.16899,
 	cx    =325.35040,
 	cy    =247.91573,
	/*Distortion parameters - must NOT be scaled for other img dims*/
 	k1    =-0.224149,
 	k2    =0.3246434,
 	k3    =0.0007780947,  //also known as p1
 	k4    =0.0002316947;  //also known as p2
     //double k5    =0.00000;       //not used

    const double FF = 10;
    if ( cp )  //If non-null then apply modifications...
    {
	fx *= ( FF*0.0015*(cp[0]-50)+1 );  
	fy *= ( FF*0.0015*(cp[1]-50)+1 ); 
	cx *= ( FF*0.002*(cp[2]-50)+1 );
	cy *= ( FF*0.002*(cp[3]-50)+1 );  
	k1 *= ( FF*10*0.001*(cp[4]-50)+1 ); 
	k2 *= ( FF*10*0.005*(cp[5]-50)+1 );
	k3 *= ( FF*100*0.01*(cp[6]-50)+1 );
	k4 *= ( FF*1000*0.01*(cp[7]-50)+1 );
    }

    double dc[] = {k1, k2, k3, k4};
    //CvMat intrMatrix, distCoeffs;
    //double im[] = { fx, 0,  cx,
    //	        0,  fy, cy,
    //	        0,  0,  1
    //	       };


    // Now Assume an image of 320 X 240 :-( Its bad to assume things like this
    double factX = 2; //320/locWidth;
    double factY = 2; //240/locHeight;

    double im[] = { fx/factX, 0,        cx/factX,
	            0,        fy/factY, cy/factY,
	            0,        0,        1};
    //========================================================
    // Now create the matrices and fill them
    CvMat intrMx=cvMat(3, 3, CV_64FC1, im);
    CvMat distCs=cvMat(4, 1, CV_64FC1, dc);

    //... and point them for return
    *intrMatrix = cvCloneMat( & intrMx );
    *distCoeffs = cvCloneMat( & distCs );

}

