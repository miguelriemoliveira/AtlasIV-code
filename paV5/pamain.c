/*!
 * @file pa.c Standalone application to test track analysis
 *
 */

#include "cv.h"
#include "highgui.h"
#include "pa.h"
#include <stdio.h>

#ifndef USECAMERAS
#   define USECAMERAS 0
#endif

#if USECAMERAS  //From environment or compiler....
#  include <mycameraclass_v1394.h> //caetano
#endif

#define LEFTCAMNODE 1
#define RIGHTCAMNODE 2

/*=======================================*/
void GlobalDisplayRefresh(void);
IplImage *DrawHistImage( IplImage *img, CvMat *vals, CvMat *bc);
void thresholdSlideBarCallBk(int pos);
void histogramSlideBarCallBk(int pos);

/*=======================================*/

static const int NN = 8;
struct vsStatus { IplImage *imgRaw, *img, *hist; 
	          char *winMainName, *winHistName, *winFiltName; 
		  CvMat *vv; //Values for the histogram...
		  CvMat *bc; //Values for the rows baricenters...
		  int bin, histLevel;
                 } sG;

//static char *versionG="V5";

/*==================================================*/
#define IMLEFT  "IM2-left.jpg"
#define IMRIGHT "IM2-right.jpg"

//#define IMLEFT  "Left1sempinos-0000000000.jpg"
//#define IMRIGHT "Right1sempinos-0000000000.jpg"

//#define IMLEFT  "Left1compinos-0000000000.jpg"
//#define IMRIGHT "Right1compinos-0000000000.jpg"

//#define IMAGE IMRIGHT
#define IMAGE "setaverde.jpg"
/*=======================================*/

/*!
 * @brief main is main of course. Nothing special...
 * @brief Oh! It is forced to use global variables due to poor openCV GUI functions :-(((
 * @brief accepts options a or h in alternative. 'a' forces the display of all images.
 * @brief 'h' gives the help message!
 */
int main( int argc, char** argv )
{
    int bin = 220;

    IplImage* imgOrg=NULL;
    IplImage* imgMain=NULL;
    IplImage* imgHist;
    
#   if USECAMERAS
    	//init images that otherwise are initialized by cvLoadimg
    	imgOrg= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
    	imgMain= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
	//init cameras
	mycameraclass_v1394 *LeftCamHandle;
	LeftCamHandle = new mycameraclass_v1394(LEFTCAMNODE );
	
	//Cameras aquisition
	*imgOrg = LeftCamHandle->myGetImageColor(); //aquire image form cam0
	cvConvertImage(imgOrg ,imgMain ,CV_CVTIMG_SWAP_RB); //convert from BGR to RGB
#   else
    	imgOrg = cvLoadImage( IMAGE, 0 ); //Load image and force grayscale 0 (or not 1)
    	imgMain= cvCreateImage(cvSize(imgOrg->width,imgOrg->height),IPL_DEPTH_8U, imgOrg->nChannels ); //1 channel...
#   endif

    cvThreshold( imgOrg, imgMain, bin, 255, CV_THRESH_BINARY ); //Threshold image
    imgHist=cvCreateImage( cvSize(200,imgMain->height), imgMain->depth, 3); //create a RGB image for hist display


    //Fill global structure for the callback to update :-(
    sG.imgRaw = imgOrg;
    sG.img = imgMain;
    sG.hist =imgHist;
    sG.winMainName="MainImgBin";
    sG.winHistName="NormHist";
    sG.winFiltName="FiltImg";
    sG.bin = bin; 
    sG.histLevel = 100; 
    sG.vv=cvCreateMat( 1, imgMain->height, CV_32F );  //vector of doubles to store the histogram
    sG.bc=cvCreateMat( 1, imgMain->height, CV_32F );  //vector of doubles to store the row barycenters. Can be NULL to ignore

    
    //Now create and position windows
    cvNamedWindow( sG.winMainName, CV_WINDOW_AUTOSIZE );
    cvMoveWindow( sG.winMainName, 220, 10 );
    cvNamedWindow( sG.winHistName, CV_WINDOW_AUTOSIZE );
    cvMoveWindow( sG.winHistName, 550, 10 );

    cvNamedWindow( sG.winFiltName, CV_WINDOW_AUTOSIZE );
    cvMoveWindow( sG.winFiltName, 220, 335 );


    //Update all windows
    GlobalDisplayRefresh();

    //Create the initial trackbars
    cvCreateTrackbar( "Threshold", sG.winMainName , &sG.bin, 255, thresholdSlideBarCallBk );
    cvCreateTrackbar( "Hist", sG.winHistName , &sG.histLevel, 100, histogramSlideBarCallBk );

    //Wait until any key is pressed and process its codes
    //Only Enter quits the applications....
    int kCode;
    int keepGoing=1;
    while(keepGoing)
    {
	
#       if USECAMERAS
    	kCode=cvWaitKey(10);  //wait 10 ms if using cameras or 0 if no cameras
	//Cameras aquisition
	*imgMain = LeftCamHandle->myGetImageColor(); //aquire image form cam0
	cvConvertImage(imgOrg ,imgMain ,CV_CVTIMG_SWAP_RB); //convert from BGR to RGB
	sG.img = imgMain;
#       else
    	kCode=cvWaitKey(0);  //endless loop
#	endif
	
	switch((char)kCode)
	{
	    case 'a':  
		    break;
	    case '\n':
		keepGoing = 0;
		break;
	    //default: if(kCode != -2) keepGoing = 0;
	}
	GlobalDisplayRefresh();
    }

    //Now we are leaving...
    cvReleaseImage( & imgMain );  
    cvReleaseImage( & imgOrg );  
    cvReleaseImage( & imgHist );  
    cvReleaseMat( & sG.vv );  

    return 0;

}

/*!
 * @brief Auxiliary function which does the actual job of trackbars and uses global data :-( modified elsewhere
 * @brief Uses global variables :-( Not recommended
 */
void GlobalDisplayRefresh(void)
{
    cvThreshold( sG.imgRaw, sG.img, sG.bin, 255, CV_THRESH_BINARY ); //Threshold image
    // Do something concerning the hist level.... nothing yet...
    //if( ! GetOccupancyHistogram( sG.img, sG.vv, 1, sG.bc) ) return;  //Use channels 1 by default (0 has the same effect)
    //DrawHistImage(sG.hist, sG.vv, sG.bc);
    cvShowImage( sG.winMainName, sG.img );  
    //cvShowImage( sG.winHistName, sG.hist ); 
    CleanIsolatedSmallFeatures(sG.img, sG.img, 0, 17);  //Filter 17X17
    CleanIsolatedSmallFeatures(sG.img, sG.img, 2, 17);  //Mode 2 forces all fiterings up to isolated points starting on 15X15....
    CleanIsolatedPoints(sG.img, sG.img, 1);  //1 to eliminate all types of isolated pixels  //POssibly resundant

    cvShowImage( sG.winFiltName, sG.img ); 

}

IplImage *DrawHistImage( IplImage *img, CvMat *vals, CvMat *baryC)
{
	int n;
	int len = img->width;
	if( len < 100) return NULL;
	cvZero(img);
	for(n=0; n < img->height; n++)
	{
	    len = (int) cvmGet(vals, 0,n);
	    cvLine( img, cvPoint(0,n), cvPoint(len,n), CV_RGB( 255, 255,0 ), 1, 8, 0 );
	}

	if(baryC) // plot the barycenters.
	{
	    for(n=0; n < img->height; n++)
	    {
	    	len = (int) cvmGet(baryC, 0,n);
	    	cvLine( img, cvPoint(len,n), cvPoint(len,n), CV_RGB( 255, 0, 255 ), 2, 8, 0 );
	    }
	}

	return img;
}

//The (dummy) callbacks
void thresholdSlideBarCallBk(int pos) { GlobalDisplayRefresh(); }
void histogramSlideBarCallBk(int pos) { GlobalDisplayRefresh(); }
