/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file readlights.cpp
* @brief All the functions used to process the lights image and reach a conclusion concerning the order from the lights.
* 
*This file executes the following procedure: \n Given a raw RGB image:
* 
* @image html "../images/step1_lights.PNG" "RGB original image"
* One converts the last image to HSV colorspace (AF_GetLightsImage)
* @image html "../images/step2_lights.PNG" "HSV image"
* Three images can be obtained by separating the three components H S and V. Color recognition is made by comparing h values of the images.
* The shape recognition is made by creating a minimum area rectangle and a convex hull images.
* @image html "../images/step4_lights.PNG" "Minimum Area Rectangle"
* @image html "../images/step5_lights.PNG" "Convex Hull"
* And the output would be the following image:
* @image html "../images/step6_lights.PNG" "Final Image"
* This is a brief tutorial and of some of the things that are done, check the Atlas III report for more details.
*/
#ifndef _READLIGHTS_
#define _READLIGHTS_

#include "header.h"


#define vsclean(src) CleanIsolatedSmallFeatures(src, src, 0, 7);\
		     CleanIsolatedPoints(src, src, 1);

// {
// 		      }

void RL_ApplyFilter_1(char FilterIdent)
{
	switch (FilterIdent)
	{
		case 1:
		{
			cvCmpS( GI.Lights.S, (double) SATURATIONLIMIT, GI.Lights.S_Filter, CV_CMP_GT);
		}
		case 2:
		{
			cvCmpS( GI.Lights.V, (double) INTENSITYLIMIT, GI.Lights.V_Filter, CV_CMP_GT);
		}
	}
}




/** @brief This function creates the Green_Filter and Red_Filter images.
 *
 *This function goes through every pixel in the H_img and the SandV_Filter images. For each if the SandV_Filter exists it checks the H_img values and if they are within the intervals given by the GREENFILTERUPPERLIMIT, GREENFILTERLOWERLIMIT, REDFILTERUPPERLIMIT and REDFILTERLOWERLIMIT and creates the Red_Filter and Green_Filter images accordingly. It then makes some statistical analysis to decide wich color is present.
 */
void RL_ColorRecognition_1()
{

	cvInRangeS( GI.Lights.H, cvScalar(GREENFILTERLOWERLIMIT) , cvScalar(GREENFILTERUPPERLIMIT) , GI.Lights.Green_Filter);
	cvAnd(GI.Lights.Green_Filter, GI.Lights.SandV_Filter, GI.Lights.Green_Filter);
	LightsAnalisys.PixelCounts.PC_GreenCount =  cvCountNonZero( GI.Lights.Green_Filter);
     	//vsclean(GI.Lights.Green_Filter);
 	
	
	cvInRangeS( GI.Lights.H, cvScalar(REDFILTERLOWERLIMIT) , cvScalar(REDFILTERUPPERLIMIT) , GI.Lights.Red_Filter);
	cvAnd(GI.Lights.Red_Filter, GI.Lights.SandV_Filter, GI.Lights.Red_Filter);
	LightsAnalisys.PixelCounts.PC_RedCount =  cvCountNonZero( GI.Lights.Red_Filter);
     	
	
	cvInRangeS( GI.Lights.H, cvScalar(YELLOWFILTERLOWERLIMIT) , cvScalar(YELLOWFILTERUPPERLIMIT) , GI.Lights.Yellow_Filter);
	cvAnd(GI.Lights.Yellow_Filter, GI.Lights.SandV_Filter, GI.Lights.Yellow_Filter);
	LightsAnalisys.PixelCounts.PC_YellowCount =  cvCountNonZero( GI.Lights.Yellow_Filter);
     	//vsclean(GI.Lights.Yellow_Filter);
	
	
	
	LightsAnalisys.PixelCounts.PC_MaxCount=macro_max(macro_max(LightsAnalisys.PixelCounts.PC_RedCount,LightsAnalisys.PixelCounts.PC_GreenCount), LightsAnalisys.PixelCounts.PC_YellowCount);
		
	
	if ( LightsAnalisys.PixelCounts.PC_MaxCount<MINIMUMCOUNTVALUE )
	{
		LightsAnalisys.DetectedColor=DC_NONE;
	}
	else if ( LightsAnalisys.PixelCounts.PC_MaxCount == LightsAnalisys.PixelCounts.PC_YellowCount)
	{
		
		cvAnd(GI.Lights.SandV_Filter, GI.Lights.Yellow_Filter, GI.Lights.SandV_Filter);
		LightsAnalisys.DetectedColor=DC_YELLOW;
		vsclean(GI.Lights.SandV_Filter);
	}
	else if (LightsAnalisys.PixelCounts.PC_MaxCount == LightsAnalisys.PixelCounts.PC_GreenCount)
	{
		
		cvAnd(GI.Lights.SandV_Filter, GI.Lights.Green_Filter, GI.Lights.SandV_Filter);
		LightsAnalisys.DetectedColor=DC_GREEN;
		vsclean(GI.Lights.SandV_Filter);
	}
	else if (LightsAnalisys.PixelCounts.PC_MaxCount == LightsAnalisys.PixelCounts.PC_RedCount)
	{	
		
		cvAnd(GI.Lights.SandV_Filter, GI.Lights.Red_Filter, GI.Lights.SandV_Filter);
		LightsAnalisys.DetectedColor=DC_RED;
		vsclean(GI.Lights.SandV_Filter);
	}
	
	
	 
	//cvMul( const CvArr* src1, const CvArr* src2, CvArr* dst, double scale=1 );
	//void cvSetImageCOI( IplImage* image, int coi );
	//void cvCopy( const CvArr* src, CvArr* dst, const CvArr* mask=NULL );
	//void cvSet( CvArr* arr, CvScalar value, const CvArr* mask=NULL );
	
	
	if (LightsAnalisys.DetectedColor==DC_YELLOW)		
		cvSet( img1, CV_RGB(Color.yellow[0],Color.yellow[1],Color.yellow[2]), GI.Lights.Yellow_Filter);	
	else if (LightsAnalisys.DetectedColor==DC_GREEN)		
		cvSet( img1, CV_RGB(Color.green[0],Color.green[1],Color.green[2]), GI.Lights.Green_Filter);	
	else if (LightsAnalisys.DetectedColor==DC_RED)		
		cvSet( img1, CV_RGB(Color.red[0],Color.red[1],Color.red[2]), GI.Lights.Red_Filter);	
			
	#if NPOUT
		printf("____________ColorRecognition Display__________\n");
		printf("LightsAnalisys.PixelCounts.PC_GreenCount=%d\n",LightsAnalisys.PixelCounts.PC_GreenCount);
		printf("LightsAnalisys.PixelCounts.PC_RedCount=%d\n",LightsAnalisys.PixelCounts.PC_RedCount);
		printf("LightsAnalisys.PixelCounts.PC_YellowCount=%d\n",LightsAnalisys.PixelCounts.PC_YellowCount);
		printf("LightsAnalisys.PixelCounts.PC_MaxCount=%d\n",LightsAnalisys.PixelCounts.PC_MaxCount);
		printf("LightsAnalisys.PixelCounts.PC_MinCount=%d\n",LightsAnalisys.PixelCounts.PC_MinCount);
		if (LightsAnalisys.DetectedColor==DC_NONE)		
		{
			printf("LightsAnalisys.DetectedColor=NONE\n");
		}
		else if (LightsAnalisys.DetectedColor==DC_YELLOW)		
		{
			printf("LightsAnalisys.DetectedColor=YELLOW\n");
		}
		if (LightsAnalisys.DetectedColor==DC_RED)		
		{
			printf("LightsAnalisys.DetectedColor=RED\n");
		}
		if (LightsAnalisys.DetectedColor==DC_GREEN)		
		{
			printf("LightsAnalisys.DetectedColor=GREEN\n");
		}
		printf("_______________________________________________\n");
	#endif

	#if IPAINT
		char str[255];	
	
		if (LightsAnalisys.DetectedColor==DC_NONE)		
			cvPutText(img1, "NO COLOR DETECTED", cvPoint(0,15),&font, CV_RGB(200,128,0));
		else if (LightsAnalisys.DetectedColor==DC_YELLOW)		
			cvPutText(img1, "YELLOW DETECTED", cvPoint(0,15),&font, CV_RGB(200,128,0));
		if (LightsAnalisys.DetectedColor==DC_RED)		
			cvPutText(img1, "RED DETECTED", cvPoint(0,15),&font, CV_RGB(200,128,0));
		if (LightsAnalisys.DetectedColor==DC_GREEN)		
			cvPutText(img1, "GREEN DETECTED", cvPoint(0,15),&font, CV_RGB(200,128,0));
		
		sprintf(str,"Red = %d",LightsAnalisys.PixelCounts.PC_RedCount);		
		cvPutText(img1, str, cvPoint(240,200),&smallfont, CV_RGB(255,0,0));
		sprintf(str,"Green = %d",LightsAnalisys.PixelCounts.PC_GreenCount);		
		cvPutText(img1, str, cvPoint(240,210),&smallfont, CV_RGB(0,255,0));
		sprintf(str,"Yellow = %d",LightsAnalisys.PixelCounts.PC_YellowCount);		
		cvPutText(img1, str, cvPoint(240,220),&smallfont, CV_RGB(255,255,0));
	#endif
}


/** @brief Used to merge the S_Filter and V_Filter images creating the SandV_Filter image.
 * 
 */
void RL_MergeSandVFilter()
{
	cvAnd( GI.Lights.V_Filter, GI.Lights.S_Filter, GI.Lights.SandV_Filter,NULL );
	vsclean(GI.Lights.SandV_Filter);
}


/** @brief Uses lots of functions to create the MAR and the Convex Hull and compares their centers. Uses this information to decide shape orientation.
 * 
 */
void RL_ShapeRecognition()
{
int line,col;
unsigned char pixelvalue;
CvPoint pt0;
int i,x=0,y=0;
int z=0;
CvPoint *a;


	
 	cvCopy(GI.Lights.SandV_Filter,workimg);

	for (line=0;line<workimg->height;line++)
	{
		for (col=0;col<workimg->width;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(workimg,line,col);	
			if (pixelvalue)
			{
				pt0=cvPoint(col,line);
				cvSeqPush(LightsAnalisys.origpts,&pt0); 
				z++;
			}
		}
	}
	
	
	x=0;y=0;
	for (i=0;i<LightsAnalisys.origpts->total;i++) //calculating the real mass center
	{
		a = (CvPoint*)cvGetSeqElem( LightsAnalisys.origpts, i );
		y+=a->y;
		x+=a->x;
	}
	
	
	RL_BuildPtsSeqFromImg();
	if (LightsAnalisys.pts->total>0)//if nothing is in SandV_Filter there is nothing to be done
	{	
	
	LightsAnalisys.OrigMassCenterFx=(float)x/(float)(i);
	LightsAnalisys.OrigMassCenterFy=(float)y/(float)(i);
	LightsAnalisys.OrigMassCenter=cvPoint(x/(i),y/(i));
	GF_DrawCross(img1,LightsAnalisys.OrigMassCenter.y,LightsAnalisys.OrigMassCenter.x,5, Color.red);		
	
	
	//RL_GetMassCenter();
	cvZero(workimg);
	
	
	


	
	
		

		RL_BuildMinimumAreaRectangle();
	
		
		RL_BuildHull();			
		
//  	cvNamedWindow("ola2",1);
//  	cvShowImage("ola2",workimg); 			
 	
	
		RL_GetMassCenter();	
		
		
		RL_CalculateObjectOrientation();	
			
		if (macro_isbetween((int)LightsAnalisys.ObjectOrientation ,OBJECTLEFTMINANGLE ,OBJECTLEFTMAXANGLE) && LightsAnalisys.OcupationPercent<MAXIMUMOCUPATIONPERCENT && LightsAnalisys.OcupationPercent > 0.75)
		{
			LightsAnalisys.DetectedShape=DS_OBJECTORIENTEDLEFT;
		}
		else if (macro_isbetween((int)LightsAnalisys.ObjectOrientation ,OBJECTUPMINANGLE ,OBJECTUPMAXANGLE) && LightsAnalisys.OcupationPercent<MAXIMUMOCUPATIONPERCENT && LightsAnalisys.OcupationPercent > 0.75)
		{
			LightsAnalisys.DetectedShape=DS_OBJECTORIENTEDUP;
		}
		else if ((macro_isbetween((int)LightsAnalisys.ObjectOrientation ,0 ,OBJECTRIGHTMINANGLE) || macro_isbetween((int)LightsAnalisys.ObjectOrientation ,OBJECTRIGHTMAXANGLE ,360)) && LightsAnalisys.OcupationPercent<MAXIMUMOCUPATIONPERCENT && LightsAnalisys.OcupationPercent > 0.75)
		{
			LightsAnalisys.DetectedShape=DS_OBJECTORIENTEDRIGHT;
		}
		else 
		{
			LightsAnalisys.DetectedShape=DS_NONE;
		}
		
					
		#if POUT
			printf("____________ShapeRecognition Display___________\n");
			printf("LightsAnalisys.pts->total=%d\n",LightsAnalisys.pts->total);
			printf("LightsAnalisys.MARectangle.Center[x,y]=(%d,%d)\n",(int)LightsAnalisys.MARectangle.Center[0],(int)LightsAnalisys.MARectangle.Center[1]);
			printf("LightsAnalisys.MassCenter(x,y)=(%g,%g)\n",LightsAnalisys.MassCenterFx,LightsAnalisys.MassCenterFy);
			printf("LightsAnalisys.OrigMassCenter(x,y)=(%g,%g)\n",LightsAnalisys.OrigMassCenterFx,LightsAnalisys.OrigMassCenterFy);
			printf("LightsAnalisys.OcupationPercent=%g\n",LightsAnalisys.OcupationPercent);
			printf("LightsAnalisys.ObjectOrientation=%g\n",LightsAnalisys.ObjectOrientation);
			if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDLEFT)
			{
				printf("Conclusion: OBJECT ORIENTED LEFT DETECTED\n");
			}
			else if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDRIGHT)
			{
				printf("Conclusion: OBJECT ORIENTED RIGHT DETECTED\n");
			}
			else if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDUP)
			{
				printf("Conclusion: OBJECT ORIENTED UP DETECTED\n");
			}
			else if (LightsAnalisys.DetectedShape==DS_NONE)
			{
				printf("Conclusion: NO SHAPE DETECTED\n");
			}
			printf("_______________________________________________\n");
		#endif
		
	}else 
	{	
		#if POUT
			printf("____________ShapeRecognition Display___________\n");
			printf("There is nothing in SandVFilter. (LightsAnalisys.pts->total=0).\nTerminating Shape Recognition.\n");
			printf("_______________________________________________\n");
		#endif
	}
	
	#if IPAINT
			if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDLEFT)
				cvPutText(img1, "OBJECT ORIENTED LEFT DETECTED", cvPoint(0,30),&font, CV_RGB(200,0,0));
			else if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDRIGHT)
				cvPutText(img1, "OBJECT ORIENTED RIGHT DETECTED", cvPoint(0,30),&font, CV_RGB(200,0,0));
			else if (LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDUP)
				cvPutText(img1, "OBJECT ORIENTED UP DETECTED", cvPoint(0,30),&font, CV_RGB(200,0,0));
			else if (LightsAnalisys.DetectedShape==DS_NONE)
				cvPutText(img1, "NO SHAPE DETECTED", cvPoint(0,30),&font, CV_RGB(200,0,0));
		#endif
	
}


/** @brief Uses the center of the minimum area rectangle and the center of the convex hull and calculates the angle of the line that unites these two points.
 * 
 */
void RL_CalculateObjectOrientation()
{
double m;
float x_0,y_0,x_1,y_1;
			
	x_0=LightsAnalisys.OrigMassCenterFx;
	y_0=LightsAnalisys.OrigMassCenterFy;
	
	x_1=LightsAnalisys.MassCenterFx;
	y_1=LightsAnalisys.MassCenterFy;
	
	m=((y_1-y_0)/(x_1-x_0));
	LightsAnalisys.ObjectOrientation=180.0+(180.0/M_PI)*(atan2((y_1-y_0),(x_1-x_0)));
	
	
	//cout << "LightsAnalisys.ObjectOrientation = " << LightsAnalisys.ObjectOrientation << endl;
	
 	if (LightsAnalisys.ObjectOrientation < 0 )
		LightsAnalisys.ObjectOrientation = 360.0 + LightsAnalisys.ObjectOrientation;

		
	//printf("m=%g\n",m);
	//printf("pt0(x,y)=(%g,%g)\n",x_0,y_0);
	//printf("pt1(x,y)=(%g,%g)\n",x_1,y_1);
}


/** @brief Calculates the mass center of the workimg.
 * 
 */
void RL_GetMassCenter()
{
unsigned char pixelvalue;
CvPoint pt0;
int i,x=0,y=0;
int z=0;
CvPoint *a;
int line,col;

	
	for (line=0;line<workimg->height;line++)
	{
		for (col=0;col<workimg->width;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(workimg,line,col);	
			if (pixelvalue)
			{
				pt0=cvPoint(col,line);
				cvSeqPush(LightsAnalisys.fillpts,&pt0); 
				z++;
			}
		}
	}
	
	
	x=0;y=0;
	for (i=0;i<LightsAnalisys.fillpts->total;i++) //calculating the real mass center
	{
		a = (CvPoint*)cvGetSeqElem( LightsAnalisys.fillpts, i );
		y+=a->y;
		x+=a->x;
	}
	
	LightsAnalisys.MassCenterFx=(float)x/(float)(i);
	LightsAnalisys.MassCenterFy=(float)y/(float)(i);
	LightsAnalisys.MassCenter=cvPoint(x/(i),y/(i));
	GF_DrawCross(img1,LightsAnalisys.MassCenter.y,LightsAnalisys.MassCenter.x,5, Color.blue);		


}

/** @brief Builds a CvSeq sequence from the SandV_Filter image. This is done because the cvMinAreaRect2 requires a CvSeq of points.
 * 
 */
void RL_BuildPtsSeqFromImg()
{
int line,col;
unsigned char pixelvalue;
CvPoint pt0;
int npts=0;

	for (line=0;line<GI.Lights.SandV_Filter->height;line++)
		{
			for (col=0;col<GI.Lights.SandV_Filter->width;col++)
			{
				pixelvalue=(unsigned char) cvGetReal2D(GI.Lights.SandV_Filter,line,col);	
				if (pixelvalue)
				{
					pt0=cvPoint(col,line);
					npts++;
					cvSeqPush(LightsAnalisys.pts,&pt0); 
				}
			}
		}
}

/** @brief Uses the cvMinAreaRect2 to calculate the minimum area rectangle. 
 * 
 */
void RL_BuildMinimumAreaRectangle()
{
CvPoint2D32f pt[4];

CvPoint boxcenter;
CvBox2D  box;	
	
	
	box=cvMinAreaRect2( LightsAnalisys.pts, NULL );
			
	cvBoxPoints( box, pt );
		
	LightsAnalisys.MARectangle.width=(int)box.size.width;
	LightsAnalisys.MARectangle.height=(int)box.size.height;
	LightsAnalisys.MARectangle.Area=LightsAnalisys.MARectangle.height*LightsAnalisys.MARectangle.width;
	
	LightsAnalisys.MARectangle.boxpts[0].x=(int)pt[0].x;
	LightsAnalisys.MARectangle.boxpts[0].y=(int)pt[0].y;
	
	LightsAnalisys.MARectangle.boxpts[1].x=(int)pt[1].x;
	LightsAnalisys.MARectangle.boxpts[1].y=(int)pt[1].y;
	
	LightsAnalisys.MARectangle.boxpts[2].x=(int)pt[2].x;
	LightsAnalisys.MARectangle.boxpts[2].y=(int)pt[2].y;
	
	LightsAnalisys.MARectangle.boxpts[3].x=(int)pt[3].x;
	LightsAnalisys.MARectangle.boxpts[3].y=(int)pt[3].y;
	
	boxcenter.x=(int)(box.center.x);
	boxcenter.y=(int) (box.center.y);
	LightsAnalisys.MARectangle.Center[0]=(box.center.x);
	LightsAnalisys.MARectangle.Center[1]=(box.center.y);
	
	
	
	
	GF_DrawCross(img1,boxcenter.y,boxcenter.x,5, Color.orange);		
	
	cvLine(img1,LightsAnalisys.MARectangle.boxpts[0],LightsAnalisys.MARectangle.boxpts[1],CV_RGB(128,128,0));
	cvLine(img1,LightsAnalisys.MARectangle.boxpts[1],LightsAnalisys.MARectangle.boxpts[2],CV_RGB(128,128,0));
	cvLine(img1,LightsAnalisys.MARectangle.boxpts[2],LightsAnalisys.MARectangle.boxpts[3],CV_RGB(128,128,0));
	cvLine(img1,LightsAnalisys.MARectangle.boxpts[3],LightsAnalisys.MARectangle.boxpts[0],CV_RGB(128,128,0));
		
}

/** @brief Uses cvConvexHull2 to get the convex hull of the SandV_Filter.
 * 
 */
void RL_BuildHull()
{
CvPoint tmp_pt0,tmp_pt1;
int i;
CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;

int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this

CvScalar a;



	LightsAnalisys.hull = cvConvexHull2( LightsAnalisys.pts, 0, CV_CLOCKWISE, 0 );
	tmp_pt0=**CV_GET_SEQ_ELEM( CvPoint*, LightsAnalisys.hull, 0 );
	for (i=1;i<LightsAnalisys.hull->total+1;i++)
	{
		tmp_pt1=**CV_GET_SEQ_ELEM( CvPoint*, LightsAnalisys.hull, i );
		cvLine(img1,tmp_pt0,tmp_pt1,CV_RGB(0,0,255));
		cvLine(workimg,tmp_pt0,tmp_pt1,cvScalar(255));
		tmp_pt0=tmp_pt1;
		
	}
	
	cvFloodFill( workimg, cvPoint( (int)LightsAnalisys.MARectangle.Center[0] ,(int)LightsAnalisys.MARectangle.Center[1]) , cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, NULL); //fill the image

	
	a= cvSum(workimg);
	

	LightsAnalisys.HullArea=(int)(a.val[0]/255);
	LightsAnalisys.OcupationPercent=(float)LightsAnalisys.HullArea/(float)LightsAnalisys.MARectangle.Area;	
	
	
	
}

/** @brief Clears all the variables of the structure LightsAnalisys. Used in every iteration.
 * 
 */
void RL_ClearLightsAnalisysStruct()
{

	cvZero(workimg);
	cvZero(GI.Lights.S_Filter);
	cvZero(GI.Lights.S_Filter);
	cvZero(GI.Lights.Red_Filter);
	cvZero(GI.Lights.Green_Filter);
	cvZero(img1);
	
		
	cvClearSeq(LightsAnalisys.pts);	
	cvClearSeq(LightsAnalisys.fillpts);	
	cvClearSeq(LightsAnalisys.origpts);	
	LightsAnalisys.DetectedColor=DC_NONE;
	LightsAnalisys.DetectedShape=DS_NONE;
	LightsAnalisys.DetectedSymbol=DSy_NONE;
	LightsAnalisys.PixelCounts.PC_GreenCount=0;
	LightsAnalisys.PixelCounts.PC_RedCount=0;
	LightsAnalisys.PixelCounts.PC_YellowCount=0;
	LightsAnalisys.PixelCounts.PC_MaxCount=0;
	LightsAnalisys.PixelCounts.PC_MinCount=0;


}

/** @brief Looks at the decisions made by the Shape and the Color functions and decides which symbol is present.
 * 
 */
void RL_COLORandShapeMerge()
{

	if (LightsAnalisys.DetectedColor==DC_YELLOW && LightsAnalisys.DetectedShape ==DS_OBJECTORIENTEDLEFT)
	{
		LightsAnalisys.DetectedSymbol=DSy_YELLOWARROWLEFT;
	}
	else if ( LightsAnalisys.DetectedColor==DC_YELLOW && LightsAnalisys.DetectedShape ==DS_OBJECTORIENTEDRIGHT)
	{
		LightsAnalisys.DetectedSymbol=DSy_YELLOWARROWRIGHT;
	}
	else if (LightsAnalisys.DetectedColor==DC_GREEN && LightsAnalisys.DetectedShape==DS_OBJECTORIENTEDUP)
	{
		LightsAnalisys.DetectedSymbol=DSy_GREENARROWUP;
	}
	else if (LightsAnalisys.DetectedColor==DC_RED && LightsAnalisys.DetectedShape==DS_NONE)
	{
		LightsAnalisys.DetectedSymbol=DSy_REDCROSS;
	}
	else
	{
		LightsAnalisys.DetectedSymbol=DSy_NONE;
	}

	#if POUT
		printf("_________Shape and Color Merging Display_______\n");
		if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWLEFT)
		{
			printf("Conclusion: YELLOW LEFT ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWRIGHT)
		{
			printf("Conclusion: YELLOW RIGHT ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_GREENARROWUP)
		{
			printf("Conclusion: GREEN UP ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_REDCROSS)
		{
			printf("Conclusion: RED CROSS DETECTED\n");	
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_NONE)
		{
			printf("Conclusion: NOTHING DETECTED\n");
		}
		printf("_______________________________________________\n");
	#endif

	#if IPAINT
		if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWLEFT)
			cvPutText(img1, "YELLOW LEFT ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWRIGHT)
			cvPutText(img1, "YELLOW RIGHT ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_GREENARROWUP)
			cvPutText(img1, "GREEN UP ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_REDCROSS)
			cvPutText(img1, "RED CROSS DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_NONE)
			cvPutText(img1, "NOTHING DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
	#endif
	
	
}


void RL_MatchShapes(void)
{
char str[255];

	
	
	//____________MATCH GAU TEMPLATE_______________________________		
	cvMatchTemplate( GI.Lights.V_Filter, LightsAnalisys.GAU_Template, LightsAnalisys.GAU_Results, CV_TM_CCORR_NORMED/*CV_TM_CCORR*/);
	cvMinMaxLoc( LightsAnalisys.GAU_Results, &LightsAnalisys.GAU_min_val, &LightsAnalisys.GAU_max_val, &LightsAnalisys.GAU_min_loc, &LightsAnalisys.GAU_max_loc, NULL );
	LightsAnalisys.GAU_max_loc.x = LightsAnalisys.GAU_max_loc.x + LightsAnalisys.GAU_TemplateMat8U->height/2 -1;
	LightsAnalisys.GAU_max_loc.y = LightsAnalisys.GAU_max_loc.y + LightsAnalisys.GAU_TemplateMat8U->width/2 -1;
	GF_DrawCross( LightsImg_Original ,LightsAnalisys.GAU_max_loc.y, LightsAnalisys.GAU_max_loc.x, 3 ,Color.green);
	sprintf(str,"GAU %f",LightsAnalisys.GAU_max_val);
	cvPutText(LightsImg_Original , str, cvPoint(LightsAnalisys.GAU_max_loc.x ,LightsAnalisys.GAU_max_loc.y), &smallfont, CV_RGB(0,255,0));	
	cvPutText(LightsImg_Original , str, cvPoint(5,10), &smallfont, CV_RGB(0,255,0));	
		
	//____________MATCH YAL TEMPLATE_______________________________		
	cvMatchTemplate( GI.Lights.V_Filter, LightsAnalisys.YAL_Template, LightsAnalisys.YAL_Results, CV_TM_CCORR_NORMED/*CV_TM_CCORR*/);
	cvMinMaxLoc( LightsAnalisys.YAL_Results, &LightsAnalisys.YAL_min_val, &LightsAnalisys.YAL_max_val, &LightsAnalisys.YAL_min_loc, &LightsAnalisys.YAL_max_loc, NULL );
	LightsAnalisys.YAL_max_loc.x = LightsAnalisys.YAL_max_loc.x + LightsAnalisys.YAL_TemplateMat8U->height/2 -1;
	LightsAnalisys.YAL_max_loc.y = LightsAnalisys.YAL_max_loc.y + LightsAnalisys.YAL_TemplateMat8U->width/2 -1;
	GF_DrawCross( LightsImg_Original ,LightsAnalisys.YAL_max_loc.y, LightsAnalisys.YAL_max_loc.x, 3 ,Color.yellow);
	sprintf(str,"YAL %f",LightsAnalisys.YAL_max_val);
	cvPutText(LightsImg_Original , str, cvPoint(LightsAnalisys.YAL_max_loc.x ,LightsAnalisys.YAL_max_loc.y), &smallfont, CV_RGB(255,255,0));	
	cvPutText(LightsImg_Original , str, cvPoint(5,20), &smallfont, CV_RGB(255,255,0));	
	
	//____________MATCH YAR TEMPLATE_______________________________		
	cvMatchTemplate( GI.Lights.V_Filter, LightsAnalisys.YAR_Template, LightsAnalisys.YAR_Results, CV_TM_CCORR_NORMED/*CV_TM_CCORR*/);
	cvMinMaxLoc( LightsAnalisys.YAR_Results, &LightsAnalisys.YAR_min_val, &LightsAnalisys.YAR_max_val, &LightsAnalisys.YAR_min_loc, &LightsAnalisys.YAR_max_loc, NULL );
	LightsAnalisys.YAR_max_loc.x = LightsAnalisys.YAR_max_loc.x + LightsAnalisys.YAR_TemplateMat8U->height/2 -1;
	LightsAnalisys.YAR_max_loc.y = LightsAnalisys.YAR_max_loc.y + LightsAnalisys.YAR_TemplateMat8U->width/2 -1;
	GF_DrawCross( LightsImg_Original ,LightsAnalisys.YAR_max_loc.y, LightsAnalisys.YAR_max_loc.x, 3 ,Color.yellow);
	sprintf(str,"YAR %f",LightsAnalisys.YAR_max_val);
	cvPutText(LightsImg_Original , str, cvPoint(LightsAnalisys.YAR_max_loc.x ,LightsAnalisys.YAR_max_loc.y), &smallfont, CV_RGB(255,255,0));
	cvPutText(LightsImg_Original , str, cvPoint(5,30), &smallfont, CV_RGB(255,255,0));
	
	//____________MATCH RC TEMPLATE_______________________________		
	cvMatchTemplate( GI.Lights.V_Filter, LightsAnalisys.RC_Template, LightsAnalisys.RC_Results, CV_TM_CCORR_NORMED/*CV_TM_CCORR*/);
	cvMinMaxLoc( LightsAnalisys.RC_Results, &LightsAnalisys.RC_min_val, &LightsAnalisys.RC_max_val, &LightsAnalisys.RC_min_loc, &LightsAnalisys.RC_max_loc, NULL );
	LightsAnalisys.RC_max_loc.x = LightsAnalisys.RC_max_loc.x + LightsAnalisys.RC_TemplateMat8U->height/2 -1;
	LightsAnalisys.RC_max_loc.y = LightsAnalisys.RC_max_loc.y + LightsAnalisys.RC_TemplateMat8U->width/2 -1;
	GF_DrawCross( LightsImg_Original ,LightsAnalisys.RC_max_loc.y, LightsAnalisys.RC_max_loc.x, 3 ,Color.red);
	sprintf(str,"RC %f",LightsAnalisys.RC_max_val);
	cvPutText(LightsImg_Original , str, cvPoint(LightsAnalisys.RC_max_loc.x ,LightsAnalisys.RC_max_loc.y), &smallfont, CV_RGB(255,0,0));
	cvPutText(LightsImg_Original , str, cvPoint(5,40), &smallfont, CV_RGB(255,0,0));
	
	float max=0;
	
	max = macro_max(LightsAnalisys.GAU_max_val,macro_max(LightsAnalisys.YAL_max_val, macro_max(LightsAnalisys.YAR_max_val,LightsAnalisys.RC_max_val)));
	
	
	if (LightsAnalisys.YAL_max_val>0.70 && LightsAnalisys.YAL_max_val==max)
	{
		LightsAnalisys.DetectedSymbol=DSy_YELLOWARROWLEFT;
	}
	else if ( LightsAnalisys.YAR_max_val>0.70 && LightsAnalisys.YAR_max_val==max)
	{
		LightsAnalisys.DetectedSymbol=DSy_YELLOWARROWRIGHT;
	}
	else if (LightsAnalisys.GAU_max_val>0.70 && LightsAnalisys.GAU_max_val==max)
	{
		LightsAnalisys.DetectedSymbol=DSy_GREENARROWUP;
	}
	else if (LightsAnalisys.RC_max_val>0.70 && LightsAnalisys.RC_max_val==max)
	{
		LightsAnalisys.DetectedSymbol=DSy_REDCROSS;
	}
	else
	{
		LightsAnalisys.DetectedSymbol=DSy_NONE;
	}

	#if POUT
		printf("_________Shape and Color Merging Display_______\n");
		if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWLEFT)
		{
			printf("Conclusion: YELLOW LEFT ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWRIGHT)
		{
			printf("Conclusion: YELLOW RIGHT ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_GREENARROWUP)
		{
			printf("Conclusion: GREEN UP ARROW DETECTED\n");
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_REDCROSS)
		{
			printf("Conclusion: RED CROSS DETECTED\n");	
		}
		else if (LightsAnalisys.DetectedSymbol==DSy_NONE)
		{
			printf("Conclusion: NOTHING DETECTED\n");
		}
		printf("_______________________________________________\n");
	#endif

	#if IPAINT
		if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWLEFT)
			cvPutText(img1, "YELLOW LEFT ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWRIGHT)
			cvPutText(img1, "YELLOW RIGHT ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_GREENARROWUP)
			cvPutText(img1, "GREEN UP ARROW DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_REDCROSS)
			cvPutText(img1, "RED CROSS DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
		else if (LightsAnalisys.DetectedSymbol==DSy_NONE)
			cvPutText(img1, "NOTHING DETECTED", cvPoint(0,45),&font, CV_RGB(255,0,128));
	#endif
	
}

#endif
