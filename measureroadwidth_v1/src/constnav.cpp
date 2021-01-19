/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: constnav.c******************************************/
/***************************************************************/

#ifndef _CONSTNAV_
#define _CONSTNAV_

#include "header.h"

#if USECONSTRUCTIONMODE
void CN_FindPins(void)
{
int line,col;
unsigned int pixelvalue;


	cvCvtColor(GI.Orig.RoadImg,GI.Const.HSV,CV_RGB2HSV);	
	
	
	cvRectangle(GI.Const.HSV,cvPoint(0,RobotStatus.Horizon-1),cvPoint(GI.Const.HSV->width,0),CV_RGB(0,0,0),-1);
	cvCvtPixToPlane(GI.Const.HSV,GI.Const.H,GI.Const.S,GI.Const.V,0);		
	
	
	
	
	
	
	
	/*cvNamedWindow("Construction_HSV",1);
	cvMoveWindow("Construction_HSV", 300, 300 );
	cvNamedWindow("RoadImg_H",1);
	cvMoveWindow("RoadImg_H", 300, 300 );
	cvNamedWindow("RoadImg_S",1);
	cvMoveWindow("RoadImg_S", 300, 300 );
	cvNamedWindow("RoadImg_V",1);
	cvMoveWindow("RoadImg_V", 300, 300 );
	cvNamedWindow("S_Filter",1);
	cvMoveWindow("S_Filter", 300, 300 );
	cvNamedWindow("V_Filter",1);
	cvMoveWindow("V_Filter", 300, 300 );
	cvNamedWindow("SandV_Filter",1);
	cvMoveWindow("SandV_Filter", 300, 300 );
	*/
	
	//cvShowImage("Red_Filter",Yellow_Filter);
	
	//cvWaitKey(0);
	//cvErode( SandV_Filter, SandV_Filter, NULL, 1 );
	/*cvShowImage("Construction_HSV",Construction_HSV);
	cvShowImage("RoadImg_H",RoadImg_H);
	cvShowImage("RoadImg_S",RoadImg_S);
	cvShowImage("RoadImg_V",RoadImg_V);
	
	*/
	//cvShowImage("S_Filter",S_Filter);
	//cvShowImage("V_Filter",V_Filter);

}

void CN_DrawPins(void)
{
int line,col;
unsigned int pixelvalue;


// 	for (line=0;line<SandV_Filter->height;line++)
// 	{
// 		for (col=0;col<SandV_Filter->width;col++)
// 		{
// 		
// 			pixelvalue=(unsigned int) cvGetReal2D( SandV_Filter, line, col );	
// 			
// 			if (pixelvalue)
// 			{
// 			
// 			cvLine(RoadImg_rgb_hough,cvPoint(col,line),cvPoint(col,line),CV_RGB(255,255,0));
// 			
// 			
// 			}
// 		
// 		}
// 	}






}

void CN_DetectPins(void)
{
CvRect rect;
rect.x=0;
rect.y=CONSTHORIZON+100;
rect.width=320;
rect.height=240;
	
/*	cvSetImageROI(SandV_Filter,rect );
	RobotStatus.SandV_sumDown=cvSum(SandV_Filter);
	RobotStatus.SandV_sumDown.val[0]=RobotStatus.SandV_sumDown.val[0]/255;
	
	if ((unsigned int)RobotStatus.SandV_sumDown.val[0]>PIXELSTOGOTOCONSTMODE)
		RobotStatus.PinsDetectedDown=1;
	
		
	
	rect.y=0;	
	cvSetImageROI(SandV_Filter,rect );
	RobotStatus.SandV_sumAll=cvSum(SandV_Filter);
	RobotStatus.SandV_sumAll.val[0]=RobotStatus.SandV_sumAll.val[0]/255;
	
	if ((unsigned int)RobotStatus.SandV_sumAll.val[0]>PIXELSTOGOTOCONSTMODE)
		RobotStatus.PinsDetectedAll=1;
	

		
	#if IPAINT
	char str[255];
	
		sprintf(str,"YP Down=%d",(int)RobotStatus.SandV_sumDown.val[0]);
		cvPutText(GI.Data.Info, str, cvPoint(0,150),&font, CV_RGB(200,30,180));
		sprintf(str,"YP All=%d",(int)RobotStatus.SandV_sumAll.val[0]);
		cvPutText(GI.Data.Info, str, cvPoint(0,165),&font, CV_RGB(200,30,180));*/
	//#endif
}


//Finds first black pixel after a white is found.(Searches in a given image colum) (Look from outside)
void CN_FindPixelsLFO(IplImage *src,int low_line,int top_line)
{
int line,col;
char continuesearch,blackfound;
unsigned char pixelvalue ;
//printf("src->height-1=*d\n",src->height-1);

	blackfound=0;
	continuesearch=1;
	for (line=low_line;line>top_line;line--)  //First look for low  left corner
	{
		//printf("******** LINE=%d **********\n",line);
		for (col=2;col<(src->width-1)/3/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=%d | col=%d | value=%d\n",line, col,pixelvalue) ;
			
				
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				SquareRegion.OSearch.lowl.val[0]=line;
				SquareRegion.OSearch.lowl.val[1]=col;
				SquareRegion.OSearch.lowl.valid=1;
				break;
			}
		
			if (!blackfound && col>COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;
	}	
	
	if (SquareRegion.OSearch.lowl.valid==0) //if could not find a good pix from low line look from bottom
	{
		blackfound=0;
		continuesearch=1;
		for (line=239;line>low_line;line--)  //First look for low  left corner from the bottom
		{
			//printf("******** LINE=%d **********\n",line);
			for (col=2;col<(src->width-1)/3/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
			{	
				pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
				//printf("line=%d | col=%d | value=%d\n",line, col,pixelvalue) ;
				
					
				if (!blackfound && !pixelvalue) blackfound=1;
				
				if (pixelvalue && blackfound)
				{
					continuesearch=0;
					SquareRegion.OSearch.lowl.val[0]=line;
					SquareRegion.OSearch.lowl.val[1]=col;
					SquareRegion.OSearch.lowl.valid=1;
					break;
				}
			
				if (!blackfound && col>COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
					break;
			}
			
			if (!continuesearch)
				break;	
			blackfound=0;
		}
	}
	
	blackfound=0;
	continuesearch=1;
	for (line=top_line;line<low_line;line++)  // look for high  left corner
	{
		//printf("******** LINE=*d **********\n",line);
		for (col=2;col<(src->width-1)/3/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				SquareRegion.OSearch.topl.val[0]=line;
				SquareRegion.OSearch.topl.val[1]=col;
				SquareRegion.OSearch.topl.valid=1;
				break;
			}
			
			if (!blackfound && col>COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
			
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;
	}	
	
	
	blackfound=0;
	continuesearch=1;
	for (line=low_line;line>top_line;line--)  // look for low  right corner
	{
		
		/*because previous image treatment has blacked col=img.width-1*/
		for (col=src->width-1-1;col>2*(src->width-1)/3/*col>((src->width-1)/2)*/ ;col--)
		{	
			
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				SquareRegion.OSearch.lowr.val[0]=line;
				SquareRegion.OSearch.lowr.val[1]=col;
				SquareRegion.OSearch.lowr.valid=1;
				break;
			}
			
			if (!blackfound && col<src->width-COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;
	}
	
			
	
	if (SquareRegion.OSearch.lowr.valid==0) //if could not find a good pix from low line look from bottom
	{
		blackfound=0;
		continuesearch=1;
		for (line=238;line>low_line;line--)  // look for low  right corner
		{
			//printf("******** LINE=*d **********\n",line);
			/*because previous image treatment has blacked col=img.width-1*/
			for (col=src->width-1-1;col>2*(src->width-1)/3/*col>((src->width-1)/2)*/ ;col--)
			{	
				pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
				//printf("line=%d | col=%d | value=%d\n",line, col,pixelvalue) ;
				
				if (!blackfound && !pixelvalue) blackfound=1;
				
				if (pixelvalue && blackfound)
				{
					continuesearch=0;
					SquareRegion.OSearch.lowr.val[0]=line;
					SquareRegion.OSearch.lowr.val[1]=col;
					SquareRegion.OSearch.lowr.valid=1;
					break;
				}
				
				if (!blackfound && col<src->width-COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
					break;
			}
			
			if (!continuesearch)
				break;	
			blackfound=0;
		}
	
	
	
	
	
	
	
	
	}
	
	blackfound=0;
	continuesearch=1;
	for (line=top_line;line<low_line;line++)  // look for top  right corner
	{
		//printf("******** LINE=*d **********\n",line);
		/*because previous image treatment has blacked col=img.width-1*/
		for (col=src->width-1-5;col>2*(src->width-1)/3/*col>((src->width-1)/2)*/ ;col--)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=%d | col=%d | value=%d",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			//printf("blackfound=%d\n",blackfound) ;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				SquareRegion.OSearch.topr.val[0]=line;
				SquareRegion.OSearch.topr.val[1]=col;
				SquareRegion.OSearch.topr.valid=1;
				break;
			}
		
			if (!blackfound && col<src->width-COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
			
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;	
		
	}
	
	
	#if NPOUT
		
		printf("\n***** Box Outside Search Results *******\n");
		printf("****** Left [x,y,valid]**** Right [x,y,valid]***\n");
		printf("* Top       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.OSearch.topl.val[0],SquareRegion.OSearch.topl.val[1] , SquareRegion.OSearch.topl.valid, SquareRegion.OSearch.topr.val[0], SquareRegion.OSearch.topr.val[1], SquareRegion.OSearch.topr.valid);
		printf("* Low       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.OSearch.lowl.val[0],SquareRegion.OSearch.lowl.val[1] , SquareRegion.OSearch.lowl.valid, SquareRegion.OSearch.lowr.val[0], SquareRegion.OSearch.lowr.val[1], SquareRegion.OSearch.lowr.valid);
		printf("*** Box Outside Search Results END*****\n\n");
		
	#endif
	
	//cvWaitKey(0);
	
}
#endif


#endif


