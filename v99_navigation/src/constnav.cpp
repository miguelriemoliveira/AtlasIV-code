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
			cvCvtColor(GI.Orig.RoadImg,GI.Const.HSV,CV_RGB2HSV);	
			cvRectangle(GI.Const.HSV,cvPoint(0,RobotStatus.Horizon-1),cvPoint(GI.Const.HSV->width,0),CV_RGB(0,0,0),-1);
			cvCvtPixToPlane(GI.Const.HSV,GI.Const.H,GI.Const.S,GI.Const.V,0);		
			cvCmpS( GI.Const.S, (double) CONSTSATURATIONLIMIT, GI.Const.S_Filter, CV_CMP_GT);
			cvCmpS( GI.Const.V, (double) CONSTINTENSITYLIMIT, GI.Const.V_Filter, CV_CMP_GT);
			cvInRangeS( GI.Const.H, cvScalar(CONSTLOWORANGELIMIT) ,cvScalar(CONSTTOPORANGELIMIT) , GI.Const.Orange);
			cvAnd(GI.Const.S_Filter, GI.Const.V_Filter, GI.Const.SandV_Filter);
			cvAnd(GI.Const.Orange, GI.Const.SandV_Filter, GI.Const.Pins);
			CleanIsolatedPoints(GI.Const.Pins, GI.Const.Pins, 1);
			cvDilate(GI.Const.Pins, GI.Const.Pins,NULL, 8);
			cvErode(GI.Const.Pins, GI.Const.Pins,NULL, 3);	

			RobotStatus.PinsPixNumAll=  cvCountNonZero( GI.Const.Pins);
			
			cvSetImageROI( GI.Const.Pins, cvRect(0, 180, (int)ImagesParams.NavImgSize.width, ImagesParams.NavImgSize.height-180));
			RobotStatus.PinsPixNumDown =  cvCountNonZero( GI.Const.Pins);
			cvResetImageROI(GI.Const.Pins);
			
			if ( RobotStatus.PinsPixNumAll > PIXELSTOGOTOCONSTMODE )
				RobotStatus.PinsDetectedAll=1;
			if ( RobotStatus.PinsPixNumDown > PIXELSTOEXITCONSTMODE )
				RobotStatus.PinsDetectedDown=1;


}

void CN_DrawPins(void)
{
	cvSet( RoadImg_rgb_hough, CV_RGB(255,128,0), GI.Const.Pins);
	if (ConstSquareRegion.ValidTopGoToPoint)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.TopGoToPoint[1] ,ConstSquareRegion.TopGoToPoint[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.ValidLowGoToPoint)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.LowGoToPoint[1] ,ConstSquareRegion.LowGoToPoint[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.ValidLowLeftGoToPoint)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.LowLeftGoToPoint[1] ,ConstSquareRegion.LowLeftGoToPoint[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.ValidLowRightGoToPoint)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.LowRightGoToPoint[1] ,ConstSquareRegion.LowRightGoToPoint[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.OSearch.lowl.valid)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.OSearch.lowl.val[1] ,ConstSquareRegion.OSearch.lowl.val[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.OSearch.lowr.valid)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.OSearch.lowr.val[1] ,ConstSquareRegion.OSearch.lowr.val[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.OSearch.topr.valid)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.OSearch.topr.val[1] ,ConstSquareRegion.OSearch.topr.val[0]) ,&smallfont ,CV_RGB(0,0,255));
		if (ConstSquareRegion.OSearch.topl.valid)
			cvPutText(RoadImg_rgb_hough, "CN", cvPoint(ConstSquareRegion.OSearch.topl.val[1] ,ConstSquareRegion.OSearch.topl.val[0]) ,&smallfont ,CV_RGB(0,0,255));
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


	blackfound=0;
	continuesearch=1;
	for (line=low_line;line>top_line;line--)  //First look for low  left corner
	{
		//printf("******** LINE=%d **********\n",line);
		for (col=2;/*(col<2*(src->width-1)/3)*/(col<(src->width-1)/3)/*because previous image treatment has blacked col=0*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=%d | col=%d | value=%d\n",line, col,pixelvalue) ;
			
				
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				ConstSquareRegion.OSearch.lowl.val[0]=line;
				ConstSquareRegion.OSearch.lowl.val[1]=col;
				ConstSquareRegion.OSearch.lowl.valid=1;
				break;
			}
		
			if (!blackfound && col>COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;
	}	
	
	if (ConstSquareRegion.OSearch.lowl.valid==0) //if could not find a good pix from low line look from bottom
	{
		blackfound=0;
		continuesearch=1;
		for (line=239;line>low_line;line--)  //First look for low  left corner from the bottom
		{
			//printf("******** LINE=%d **********\n",line);
			for (col=2;/*(col<2*(src->width-1)/3)*/(col<(src->width-1)/2)/*because previous image treatment has blacked col=0*/ ;col++)
			{	
				pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
				//printf("line=%d | col=%d | value=%d\n",line, col,pixelvalue) ;
				
					
				if (!blackfound && !pixelvalue) blackfound=1;
				
				if (pixelvalue && blackfound)
				{
					continuesearch=0;
					ConstSquareRegion.OSearch.lowl.val[0]=line;
					ConstSquareRegion.OSearch.lowl.val[1]=col;
					ConstSquareRegion.OSearch.lowl.valid=1;
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
		for (col=2;/*(col<2*(src->width-1)/3)*/(col<(src->width-1)/3)/*because previous image treatment has blacked col=0*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				ConstSquareRegion.OSearch.topl.val[0]=line;
				ConstSquareRegion.OSearch.topl.val[1]=col;
				ConstSquareRegion.OSearch.topl.valid=1;
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
		//printf("******** LINE=*d **********\n",line);
		/*because previous image treatment has blacked col=img.width-1*/
		for (col=src->width-1-1;/*(col>(src->width-1)/3)*/col>(2*(src->width-1)/3) ;col--)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				ConstSquareRegion.OSearch.lowr.val[0]=line;
				ConstSquareRegion.OSearch.lowr.val[1]=col;
				ConstSquareRegion.OSearch.lowr.valid=1;
				break;
			}
			
			if (!blackfound && col<src->width-COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;
	}
	
	if (ConstSquareRegion.OSearch.lowr.valid==0) //if could not find a good pix from low line look from bottom
	{
		blackfound=0;
		continuesearch=1;
		for (line=238;line>low_line;line--)  // look for low  right corner
		{
			//printf("******** LINE=*d **********\n",line);
			/*because previous image treatment has blacked col=img.width-1*/
			for (col=src->width-1-1;/*(col>(src->width-1)/3)*/col>((src->width-1)/2) ;col--)
			{	
				pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
				//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
				
				if (!blackfound && !pixelvalue) blackfound=1;
				
				if (pixelvalue && blackfound)
				{
					continuesearch=0;
					ConstSquareRegion.OSearch.lowr.val[0]=line;
					ConstSquareRegion.OSearch.lowr.val[1]=col;
					ConstSquareRegion.OSearch.lowr.valid=1;
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
		for (col=src->width-1-5;/*(col>(src->width-1)/3)*/col>(2*(src->width-1)/3) ;col--)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=%d | col=%d | value=%d",line, col,pixelvalue) ;
			
			if (!blackfound && !pixelvalue) blackfound=1;
			
			//printf("blackfound=%d\n",blackfound) ;
			
			if (pixelvalue && blackfound)
			{
				continuesearch=0;
				ConstSquareRegion.OSearch.topr.val[0]=line;
				ConstSquareRegion.OSearch.topr.val[1]=col;
				ConstSquareRegion.OSearch.topr.valid=1;
				break;
			}
		
			if (!blackfound && col<src->width-COLSTOFINDABLACKPIX) //breaks if does not find blask in the first 30 cols
				break;
			
		}
		
		if (!continuesearch)
			break;	
		blackfound=0;	
		
	}
	
	
	#if POUT
		
		printf("\n***** Box Outside Search Results *******\n");
		printf("****** Left [x,y,valid]**** Right [x,y,valid]***\n");
		printf("* Top       [%d,%d,%d]            [%d,%d,%d]\n", ConstSquareRegion.OSearch.topl.val[0],ConstSquareRegion.OSearch.topl.val[1] , ConstSquareRegion.OSearch.topl.valid, ConstSquareRegion.OSearch.topr.val[0], ConstSquareRegion.OSearch.topr.val[1], ConstSquareRegion.OSearch.topr.valid);
		printf("* Low       [%d,%d,%d]            [%d,%d,%d]\n", ConstSquareRegion.OSearch.lowl.val[0],ConstSquareRegion.OSearch.lowl.val[1] , ConstSquareRegion.OSearch.lowl.valid, ConstSquareRegion.OSearch.lowr.val[0], ConstSquareRegion.OSearch.lowr.val[1], ConstSquareRegion.OSearch.lowr.valid);
		printf("*** Box Outside Search Results END*****\n\n");
		
	#endif
	
}	
	


void CN_GetGoToPoint(int *TLN,int *TRW)
{
int i;
int cont1=0;
int pixelval;


	//______________________________if both low points are valid___________________________________
	if(ConstSquareRegion.OSearch.lowl.valid && ConstSquareRegion.OSearch.lowr.valid)
	{
	
	
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(ConstSquareRegion.OSearch.lowl.val[0], ConstSquareRegion.OSearch.lowr.val[0]))
			break;	
	
		ConstSquareRegion.Boxlow.lenght=(int) sqrt((ConstSquareRegion.OSearch.lowl.val[0]-ConstSquareRegion.OSearch.lowr.val[0]) *(ConstSquareRegion.OSearch.lowl.val[0]-ConstSquareRegion.OSearch.lowr.val[0]) + (ConstSquareRegion.OSearch.lowl.val[1]- ConstSquareRegion.OSearch.lowr.val[1]) * (ConstSquareRegion.OSearch.lowl.val[1]-ConstSquareRegion.OSearch.lowr.val[1])); //calculate lenght
	
		if (ConstSquareRegion.OSearch.lowl.val[0]>ConstSquareRegion.OSearch.lowr.val[0]) //then left corner is under the right corner
		{
			ConstSquareRegion.LowGoToPoint[1]=ConstSquareRegion.OSearch.lowl.val[1]+ConstSquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			ConstSquareRegion.LowGoToPoint[0]=ConstSquareRegion.OSearch.lowl.val[0]; //x coords of lowgotopoint
		}
		else
		{
			ConstSquareRegion.LowGoToPoint[1]=ConstSquareRegion.OSearch.lowr.val[1]-ConstSquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			ConstSquareRegion.LowGoToPoint[0]=ConstSquareRegion.OSearch.lowr.val[0]; //x coords of lowgotopoint
		}
		ConstSquareRegion.ValidLowGoToPoint=1;
	
		
		
		ConstSquareRegion.LowGoToPoint[0] = saturatemin(ConstSquareRegion.LowGoToPoint[0],0);
		ConstSquareRegion.LowGoToPoint[0] = saturatemax(ConstSquareRegion.LowGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		ConstSquareRegion.LowGoToPoint[1] = saturatemin(ConstSquareRegion.LowGoToPoint[1],0);
		ConstSquareRegion.LowGoToPoint[1] = saturatemax(ConstSquareRegion.LowGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		//ConstSquareRegion.LowGoToPoint[1]
		if (ConstSquareRegion.ValidLowGoToPoint) //test for point relocation
		{
			cont1=0;
			do 
			{
				if (ConstSquareRegion.LowGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowGoToPoint[0] ,ConstSquareRegion.LowGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			
			int cont2=0;
			do 
			{
				if (ConstSquareRegion.LowGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				//cout << "ConstSquareRegion.LowGoToPoint[0]= " << ConstSquareRegion.LowGoToPoint[0] << endl;
				//cout << "ConstSquareRegion.LowGoToPoint[1]-cont2= " << ConstSquareRegion.LowGoToPoint[1]+cont2 << endl;
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowGoToPoint[0] ,ConstSquareRegion.LowGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			
			
			ConstSquareRegion.LowGoToPointWidth = cont2 + cont1;
			
// 			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /2)
// 			{	
// 				ConstSquareRegion.RelocatedLowGoToPoint=1;
// 				ConstSquareRegion.LowGoToPoint[1] = (ConstSquareRegion.LowGoToPoint[1] - cont1) + (ConstSquareRegion.LowGoToPointWidth/2);
// // 				if (ConstSquareRegion.LowGoToPoint[0] < 239-80)
// // 					ConstSquareRegion.LowGoToPoint[0] = ConstSquareRegion.LowGoToPoint[0] +80;
// 			}
// 			else if (ConstSquareRegion.LowGoToPointWidth > MAXADMISSIBLEROADWIDTH * TRW[i])
// 			{
// 				ConstSquareRegion.ValidLowGoToPoint = 0;
// 				ConstSquareRegion.MaxRoadWidthExcededLowGoToPoint = 1;
// 				//cout << "MAXADMISSIBLEROADWIDTH Exceded " << endl;
// 			}
	
		}
	}
	//_____________________________________________________________________________________
	
	//___________________________________if both top points are valid______________________
	if(ConstSquareRegion.OSearch.topl.valid && ConstSquareRegion.OSearch.topr.valid)
	{
		
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(ConstSquareRegion.OSearch.topl.val[0], ConstSquareRegion.OSearch.topr.val[0]))
				break;
	
		ConstSquareRegion.Boxtop.lenght=(int) sqrt((ConstSquareRegion.OSearch.topl.val[0]-ConstSquareRegion.OSearch.topr.val[0]) *(ConstSquareRegion.OSearch.topl.val[0]-ConstSquareRegion.OSearch.topr.val[0]) + (ConstSquareRegion.OSearch.topl.val[1]- ConstSquareRegion.OSearch.topr.val[1]) * (ConstSquareRegion.OSearch.topl.val[1]-ConstSquareRegion.OSearch.topr.val[1])); //calculate lenght
		
		if (ConstSquareRegion.OSearch.topl.val[0]>ConstSquareRegion.OSearch.topr.val[0]) //then left corner is under the right corner
		{
			ConstSquareRegion.TopGoToPoint[1]=ConstSquareRegion.OSearch.topl.val[1]+ConstSquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			ConstSquareRegion.TopGoToPoint[0]=ConstSquareRegion.OSearch.topl.val[0]; //x coords of lowgotopoint
		}
		else
		{
			ConstSquareRegion.TopGoToPoint[1]=ConstSquareRegion.OSearch.topr.val[1]-ConstSquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			ConstSquareRegion.TopGoToPoint[0]=ConstSquareRegion.OSearch.topr.val[0]; //x coords of lowgotopoint
			
		}
		ConstSquareRegion.ValidTopGoToPoint=1;
		
		ConstSquareRegion.TopGoToPoint[0] = saturatemin(ConstSquareRegion.TopGoToPoint[0],0);
		ConstSquareRegion.TopGoToPoint[0] = saturatemax(ConstSquareRegion.TopGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		ConstSquareRegion.TopGoToPoint[1] = saturatemin(ConstSquareRegion.TopGoToPoint[1],0);
		ConstSquareRegion.TopGoToPoint[1] = saturatemax(ConstSquareRegion.TopGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		if (ConstSquareRegion.ValidTopGoToPoint)//test for relocation
		{
			cont1=0;
			do 
			{
				if (ConstSquareRegion.TopGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.TopGoToPoint[0] ,ConstSquareRegion.TopGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (ConstSquareRegion.TopGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.TopGoToPoint[0] ,ConstSquareRegion.TopGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			ConstSquareRegion.TopGoToPointWidth = cont2 + cont1;
			
			//Relocation condition
// 			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /*/2*/)
// 			{	
// 				ConstSquareRegion.RelocatedTopGoToPoint=1;
// 				ConstSquareRegion.TopGoToPoint[1] = (ConstSquareRegion.TopGoToPoint[1] - cont1) + (ConstSquareRegion.TopGoToPointWidth/2);
// 				if (ConstSquareRegion.TopGoToPoint[0] < 239-80)
// 					ConstSquareRegion.TopGoToPoint[0] = ConstSquareRegion.TopGoToPoint[0] +80;
// 				else 
// 					ConstSquareRegion.TopGoToPoint[0]=239;
// 			}
// 			else if (ConstSquareRegion.TopGoToPointWidth > MAXADMISSIBLEROADWIDTH * TRW[i] || ConstSquareRegion.TopGoToPointWidth > ConstSquareRegion.LowGoToPointWidth )
// 			{
// 				ConstSquareRegion.ValidTopGoToPoint = 0;
// 				ConstSquareRegion.MaxRoadWidthExcededTopGoToPoint = 1;
// 				//cout << "MAXADMISSIBLEROADWIDTH Exceded " << endl;
// 			}
		}
	}
	//__________________________________________________________________________________
	
	//___________________________________if lowleft point is valid______________________
	if(ConstSquareRegion.OSearch.lowl.valid)
	{
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==ConstSquareRegion.OSearch.lowl.val[0])
				break;
				
		ConstSquareRegion.LowLeftGoToPoint[0]=ConstSquareRegion.OSearch.lowl.val[0];
 		ConstSquareRegion.LowLeftGoToPoint[1]=ConstSquareRegion.OSearch.lowl.val[1]+TRW[i]/2*2;  //tirei aqui
		//cout << "ConstSquareRegion.LowLeftGoToPoint[1] = " << ConstSquareRegion.LowLeftGoToPoint[1] << endl; 
		ConstSquareRegion.ValidLowLeftGoToPoint=1;

		
		
		ConstSquareRegion.LowLeftGoToPoint[0] = saturatemin(ConstSquareRegion.LowLeftGoToPoint[0],0);
		ConstSquareRegion.LowLeftGoToPoint[0] = saturatemax(ConstSquareRegion.LowLeftGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		ConstSquareRegion.LowLeftGoToPoint[1] = saturatemin(ConstSquareRegion.LowLeftGoToPoint[1],0);
		ConstSquareRegion.LowLeftGoToPoint[1] = saturatemax(ConstSquareRegion.LowLeftGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		if (ConstSquareRegion.ValidLowLeftGoToPoint)//test for relocation
		{
			cont1=0;
			do 
			{
				if (ConstSquareRegion.LowLeftGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowLeftGoToPoint[0] ,ConstSquareRegion.LowLeftGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (ConstSquareRegion.LowLeftGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowLeftGoToPoint[0] ,ConstSquareRegion.LowLeftGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			ConstSquareRegion.LowLeftGoToPointWidth = cont2 + cont1;
			
			//Relocation condition
// 			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /*/2*/)
// 			{	
// 				ConstSquareRegion.RelocatedLowLeftGoToPoint=1;
// 				ConstSquareRegion.LowLeftGoToPoint[1] = (ConstSquareRegion.LowLeftGoToPoint[1] - cont1) + (ConstSquareRegion.LowLeftGoToPointWidth/2);
// 				if (ConstSquareRegion.LowGoToPoint[0] < 239-30)
// 					ConstSquareRegion.LowGoToPoint[0] = ConstSquareRegion.LowGoToPoint[0] +30;
// 				else 
// 					ConstSquareRegion.LowGoToPoint[0]=239;
// 			}	
		}
	}
	//___________________________________________________________________________________
	
	//___________________________________if lowright point is valid______________________
	if(ConstSquareRegion.OSearch.lowr.valid)
	{
		
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==ConstSquareRegion.OSearch.lowr.val[0])
				break;
		
		ConstSquareRegion.LowRightGoToPoint[0]=ConstSquareRegion.OSearch.lowr.val[0];
		//cout << "ConstSquareRegion.OSearch.lowr.val[1] = " << ConstSquareRegion.OSearch.lowr.val[1] << endl;
		ConstSquareRegion.LowRightGoToPoint[1]=ConstSquareRegion.OSearch.lowr.val[1]-TRW[i]/2*2; //tirei aqui
		//cout << "ConstSquareRegion.LowRightGoToPoint[1]= " << ConstSquareRegion.LowRightGoToPoint[1] << endl;
		ConstSquareRegion.ValidLowRightGoToPoint=1;
		
		
		
		ConstSquareRegion.LowRightGoToPoint[0] = saturatemin(ConstSquareRegion.LowRightGoToPoint[0],0);
		ConstSquareRegion.LowRightGoToPoint[0] = saturatemax(ConstSquareRegion.LowRightGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		ConstSquareRegion.LowRightGoToPoint[1] = saturatemin(ConstSquareRegion.LowRightGoToPoint[1],0);
		ConstSquareRegion.LowRightGoToPoint[1] = saturatemax(ConstSquareRegion.LowRightGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		if (ConstSquareRegion.ValidLowRightGoToPoint)//test for relocation
		{
			cont1=0;
			do 
			{
				if (ConstSquareRegion.LowRightGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowRightGoToPoint[0] ,ConstSquareRegion.LowRightGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (ConstSquareRegion.LowRightGoToPoint[1]+cont2 >= mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, ConstSquareRegion.LowRightGoToPoint[0] ,ConstSquareRegion.LowRightGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			ConstSquareRegion.LowRightGoToPointWidth = cont2 + cont1;
			
			//Relocation condition
// 			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /2)
// 			{	
// 				ConstSquareRegion.RelocatedLowRightGoToPoint=1;
// 				ConstSquareRegion.LowRightGoToPoint[1] = (ConstSquareRegion.LowRightGoToPoint[1] - cont1) + (ConstSquareRegion.LowRightGoToPointWidth/2);
// 			}	
		}
	}
	
	
	
	//______________________________________________________________________________________
	
	#if POUT
		printf("___________________ConstGoToPoints__________________*\n");
		printf("Both Top (x,y,valid?)=[%d,%d,%d]\n", ConstSquareRegion.TopGoToPoint[0],ConstSquareRegion.TopGoToPoint[1],ConstSquareRegion.ValidTopGoToPoint);
		if (ConstSquareRegion.RelocatedTopGoToPoint)
			cout << "Both Top RELOCATED." << endl;
		if (ConstSquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Top INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Both Low (x,y,valid?)=[%d,%d,%d]\n", ConstSquareRegion.LowGoToPoint[0],ConstSquareRegion.LowGoToPoint[1],ConstSquareRegion.ValidLowGoToPoint);
		if (ConstSquareRegion.RelocatedLowGoToPoint)
			cout << "Both Low RELOCATED." << endl;
		if (ConstSquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Low INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Single LowLeft (x,y,valid?) =  [%d,%d,%d] \n",ConstSquareRegion.LowLeftGoToPoint[0] , ConstSquareRegion.LowLeftGoToPoint[1] , ConstSquareRegion.ValidLowLeftGoToPoint);
		if (ConstSquareRegion.RelocatedLowLeftGoToPoint)
			cout << "LowLeft RELOCATED." << endl;
		printf("Single LowRight (x,y,valid?) = [%d,%d,%d] \n" , ConstSquareRegion.LowRightGoToPoint[0],ConstSquareRegion.LowRightGoToPoint[1],ConstSquareRegion.ValidLowRightGoToPoint);
		if (ConstSquareRegion.RelocatedLowRightGoToPoint)
			cout << "Low Right RELOCATED." << endl;
		printf("Both Left (x,y,valid?) =  [%d,%d,%d] \n",ConstSquareRegion.BothLeftGoToPoint[0] , ConstSquareRegion.BothLeftGoToPoint[1] , ConstSquareRegion.ValidBothLeftGoToPoint);
		printf("Both Right (x,y,valid?) = [%d,%d,%d] \n" , ConstSquareRegion.BothRightGoToPoint[0],ConstSquareRegion.BothRightGoToPoint[1],ConstSquareRegion.ValidBothRightGoToPoint);
		printf("_______________________________________________\n\n");
	#endif

}


void CN_GetDAbyPP(void)
{
float x0,y0,x1,y1,yc,r,d=131;	



	if(ConstSquareRegion.ValidLowGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.LowGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.LowGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.BothLow[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.BothLow[0]<0)
			RobotStatus.ConstDA_PP.BothLow[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.BothLow[0])))/2;
		else
			RobotStatus.ConstDA_PP.BothLow[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.BothLow[0])))/2;
		RobotStatus.ConstDA_PP.BothLow[1]=1;
		
	}	
	if(ConstSquareRegion.ValidTopGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.TopGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.TopGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.BothTop[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.BothTop[0]<0)
			RobotStatus.ConstDA_PP.BothTop[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.BothTop[0])))/2;
		else
			RobotStatus.ConstDA_PP.BothTop[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.BothTop[0])))/2;
		
		RobotStatus.ConstDA_PP.BothTop[1]=1;
		
	}
	if(ConstSquareRegion.ValidLowLeftGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.LowLeftGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.LowLeftGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.LowLeft[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.LowLeft[0]<0)
			RobotStatus.ConstDA_PP.LowLeft[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.LowLeft[0])))/2;
		else
			RobotStatus.ConstDA_PP.LowLeft[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.LowLeft[0])))/2;
		RobotStatus.ConstDA_PP.LowLeft[1]=1;
		
	}
	if(ConstSquareRegion.ValidLowRightGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.LowRightGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.LowRightGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.LowRight[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.LowRight[0]<0)
			RobotStatus.ConstDA_PP.LowRight[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.LowRight[0])))/2;
		else
			RobotStatus.ConstDA_PP.LowRight[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.LowRight[0])))/2;
		RobotStatus.ConstDA_PP.LowRight[1]=1;
		
	}
	
	
	if(ConstSquareRegion.ValidBothLeftGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.BothLeftGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.BothLeftGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.BothLeft[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.BothLeft[0]<0)
			RobotStatus.ConstDA_PP.BothLeft[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.BothLeft[0])))/2;
		else
			RobotStatus.ConstDA_PP.BothLeft[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.BothLeft[0])))/2;
		RobotStatus.ConstDA_PP.BothLeft[1]=1;
		
	}	
	
	if(ConstSquareRegion.ValidBothRightGoToPoint)
	{
		x0=(float)ConstSquareRegion.PresentPoint.x;
		y0=(float)(240-ConstSquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)ConstSquareRegion.BothRightGoToPoint[1];
		y1=(float)(240-ConstSquareRegion.BothRightGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.ConstDA_PP.BothRight[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.ConstDA_PP.BothRight[0]<0)
			RobotStatus.ConstDA_PP.BothRight[0]=45-(90-(fabs(RobotStatus.ConstDA_PP.BothRight[0])))/2;
		else
			RobotStatus.ConstDA_PP.BothRight[0]=45+(90-(fabs(RobotStatus.ConstDA_PP.BothRight[0])))/2;
		RobotStatus.ConstDA_PP.BothRight[1]=1;
		
	}
	
	#if POUT
		printf("\n***********ConstDA_PP Display***************\n");
		printf("DA_PPbyBothLow(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.BothLow[0],RobotStatus.ConstDA_PP.BothLow[1]);
		printf("DA_PPbyBothTop(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.BothTop[0],RobotStatus.ConstDA_PP.BothTop[1]);
		printf("DA_PPbyLowLeft(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.LowLeft[0],RobotStatus.ConstDA_PP.LowLeft[1]);
		printf("DA_PPbyLowRight(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.LowRight[0],RobotStatus.ConstDA_PP.LowRight[1]);
		printf("DA_PPbyBothLeft(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.BothLeft[0],RobotStatus.ConstDA_PP.BothLeft[1]);
		printf("DA_PPbyBothRight(values,valid?)=(%g , %g)\n",RobotStatus.ConstDA_PP.BothRight[0],RobotStatus.ConstDA_PP.BothRight[1]);
		printf("**********DA_PP Display END************\n");
	#endif

}

#endif
#endif
