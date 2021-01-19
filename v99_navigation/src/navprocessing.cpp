/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file navprocessing.cpp
* @brief Used to process the image. Cross finding, box analysis, etc. All functions start with NAVP_.
*/
#ifndef _NAVPREPROCESSING_
#define _NAVPREPROCESSING_

#include "header.h"


/**@brief Calls every other functions nedeed to make the processing.
 * 
 */
void NAVP_NavCamProcessing()
{
// 	#if USELIGHTSREADING
// 		
// 		if (!(Mode.LastMode==STOPMODE && Time.Since.LastModeChange<TIMETOSTARTLOOKINGFORCROSS))
// 			NAVP_FindCross4(FindCross_src,RobotStatus.Horizon);
// 	#endif
// 	
// 
// 	#if !USELIGHTSREADING
// 		if((RobotStatus.LapNumber>=(TOTALLAPS-1)))	
// 			NAVP_FindCross4(FindCross_src,RobotStatus.Horizon);
// 	#endif
	
	
	NAVP_FindPixelsLFO(mask_Img1,LINETOSTARTLOWSEARCH,RobotStatus.Horizon+LINETOSTARTTOPSEARCH);
	NAVP_GetGoToPoint_1(Table_LineNumber_read,Table_RoadWidth_read);
	NAVP_GetDAbyPP1();
	NAVP_CheckLineSteping_1(mask_Img1,Table_LineNumber_read,Table_RoadWidth_read);
	
}

void NAVP_FindPixelsLFM(IplImage *src,int low_line,int top_line)
{
int line,col;
char continuesearch;
unsigned char pixelvalue ;
//printf("src->height-1=*d\n",src->height-1);

	
	continuesearch=1;
	for (line=low_line;line>top_line;line--)  //First look for low  left corner
	{
		//printf("******** LINE=*d **********\n",line);
		for (col=(src->width-1)/2;col>1/*because previous image treatment has blacked col=0*/ ;col--)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!pixelvalue)
			{
				continuesearch=0;
				SquareRegion.MSearch.lowl.val[0]=line;
				SquareRegion.MSearch.lowl.val[1]=col;
				SquareRegion.MSearch.lowl.valid=1;
				break;
			}
		
		}
		
		if (!continuesearch)
			break;	
		
	}	

	
	continuesearch=1;
	for (line=top_line;line<low_line;line++)  // look for high  left corner
	{
		//printf("******** LINE=*d **********\n",line);
		for (col=(src->width-1)/2;col>1/*because previous image treatment has blacked col=0*/ ;col--)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!pixelvalue)
			{
				continuesearch=0;
				SquareRegion.MSearch.topl.val[0]=line;
				SquareRegion.MSearch.topl.val[1]=col;
				SquareRegion.MSearch.topl.valid=1;
				break;
			}
		
		}
		
		if (!continuesearch)
			break;	
		
	}	
	
	continuesearch=1;
	for (line=low_line;line>top_line;line--)  // look for low  right corner
	{
		//printf("******** LINE=*d **********\n",line);
		for (col=((src->width-1)/2+1);col<(src->width-1-1)/*because previous image treatment has blacked col=img.width-1*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!pixelvalue)
			{
				continuesearch=0;
				SquareRegion.MSearch.lowr.val[0]=line;
				SquareRegion.MSearch.lowr.val[1]=col;
				SquareRegion.MSearch.lowr.valid=1;
				break;
			}
		
		}
		
		if (!continuesearch)
			break;	
		
	}	

	continuesearch=1;
	for (line=top_line;line<low_line;line++)  // look for top  right corner
	{
		//printf("******** LINE=*d **********\n",line);
		for (col=((src->width-1)/2+1);col<(src->width-1-1)/*because previous image treatment has blacked col=img.width-1*/ ;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			//printf("line=*d | col=*d | value=*d\n",line, col,pixelvalue) ;
			
			if (!pixelvalue)
			{
				continuesearch=0;
				SquareRegion.MSearch.topr.val[0]=line;
				SquareRegion.MSearch.topr.val[1]=col;
				SquareRegion.MSearch.topr.valid=1;
				break;
			}
		
		}
		
		if (!continuesearch)
			break;	
	}
	
	
	#if NPOUT
		
		printf("\n****** Box Middle Search Results *********\n");
		printf("****** Left [x,y,valid]**** Right [x,y,valid]***\n");
		printf("* Top       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.MSearch.topl.val[0],SquareRegion.MSearch.topl.val[1] , SquareRegion.MSearch.topl.valid, SquareRegion.MSearch.topr.val[0], SquareRegion.MSearch.topr.val[1], SquareRegion.MSearch.topr.valid);
		printf("* Low       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.MSearch.lowl.val[0],SquareRegion.MSearch.lowl.val[1] , SquareRegion.MSearch.lowl.valid, SquareRegion.MSearch.lowr.val[0], SquareRegion.MSearch.lowr.val[1], SquareRegion.MSearch.lowr.valid);
		printf("**** Box Middle Search Results END******\n\n");
		
	#endif
	
		
	
}


/**@brief Cross finding function version 0
 * 
 * @param src pointer to source image
 * @param dst pointer to destination image
 * @param hor horizon
 */
void NAVP_FindCross(IplImage *src,IplImage *dst,int hor)
{

CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(322,242),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst= cvCreateImage(cvSize(320,240),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(240,320,CV_8UC1); //same here
int whitefound=0;
int continuesearch=1;
int col,line=hor+3,i,j;
int CrossVSize=15;
unsigned char pixelvalue;
int startcol=2,endcol=src->width-5;
int notacross=0;
int a=77;
cvRectangle(pre_dst,cvPoint(0,0),cvPoint(pre_dst->width,pre_dst->height),cvScalar(0),-1); //to white the image

for (col=src->width-2;col>0;col--)//search for right limit
	{	
		pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
		
		
		if (pixelvalue)
		{	
			endcol=col;			
				break;
		}
	}

	
while(1)
{	
	
	for (col=startcol;(col<(src->width-2));col++)
	{	
		pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
		
		
		if (pixelvalue && !whitefound)
			whitefound=1;			
		else if (whitefound && !pixelvalue)
		{
			seed1_tmp.x=col;
			seed1_tmp.y=line;
			continuesearch=0;	
		}
			
		if (!continuesearch)	
		{
			startcol=col+1;
			break;
		}
		
		if (col==(src->width-3))
			startcol=1000;
	}	
		whitefound=0;			
		continuesearch=1;
		
		//printf("startcol=%d\n",startcol);
		//printf("endcol=%d\n",endcol);
		//printf("col=%d\n",col);
		
		if (startcol>=endcol)
			break;
			
		cvZero(mask1_tmp);
		cvZero(mask3_tmp);
		cvFloodFill( src, seed1_tmp, cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
		cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,320,240)); //copy threshold img to gray filled img
		cvGetImage(mask3_tmp,dst);
		
		
		notacross=0;//test to see if the cross image is not to big
		for (i=dst->height-1;i>(RobotStatus.Horizon+CrossVSize);i--)
			for (j=0;j<dst->width;j++)
			{	//printf("i=%d || j=%d || Pvalue=%d\n",i,j,(unsigned char) cvGetReal2D( dst, i, j ));
				if ((unsigned char) cvGetReal2D( dst, i, j ))
				{
					notacross=1;
					break;
				}
			}
		
		if (!notacross)
		cvOr( pre_dst, dst, pre_dst, NULL );

		
}		

	
		
	cvCopy( pre_dst, dst,NULL );
	
	
	CvScalar cross=cvSum( dst );
	
	if ((int)cross.val[0]>0)
	{
		Mode.DistantCross=CROSSFOUND;
		TM_UpdateTick(&Time.Tick.LastCross);
		a=0;	
	}
	else
	{
		Mode.DistantCross=NOCROSSFOUND;
		a=1;	
	}
	
		
	#if NPOUT
		printf("______________FindCross Display________________\n");
		if (a==0)
		printf("Mode.DistantCross=CROSSFOUND\n");
		else if (a==1)
		printf("Mode.DistantCross=NOCROSSFOUND\n");
		printf("_______________________________________________\n");
	#endif
	
	#if NIPAINT
		if (a==0)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
		else if (a==1)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: NOT FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
	#endif
}

/**@brief Cross finding function version 1
 * 
 * @param src pointer to source image
 * @param hor horizon
 */
void NAVP_FindCross1(IplImage *src,int hor)
{

CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(322,242),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst= cvCreateImage(cvSize(320,240),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst1= cvCreateImage(cvSize(320,240),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst2= cvCreateImage(cvSize(320,240),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(240,320,CV_8UC1); //same here


int col,line,i;
unsigned char pixelvalue;
int endline=0,endcol=src->width-5;
int a=77,tobreak=0;;
int itsacross=0;
float angulo1;

//cvRectangle(pre_dst,cvPoint(0,0),cvPoint(pre_dst->width,pre_dst->height),cvScalar(0),-1); //to white the image
/*cvNamedWindow("ola",1);
cvNamedWindow("ola1",1);
cvNamedWindow("ola2",1);
cvNamedWindow("ola3",1);
*/

cvZero(pre_dst);
cvZero(pre_dst1);
cvZero(pre_dst2);
cvZero(mask1_tmp);
cvZero(mask3_tmp);



cvCopy(src,pre_dst);
//cvShowImage("ola1",src);

cvLine(pre_dst,cvPoint(0,hor),cvPoint(src->width,hor),cvScalar(0));


cvFloodFill( pre_dst, cvPoint(3,3), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,320,240)); //copy threshold img to gray filled img
cvGetImage(mask3_tmp,pre_dst1);

//cvShowImage("ola",pre_dst1);

cvOr(pre_dst1,pre_dst,pre_dst2,NULL);


cvNot(pre_dst2,pre_dst2);



cvZero(pre_dst);
cvZero(pre_dst1);
cvZero(mask1_tmp);
cvZero(mask3_tmp);

int startline=pre_dst2->height-1;
for (i=0;i<4;i++)
{
	
	CvScalar total1pts=cvSum(pre_dst2);	
	int ab=(int)total1pts.val[0]/255;
	//printf("ab=%d\n",ab);
	
	if (ab<MINIMUMCROSSSIZE)
		break; 
	
	//cvShowImage("ola2",pre_dst2);
	CrossAnalisys.PixelNum=0;
	tobreak=0;
	cvZero(pre_dst);
	cvZero(pre_dst1);
	cvZero(mask1_tmp);
	cvZero(mask3_tmp);
	cvClearSeq(CrossAnalisys.pts);	

	
	for (line=startline;line>hor;line--)
	{
		for (col=15;col<pre_dst2->width-16;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(pre_dst2,line,col);	
			if (pixelvalue)
			{
				tobreak=1;
				endcol=col;
				endline=line;
				break;
			}
		}
		if (tobreak)
		{
			startline=line-1;
			break;
		}
	}
	//printf("endline=%d  endcol=%d\n",endline,endcol);
		
	cvCopy(pre_dst2,pre_dst);
	
	cvFloodFill( pre_dst, cvPoint(endcol,endline), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
	cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,320,240)); //copy threshold img to gray filled img
	cvGetImage(mask3_tmp,pre_dst1);
	
	//cvShowImage("ola",pre_dst1);
	
	CvScalar totalpts=cvSum(pre_dst1);	
	
	CrossAnalisys.PixelNum=(int)totalpts.val[0]/255;
	
	printf("PixelNum=%d\n",CrossAnalisys.PixelNum);
	//cvWaitKey(0);
	
	if (CrossAnalisys.PixelNum>MINIMUMCROSSSIZE/*to make sure its not just an isolated pix*/)
	{
		
	
	
	CvPoint pt0;
	int npts=0;
	
		for (line=0;line<pre_dst1->height;line++)
		{
			for (col=0;col<pre_dst1->width;col++)
			{
				pixelvalue=(unsigned char) cvGetReal2D(pre_dst1,line,col);	
				if (pixelvalue)
				{
					pt0=cvPoint(col,line);
					npts++;
					cvSeqPush(CrossAnalisys.pts,&pt0); 
				}
			}
		}

	
		CvPoint2D32f pt[4];
		CvBox2D box=cvMinAreaRect2( CrossAnalisys.pts, NULL );
		cvBoxPoints( box, pt );
	
		
		
		CrossAnalisys.MARectangle.width=(int)box.size.width;
		CrossAnalisys.MARectangle.height=(int)box.size.height;
		CrossAnalisys.MARectangle.Area=CrossAnalisys.MARectangle.height*CrossAnalisys.MARectangle.width;
		
		CrossAnalisys.MARectangle.boxpts[0].x=(int)pt[0].x;
		CrossAnalisys.MARectangle.boxpts[0].y=(int)pt[0].y;
		
		CrossAnalisys.MARectangle.boxpts[1].x=(int)pt[1].x;
		CrossAnalisys.MARectangle.boxpts[1].y=(int)pt[1].y;
		
		CrossAnalisys.MARectangle.boxpts[2].x=(int)pt[2].x;
		CrossAnalisys.MARectangle.boxpts[2].y=(int)pt[2].y;
		
		CrossAnalisys.MARectangle.boxpts[3].x=(int)pt[3].x;
		CrossAnalisys.MARectangle.boxpts[3].y=(int)pt[3].y;
		
		CrossAnalisys.MARectangle.Center[0]=(box.center.x);
		CrossAnalisys.MARectangle.Center[1]=(box.center.y);
		
		
	
		CrossAnalisys.HeightWidthRatio=((float)macro_min(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height))/((float)macro_max(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height));
		
		
		
		int distancia1= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)*(CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)+(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y)*(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y));
		
		int distancia2= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)*(CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)+(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y)*(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y));
		
		
		if (macro_max(distancia1,distancia2)==distancia1) //get angle from 1 and 0
		{
			
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y,CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x);
		
		}
		else  //get angle from 2 and 1
		{
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y,CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x);
		}
		
		CrossAnalisys.MARAngle=angulo1;
		
		/*printf("distancia1=%d\n",distancia1);
		printf("distancia2=%d\n",distancia2);
		printf("angulo1=%f\n",angulo1);
		
		
		printf("width do MAR=%d\n",CrossAnalisys.MARectangle.width);
		printf("height do MAR=%d\n",CrossAnalisys.MARectangle.height);
		printf("Area do MAR=%d\n",CrossAnalisys.MARectangle.Area);
		printf("HWratio=%f\n",CrossAnalisys.HeightWidthRatio);
		printf("Angle=%f\n",CrossAnalisys.MARAngle);
		*/
		//cvWaitKey(0);
		
		
		
		
		if (CrossAnalisys.HeightWidthRatio<0.25 /*&& (angulo1<30.0 && angulo1>-30.0)*/)
		{
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[0],CrossAnalisys.MARectangle.boxpts[1],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[1],CrossAnalisys.MARectangle.boxpts[2],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[2],CrossAnalisys.MARectangle.boxpts[3],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[3],CrossAnalisys.MARectangle.boxpts[0],cvScalar(128));
		itsacross=1;
		//cvShowImage("OTHER ->cross_Img",cross_Img);
			break;	
		}
		
	}
	else continue;
	
	

}
	


if (itsacross)
{
	Mode.DistantCross=CROSSFOUND;
	TM_UpdateTick(&Time.Tick.LastCross);
	a=0;	
}
else
{
	Mode.DistantCross=NOCROSSFOUND;
	a=1;	
}	
		

	#if POUT
		printf("______________FindCross Display________________\n");
		if (a==0)
		printf("Mode.Distant=CROSSFOUND\n");
		else if (a==1)
		printf("Mode.Distant=NOCROSSFOUND\n");
		printf("_______________________________________________\n");
	#endif
	
	#if IPAINT
		if (a==0)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
		else if (a==1)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: NOT FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
	#endif

}

/**@brief Cross finding function version 2. Used by the program.
 * 
 * @param src pointer to source image
 * @param hor horizon
 */
void NAVP_FindCross2(IplImage *src,int hor)
{

CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst1= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst2= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here


int col,line,i,t;
unsigned char pixelvalue;
int endline=0,endcol=src->width-5;
int a=77,tobreak=0;;
int itsacross=0;
float angulo1;

//cvRectangle(pre_dst,cvPoint(0,0),cvPoint(pre_dst->width,pre_dst->height),cvScalar(0),-1); //to white the image
// cvNamedWindow("ola",1);
// cvNamedWindow("ola1",1);
// cvNamedWindow("ola2",1);
// cvNamedWindow("ola3",1);


cvZero(pre_dst);
cvZero(pre_dst1);
cvZero(pre_dst2);
cvZero(mask1_tmp);
cvZero(mask3_tmp);



cvCopy(src,pre_dst);
//cvShowImage("ola1",src);

cvLine(pre_dst,cvPoint(0,hor),cvPoint(src->width,hor),cvScalar(0));
	

cvFloodFill( pre_dst, cvPoint(3,3), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
cvGetImage(mask3_tmp,pre_dst1);

//cvShowImage("ola",pre_dst1);

cvOr(pre_dst1,pre_dst,pre_dst2,NULL);


cvNot(pre_dst2,pre_dst2);



cvZero(pre_dst);
cvZero(pre_dst1);
cvZero(mask1_tmp);
cvZero(mask3_tmp);





int startline=hor+1;//pre_dst2->height-1;

for (i=0;i<TIMESTOANALISESPOTFORCROSS;i++)
{
	
	CvScalar total1pts=cvSum(pre_dst2);	
	int ab=(int)total1pts.val[0]/255;
	
	//printf("ab=%d\n",ab);
	
	if (ab<MINIMUMCROSSSIZE)
		break; 
	
	//cvShowImage("ola2",pre_dst2);
	CrossAnalisys.PixelNum=0;
	tobreak=0;
	cvZero(pre_dst);
	cvZero(pre_dst1);
	cvZero(mask1_tmp);
	cvZero(mask3_tmp);
	cvClearSeq(CrossAnalisys.pts);	

	/*for (line=startline;line>hor;line--)
	{
		for (col=15;col<pre_dst2->width-16;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(pre_dst2,line,col);	
			if (pixelvalue)
			{
				tobreak=1;
				endcol=col;
				endline=line;
				break;
			}
		}
		if (tobreak)
		{
			startline=line-1;
			break;
		}
	}*/
	for (line=startline;line<pre_dst2->height-10;line++)
	{
		for (col=15;col<pre_dst2->width-16;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(pre_dst2,line,col);	
			if (pixelvalue)
			{
				tobreak=1;
				endcol=col;
				endline=line;
				break;
			}
		}
		if (tobreak)
		{
			startline=line+10;
			break;
		}
	}
	//printf("endline=%d  endcol=%d\n",endline,endcol);
		
	cvCopy(pre_dst2,pre_dst);
	
	cvFloodFill( pre_dst, cvPoint(endcol,endline), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
	cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
	cvGetImage(mask3_tmp,pre_dst1);
	
	cvNot(pre_dst1,pre_dst);
	cvAnd(pre_dst2,pre_dst,pre_dst2,NULL);
	

	//cvShowImage("ola",pre_dst1);
	
	CvScalar totalpts=cvSum(pre_dst1);	
	
	CrossAnalisys.PixelNum=(int)totalpts.val[0]/255;
	
	//printf("PixelNum=%d\n",CrossAnalisys.PixelNum);
	
	
	if (CrossAnalisys.PixelNum>MINIMUMCROSSSIZE/*to make sure its not just an isolated pix*/)
	{
		
	
	
	CvPoint pt0;
	int npts=0;
	
		for (line=0;line<pre_dst1->height;line++)
		{
			for (col=0;col<pre_dst1->width;col++)
			{
				pixelvalue=(unsigned char) cvGetReal2D(pre_dst1,line,col);	
				if (pixelvalue)
				{
					pt0=cvPoint(col,line);
					npts++;
					cvSeqPush(CrossAnalisys.pts,&pt0); 
				}
			}
		}

	
		CvPoint2D32f pt[4];
		CvBox2D box=cvMinAreaRect2( CrossAnalisys.pts, NULL );
		cvBoxPoints( box, pt );
	
		
		
		CrossAnalisys.MARectangle.width=(int)box.size.width;
		CrossAnalisys.MARectangle.height=(int)box.size.height;
		CrossAnalisys.MARectangle.Area=CrossAnalisys.MARectangle.height*CrossAnalisys.MARectangle.width;
		
		CrossAnalisys.MARectangle.boxpts[0].x=(int)pt[0].x;
		CrossAnalisys.MARectangle.boxpts[0].y=(int)pt[0].y;
		
		CrossAnalisys.MARectangle.boxpts[1].x=(int)pt[1].x;
		CrossAnalisys.MARectangle.boxpts[1].y=(int)pt[1].y;
		
		CrossAnalisys.MARectangle.boxpts[2].x=(int)pt[2].x;
		CrossAnalisys.MARectangle.boxpts[2].y=(int)pt[2].y;
		
		CrossAnalisys.MARectangle.boxpts[3].x=(int)pt[3].x;
		CrossAnalisys.MARectangle.boxpts[3].y=(int)pt[3].y;
		
		CrossAnalisys.MARectangle.Center[0]=(box.center.x);
		CrossAnalisys.MARectangle.Center[1]=(box.center.y);
		
		
	
		CrossAnalisys.HeightWidthRatio=((float)macro_min(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height))/((float)macro_max(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height));
		
		
		
		int distancia1= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)*(CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)+(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y)*(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y));
		
		int distancia2= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)*(CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)+(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y)*(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y));
		
		
		if (macro_max(distancia1,distancia2)==distancia1) //get angle from 1 and 0
		{
			
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y,CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x);
		
		}
		else  //get angle from 2 and 1
		{
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y,CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x);
		}
		
		CrossAnalisys.MARAngle=angulo1;
		
		CrossAnalisys.CrossYVal=CrossAnalisys.MARectangle.boxpts[0].y;
		for (t=1;t<4;t++)
		{	
			if (CrossAnalisys.MARectangle.boxpts[t].y>CrossAnalisys.CrossYVal)
				CrossAnalisys.CrossYVal=CrossAnalisys.MARectangle.boxpts[t].y;
		}
		
		
		/*printf("distancia1=%d\n",distancia1);
		printf("distancia2=%d\n",distancia2);
		printf("angulo1=%f\n",angulo1);
		
		
		printf("width do MAR=%d\n",CrossAnalisys.MARectangle.width);
		printf("height do MAR=%d\n",CrossAnalisys.MARectangle.height);
		printf("Area do MAR=%d\n",CrossAnalisys.MARectangle.Area);
		printf("HWratio=%f\n",CrossAnalisys.HeightWidthRatio);
		printf("Angle=%f\n",CrossAnalisys.MARAngle);
		printf("Yval=%d\n",CrossAnalisys.CrossYVal);
		
		cvWaitKey(0);*/
		
		
		if (CrossAnalisys.HeightWidthRatio<0.30 && (angulo1<30.0 && angulo1>-30.0))
		{
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[0],CrossAnalisys.MARectangle.boxpts[1],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[1],CrossAnalisys.MARectangle.boxpts[2],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[2],CrossAnalisys.MARectangle.boxpts[3],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[3],CrossAnalisys.MARectangle.boxpts[0],cvScalar(128));
		itsacross=1;
		//cvShowImage("OTHER ->cross_Img",cross_Img);
			break;	
		}
		
	}
	else continue;
	
	

}
	


if (itsacross)
{
	Mode.DistantCross=CROSSFOUND;
	TM_UpdateTick(&Time.Tick.LastCross);
	a=0;	
}
else
{
	Mode.DistantCross=NOCROSSFOUND;
	a=1;	
}	
		

	#if NPOUT
		printf("______________FindCross Display________________\n");
		if (a==0)
		printf("Mode.Distant=CROSSFOUND\n");
		else if (a==1)
		printf("Mode.Distant=NOCROSSFOUND\n");
		printf("_______________________________________________\n");
	#endif
	
	#if IPAINT
		if (a==0)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
		else if (a==1)
			cvPutText(RoadImg_rgb_hough, "Distant Cross: NOT FOUND", cvPoint(0,45),&font, CV_RGB(20,180,0));
	#endif

}

/**@brief Cross finding function version 3. Used by the program.
 * Searches cross starting from the image top
 * @param src pointer to source image
 * @param hor horizon
 */
void NAVP_FindCross3(IplImage *src,int hor)
{





static IplImage *InnerSpotsImg= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static IplImage *CurrentSpotsImg= cvCreateImage(cvSize(src->width,src->height),8,1); //it is


int col=0,line=0,t;
unsigned char pixelvalue;


int endline=0,endcol=src->width-5;
int a=77,tobreak=0;;
int itsacross=0;
float angulo1;
char str[255];

unsigned int npts=0;
double tigetreal=0;		
CvPoint pt0;

	cvZero(GI.Cross.test1);

	NAVP_IsolateInnerSpots(src, InnerSpotsImg);

	
	
	
	
int startline=hor+1;

for (CrossAnalisys.AttemptedSpots=0 ;CrossAnalisys.AttemptedSpots<TIMESTOANALISESPOTFORCROSS; CrossAnalisys.AttemptedSpots++)
{	
	//cout << "for cycle number " <<  CrossAnalisys.AttemptedSpots <<  " spots " << endl;

	TM_UpdateTick(&Time.Tick.Tick1);			
	CvScalar total1pts=cvSum(InnerSpotsImg);
	int ab=(int)total1pts.val[0]/255;

	//printf("ab=%d\n",ab);

	if (ab<MINIMUMCROSSSIZE){break;} 

	
	CrossAnalisys.PixelNum=0;
	tobreak=0;
	cvClearSeq(CrossAnalisys.pts);	

	
	for (line=startline;line<InnerSpotsImg->height-10;line++)//Look for a white pixel to serve as a seed point
	{
		for (col=15;col<InnerSpotsImg->width-16;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D(InnerSpotsImg,line,col);
			if (pixelvalue)
			{
				tobreak=1;
				endcol=col;
				endline=line;
				break;
			}
		}
		if (tobreak)
		{
			startline=line+10;
			break;
		}
	}
	//printf("endline=%d  endcol=%d\n",endline,endcol);
	
// 	cvNamedWindow("InnerSpotsImgWithSpots",1);
// 	cvShowImage("InnerSpotsImgWithSpots",InnerSpotsImg);
// 	
	
	NAP_GetSpot(InnerSpotsImg, CurrentSpotsImg, endline, endcol);
	
	
	
	
	
	
// 	cout << "endline=" << endline << "endcol" << endcol << endl;
// 	cvNamedWindow("CurrentSpotsImg",1);
// 	cvShowImage("CurrentSpotsImg",CurrentSpotsImg);
// 	
// 	cvNamedWindow("InnerSpotsImgWithoutSpots",1);
// 	cvShowImage("InnerSpotsImgWithoutSpots",InnerSpotsImg);
// 	
	
	
	CvScalar totalpts=cvSum(CurrentSpotsImg);	
	
	CrossAnalisys.PixelNum=(int)totalpts.val[0]/255;
	
	//printf("PixelNum=%d and MINIMUMCROSSSIZE=%d\n",CrossAnalisys.PixelNum,MINIMUMCROSSSIZE);
	
	
	if (CrossAnalisys.PixelNum>MINIMUMCROSSSIZE/*to make sure its not just an isolated pix*/)
	{
		
	
	
	
	
	TM_UpdateTick(&Time.Tick.Tick3);
	tigetreal=0;	

//cvCopy(pre_dst1,CrossTest);


		for (line=hor+1;line<CurrentSpotsImg->height;line++)
		{
			for (col=0;col<CurrentSpotsImg->width;col++)
			{	
				pixelvalue = (unsigned char) cvGetReal2D(CurrentSpotsImg ,line ,col ) ; //cvmGet(test1,line,col);			
 				if (pixelvalue)
 				{
					pt0=cvPoint(col,line);
 					npts++;
 					cvSeqPush(CrossAnalisys.pts,&pt0); 
 				}
			}
		}
		
		
	TM_UpdateTick(&Time.Tick.Tick4);
	
	//cout << "seqpushgeral: " << (Time.Tick.Tick4-Time.Tick.Tick3)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;
	//cout << "totaalpts = " << line+col << "  used pts = " << npts << endl; 	
	
		TM_UpdateTick(&Time.Tick.Tick1);
		
		CvPoint2D32f pt[4];
		CvBox2D box=cvMinAreaRect2( CrossAnalisys.pts, NULL );
		cvBoxPoints( box, pt );
	
		TM_UpdateTick(&Time.Tick.Tick2);
	//cout << "MAR calculation: " << (Time.Tick.Tick2-Time.Tick.Tick1)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl << endl << endl << endl;	
	
	
	//cvWaitKey(0);	
		CrossAnalisys.MARectangle.width=(int)box.size.width;
		CrossAnalisys.MARectangle.height=(int)box.size.height;
		CrossAnalisys.MARectangle.Area=CrossAnalisys.MARectangle.height*CrossAnalisys.MARectangle.width;
		
		CrossAnalisys.MARectangle.boxpts[0].x=(int)pt[0].x;
		CrossAnalisys.MARectangle.boxpts[0].y=(int)pt[0].y;
		
		CrossAnalisys.MARectangle.boxpts[1].x=(int)pt[1].x;
		CrossAnalisys.MARectangle.boxpts[1].y=(int)pt[1].y;
		
		CrossAnalisys.MARectangle.boxpts[2].x=(int)pt[2].x;
		CrossAnalisys.MARectangle.boxpts[2].y=(int)pt[2].y;
		
		CrossAnalisys.MARectangle.boxpts[3].x=(int)pt[3].x;
		CrossAnalisys.MARectangle.boxpts[3].y=(int)pt[3].y;
		
		CrossAnalisys.MARectangle.Center[0]=(box.center.x);
		CrossAnalisys.MARectangle.Center[1]=(box.center.y);
		
		
	
		CrossAnalisys.HeightWidthRatio=((float)macro_min(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height))/((float)macro_max(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height));
		
		
		
		int distancia1= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)*(CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)+(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y)*(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y));
		
		int distancia2= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)*(CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)+(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y)*(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y));
		
		
		if (macro_max(distancia1,distancia2)==distancia1) //get angle from 1 and 0
		{
			
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y,CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x);
		
		}
		else  //get angle from 2 and 1
		{
			angulo1=180.0/M_PI*atan2(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y,CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x);
		}
		
		CrossAnalisys.MARAngle=angulo1;
		
		CrossAnalisys.CrossYVal=CrossAnalisys.MARectangle.boxpts[0].y;
		for (t=1;t<4;t++)
		{	
			if (CrossAnalisys.MARectangle.boxpts[t].y>CrossAnalisys.CrossYVal)
				CrossAnalisys.CrossYVal=CrossAnalisys.MARectangle.boxpts[t].y;
		}
		
		
		/*printf("distancia1=%d\n",distancia1);
		printf("distancia2=%d\n",distancia2);
		printf("angulo1=%f\n",angulo1);
		
		
		printf("width do MAR=%d\n",CrossAnalisys.MARectangle.width);
		printf("height do MAR=%d\n",CrossAnalisys.MARectangle.height);
		printf("Area do MAR=%d\n",CrossAnalisys.MARectangle.Area);
		printf("HWratio=%f\n",CrossAnalisys.HeightWidthRatio);
		printf("Angle=%f\n",CrossAnalisys.MARAngle);
		printf("Yval=%d\n",CrossAnalisys.CrossYVal);
		
		cvWaitKey(0);*/
		
		
		if (CrossAnalisys.HeightWidthRatio<0.30 && (angulo1<30.0 && angulo1>-30.0))
		{
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[0],CrossAnalisys.MARectangle.boxpts[1],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[1],CrossAnalisys.MARectangle.boxpts[2],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[2],CrossAnalisys.MARectangle.boxpts[3],cvScalar(128));
			cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[3],CrossAnalisys.MARectangle.boxpts[0],cvScalar(128));
		itsacross=1;
		//cvShowImage("OTHER ->cross_Img",cross_Img);
			break;	
		}
		
	}
	else continue;
	
	

}
	


if (itsacross)
{
	Mode.DistantCross=CROSSFOUND;
	TM_UpdateTick(&Time.Tick.LastCross);
	a=0;	
}
else
{
	Mode.DistantCross=NOCROSSFOUND;
	a=1;	
}	
		

	#if NPOUT
		printf("______________FindCross Display________________\n");
		if (a==0)
		printf("Mode.Distant=CROSSFOUND\n");
		else if (a==1)
		printf("Mode.Distant=NOCROSSFOUND\n");
		printf("_______________________________________________\n");
	#endif
	
	
	sprintf(str,"Prog. Start %d",Time.Since.ProgramStart);
		
	
	#if IPAINT
		
		if (a==0)
			sprintf(str,"Dist Cross: FOUND (%d Spots tried)",CrossAnalisys.AttemptedSpots);	
		else if (a==1)
			sprintf(str,"Dist Cross: NOTFOUND (%d Spots tried)",CrossAnalisys.AttemptedSpots);	
			
		cvPutText(RoadImg_rgb_hough, str, cvPoint(0,45),&font, CV_RGB(20,180,0));
	#endif

}

/**@brief Finds all the go to points based on the middle and outside searches.
 * 
 * @param TLN dont remember
 * @param TRW dont remember
 */
/**
 * 
 * @param TLN 
 * @param TRW 
 */
void NAVP_GetGoToPoint(int *TLN,int *TRW)
{
int i=0;
int cont1=0;
int pixelval;


	//______________________________if both low points are valid___________________________________
	if(SquareRegion.OSearch.lowl.valid && SquareRegion.OSearch.lowr.valid)
	{
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(SquareRegion.OSearch.lowl.val[0], SquareRegion.OSearch.lowr.val[0]))
			break;	
	
		SquareRegion.Boxlow.lenght=(int) sqrt((SquareRegion.OSearch.lowl.val[0]-SquareRegion.OSearch.lowr.val[0]) *(SquareRegion.OSearch.lowl.val[0]-SquareRegion.OSearch.lowr.val[0]) + (SquareRegion.OSearch.lowl.val[1]- SquareRegion.OSearch.lowr.val[1]) * (SquareRegion.OSearch.lowl.val[1]-SquareRegion.OSearch.lowr.val[1])); //calculate lenght
		
		if (SquareRegion.OSearch.lowl.val[0]>SquareRegion.OSearch.lowr.val[0]) //then left corner is under the right corner
		{
			SquareRegion.LowGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+SquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			SquareRegion.LowGoToPoint[0]=SquareRegion.OSearch.lowl.val[0]; //x coords of lowgotopoint
		}
		else
		{
			SquareRegion.LowGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-SquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			SquareRegion.LowGoToPoint[0]=SquareRegion.OSearch.lowr.val[0]; //x coords of lowgotopoint
		}
		SquareRegion.ValidLowGoToPoint=1;
		
		SquareRegion.LowGoToPoint[0] = saturatemin(SquareRegion.LowGoToPoint[0],0);
		SquareRegion.LowGoToPoint[0] = saturatemax(SquareRegion.LowGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowGoToPoint[1] = saturatemin(SquareRegion.LowGoToPoint[1],0);
		SquareRegion.LowGoToPoint[1] = saturatemax(SquareRegion.LowGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		
		
		
		if (SquareRegion.ValidLowGoToPoint) //test for point relocation
		{
			cont1=0;
			do 
			{
				if (SquareRegion.LowGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowGoToPoint[0] ,SquareRegion.LowGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (SquareRegion.LowGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowGoToPoint[0] ,SquareRegion.LowGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.LowGoToPointWidth = cont2 + cont1;
		
			cout << "MAXADMISSIBLEROADWIDTH = " << MAXADMISSIBLEROADWIDTH << endl; 
			
		//	cvWaitKey(0);
			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /2)
			{	
				SquareRegion.RelocatedLowGoToPoint=1;
				SquareRegion.LowGoToPoint[1] = (SquareRegion.LowGoToPoint[1] - cont1) + (SquareRegion.LowGoToPointWidth/2);
				if (SquareRegion.LowGoToPoint[0] < 239-30)
					SquareRegion.LowGoToPoint[0] = SquareRegion.LowGoToPoint[0] +10;
			}
			else if (SquareRegion.LowGoToPointWidth > MAXADMISSIBLEROADWIDTH * TRW[i])
			{
				SquareRegion.ValidLowGoToPoint = 0;
				SquareRegion.MaxRoadWidthExcededLowGoToPoint = 1;
				//cout << "MAXADMISSIBLEROADWIDTH Exceded " << endl;
			}
		}
	}
	//_____________________________________________________________________________________
	
	//___________________________________if both top points are valid______________________
	if(SquareRegion.OSearch.topl.valid && SquareRegion.OSearch.topr.valid)
	{
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(SquareRegion.OSearch.topl.val[0], SquareRegion.OSearch.topr.val[0]))
				break;
				
		SquareRegion.Boxtop.lenght=(int) sqrt((SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topr.val[0]) *(SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topr.val[0]) + (SquareRegion.OSearch.topl.val[1]- SquareRegion.OSearch.topr.val[1]) * (SquareRegion.OSearch.topl.val[1]-SquareRegion.OSearch.topr.val[1])); //calculate lenght
		
		if (SquareRegion.OSearch.topl.val[0]>SquareRegion.OSearch.topr.val[0]) //then left corner is under the right corner
		{
			//SquareRegion.TopGoToPoint[1]=SquareRegion.OSearch.topl.val[1]+SquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			//SquareRegion.TopGoToPoint[0]=SquareRegion.OSearch.topl.val[0]; //x coords of lowgotopoint
			IplImage *TEMP= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
			cvCopy(mask_Img1,TEMP);
			
			cvSetImageROI(mask_Img1,cvRect(SquareRegion.OSearch.topl.val[1],SquareRegion.OSearch.topl.val[0],-(SquareRegion.OSearch.topl.val[1]-SquareRegion.OSearch.topr.val[1]),-(SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topl.val[0])));
			cvRectangle( TEMP, cvPoint(SquareRegion.OSearch.topl.val[1],SquareRegion.OSearch.topl.val[0]), cvPoint(SquareRegion.OSearch.topr.val[1],SquareRegion.OSearch.topr.val[0]), cvScalar(128),
							1, 8,0 );
			
			cvNamedWindow("Display",1);
			cvShowImage("Display",TEMP);
			
			cvWaitKey(0);
			cvResetImageROI(mask_Img1);
		}
		else
		{
			//SquareRegion.TopGoToPoint[1]=SquareRegion.OSearch.topr.val[1]-SquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			//SquareRegion.TopGoToPoint[0]=SquareRegion.OSearch.topr.val[0]; //x coords of lowgotopoint
			IplImage *TEMP= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
			cvCopy(mask_Img1,TEMP);
			
			
			int line=SquareRegion.OSearch.topl.val[0];
			int col=SquareRegion.OSearch.topl.val[1];
			int w=SquareRegion.OSearch.topr.val[1] - SquareRegion.OSearch.topl.val[1];
			int h=SquareRegion.OSearch.topr.val[0] - SquareRegion.OSearch.topl.val[0];
			
			if (!h)h++;
			
			cvSetImageROI(mask_Img1,cvRect(col,line,w,h));
			
			
			
			
			cvRectangle( TEMP, cvPoint(col,line), cvPoint(col+w,line+h), cvScalar(128),1, 8,0 );
		
			
			
			
			
			CvMoments moments;
			cvMoments( mask_Img1, &moments, 1 );
			double spxmed=0;
			double M00=0;
			
			M00 = cvGetSpatialMoment( &moments, 0, 0);
			spxmed = cvGetSpatialMoment( &moments, 1, 0)/M00;
			cvResetImageROI(mask_Img1);
			
			GF_DrawCross(RoadImg_rgb_hough,(int)line,(int)(col+spxmed),8,Color.darkblue);
			
			
			cvLine( mask_Img1, cvPoint(col,line), cvPoint((int)(col+spxmed),line), cvScalar(100), 3, 8 );
        		
			#if NPOUT
			cout << "line = " << line << endl;		
			cout << "col = " << col << endl;		
			cout << "w" << w << endl;
			cout << "h" << h << endl;
			
			cvNamedWindow("D",1);
			cvShowImage("D",mask_Img1);
			cvNamedWindow("Display",1);
			cvShowImage("Display",TEMP);
			cvNamedWindow("D1",1);
			cvShowImage("D1",mask_Img1);
			
			
			cvWaitKey(0);
			#endif
			
		}
		
		
		
		
		SquareRegion.ValidTopGoToPoint=1;
		
		
		//cout << "SquareRegion.TopGoToPoint[0]" << SquareRegion.TopGoToPoint[0] << endl;
		SquareRegion.TopGoToPoint[0] = saturatemin(SquareRegion.TopGoToPoint[0],0);
		SquareRegion.TopGoToPoint[0] = saturatemax(SquareRegion.TopGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		//cout << "SquareRegion.TopGoToPoint[0]" << SquareRegion.TopGoToPoint[0] << endl;
		
		//cout << "SquareRegion.TopGoToPoint[1]" << SquareRegion.TopGoToPoint[1] << endl;
		SquareRegion.TopGoToPoint[1] = saturatemin(SquareRegion.TopGoToPoint[1],0);
		SquareRegion.TopGoToPoint[1] = saturatemax(SquareRegion.TopGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		//cout << "SquareRegion.TopGoToPoint[1]" << SquareRegion.TopGoToPoint[1] << endl;
		
		
		int a=40;
		a = (int)cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] , SquareRegion.TopGoToPoint[1]);
		cout << "a = " << a << endl;
		
		if (a==0)
		{
				SquareRegion.ValidTopGoToPoint=0;
				cout << "TopGoToPoint Invalid not a white pix" << endl;
				//cvWaitKey(0);
		}
		
		if (SquareRegion.ValidTopGoToPoint)//test for relocation
		{
		
				
			
			
			cont1=0;
			do 
			{
				if (SquareRegion.TopGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] ,SquareRegion.TopGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (SquareRegion.TopGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] ,SquareRegion.TopGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.TopGoToPointWidth = cont2 + cont1;
			
			
			
			cout << "macro_min(cont1,cont2)" << macro_min(cont1,cont2) <<endl;
			cout << "TRW[i] * (ROBOTWIDTHFACTORTOROADWITH *2 = "  << TRW[i] * ROBOTWIDTHFACTORTOROADWITH *2  <<endl;
			
			
			//Relocation condition
			if (macro_min(cont1,cont2) <  TRW[i] * (ROBOTWIDTHFACTORTOROADWITH /2))
			{
			
				
				SquareRegion.RelocatedTopGoToPoint=1;
				SquareRegion.TopGoToPoint[1] = (SquareRegion.TopGoToPoint[1] - cont1) + (SquareRegion.TopGoToPointWidth/2);
				if (SquareRegion.TopGoToPoint[0] < 239-10)
					SquareRegion.TopGoToPoint[0] = SquareRegion.TopGoToPoint[0] +10;
				else 
					SquareRegion.TopGoToPoint[0]=239;
			}
			else if (SquareRegion.TopGoToPointWidth > MAXADMISSIBLEROADWIDTH * TRW[i] || SquareRegion.TopGoToPointWidth > SquareRegion.LowGoToPointWidth )
			{
				
				SquareRegion.ValidTopGoToPoint = 0;
				SquareRegion.MaxRoadWidthExcededTopGoToPoint = 1;
				//cout << "MAXADMISSIBLEROADWIDTH Exceded " << endl;
			}
		}
	}
	//__________________________________________________________________________________
	
	//___________________________________if lowleft point is valid______________________
	if(SquareRegion.OSearch.lowl.valid)
	{
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowl.val[0])
				break;
				
		SquareRegion.LowLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
		SquareRegion.LowLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/2;
		SquareRegion.ValidLowLeftGoToPoint=1;

		SquareRegion.LowLeftGoToPoint[0] = saturatemin(SquareRegion.LowLeftGoToPoint[0],0);
		SquareRegion.LowLeftGoToPoint[0] = saturatemax(SquareRegion.LowLeftGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowLeftGoToPoint[1] = saturatemin(SquareRegion.LowLeftGoToPoint[1],0);
		SquareRegion.LowLeftGoToPoint[1] = saturatemax(SquareRegion.LowLeftGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		if (SquareRegion.ValidLowLeftGoToPoint)//test for relocation
		{
			cont1=0;
			do 
			{
				if (SquareRegion.LowLeftGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowLeftGoToPoint[0] ,SquareRegion.LowLeftGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (SquareRegion.LowLeftGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowLeftGoToPoint[0] ,SquareRegion.LowLeftGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.LowLeftGoToPointWidth = cont2 + cont1;
			
			
		}
	}
	//___________________________________________________________________________________
	
	//___________________________________if lowright point is valid______________________
	if(SquareRegion.OSearch.lowr.valid)
	{
		
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowr.val[0])
				break;
		
		SquareRegion.LowRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
		SquareRegion.LowRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/2;
		SquareRegion.ValidLowRightGoToPoint=1;
		
		SquareRegion.LowRightGoToPoint[0] = saturatemin(SquareRegion.LowRightGoToPoint[0],0);
		SquareRegion.LowRightGoToPoint[0] = saturatemax(SquareRegion.LowRightGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowRightGoToPoint[1] = saturatemin(SquareRegion.LowRightGoToPoint[1],0);
		SquareRegion.LowRightGoToPoint[1] = saturatemax(SquareRegion.LowRightGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		if (SquareRegion.ValidLowRightGoToPoint)//test for relocation
		{
			cont1=0;
			do 
			{
				if (SquareRegion.LowRightGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowRightGoToPoint[0] ,SquareRegion.LowRightGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (SquareRegion.LowRightGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowRightGoToPoint[0] ,SquareRegion.LowRightGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.LowRightGoToPointWidth = cont2 + cont1;
			
			//Relocation condition
			/*if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /2)
			{	
				SquareRegion.RelocatedLowRightGoToPoint=1;
				SquareRegion.LowRightGoToPoint[1] = (SquareRegion.LowRightGoToPoint[1] - cont1) + (SquareRegion.LowRightGoToPointWidth/2);
			}*/	
		}
	}
	
	
	//_____________________________if both left point are valid___________________________
	if(SquareRegion.OSearch.lowl.valid && SquareRegion.OSearch.topl.valid)
	{
		float alfa,x0,x1,y0,y1;	
	
		x0=(float)SquareRegion.OSearch.lowl.val[1];
		y0=(float)(240-SquareRegion.OSearch.lowl.val[0]);
		x1=(float)SquareRegion.OSearch.topl.val[1];
		y1=(float)(240-SquareRegion.OSearch.topl.val[0]);
		
		alfa=(float)(180/M_PI*atan((y1-y0)/(x1-x0)));
		SquareRegion.LeftLineAngle=(float)(180/M_PI*atan2((y1-y0),(x1-x0)));
		
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowl.val[0])
				break;
		
		SquareRegion.BothLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
		SquareRegion.BothLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/2+(int)(TRW[i]/2*sin(M_PI/180*(ANGLEATSTRAIGHTLEFTLINE-alfa)));
		SquareRegion.ValidBothLeftGoToPoint=1;
		
		/*printf("alfa=%g\n",alfa);
		printf("ANGLEATSTRAIGHTLEFTLINE=%g\n",ANGLEATSTRAIGHTLEFTLINE);
		printf("compesador do angulo=%d\n",(int)(TRW[i]/2*sin(M_PI/180*(ANGLEATSTRAIGHTLEFTLINE-alfa))));
		printf("TRW[i]/2=%d\n",TRW[i]/2);
		printf("Coluna do LowLEFT=%d\n",SquareRegion.OSearch.lowl.val[1]);
		printf("Resultado=%d\n",SquareRegion.BothLeftGoToPoint[1]);
		cvWaitKey(0);*/

	}
	//_____________________________________________________________________________________
	
	//_____________________________if both right point are valid___________________________
	if(SquareRegion.OSearch.lowr.valid && SquareRegion.OSearch.topr.valid)
	{
		float alfa,x0,x1,y0,y1;	
	
		x0=(float)SquareRegion.OSearch.lowr.val[1];
		y0=(float)(240-SquareRegion.OSearch.lowr.val[0]);
		x1=(float)SquareRegion.OSearch.topr.val[1];
		y1=(float)(240-SquareRegion.OSearch.topr.val[0]);
		
		alfa=(float)(180/M_PI*atan((y1-y0)/(x1-x0)));
		SquareRegion.RightLineAngle=(float)(180/M_PI*atan2((y1-y0),(x1-x0)));
		
		
		cout << "alfa1= " << SquareRegion.RightLineAngle<< endl;
		
		for (i=0;i<161;i++)
			if (TLN[i]==SquareRegion.OSearch.lowr.val[0])
				break;
		
		SquareRegion.BothRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
		SquareRegion.BothRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/2+(int)(TRW[i]/2*sin(M_PI/180*(ANGLEATSTRAIGHTRIGHTLINE-alfa)));
		SquareRegion.ValidBothRightGoToPoint=1;
		
	}
	//______________________________________________________________________________________
	
	
	
	//Test for obstacle position
	if (Mode.Direction==TAKERIGHT && SquareRegion.RelocatedTopGoToPoint)
	{
	unsigned char pixelvalue;
	int col,col1=0,col2=0,col3=0;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, RobotStatus.Horizon+1, col );
			
			if (pixelvalue)
				break;
		}
		col1= col;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1,(mask_Img1->height-1)- ((mask_Img1->height-1)-(RobotStatus.Horizon+1))/2, col );
			
			if (pixelvalue)
				break;
		}
		col2= col;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, mask_Img1->height-1, col );
			
			if (pixelvalue)
				break;
		}
		col3= col;
		
		SquareRegion.LeftHorizonSpace = macro_max(col1,macro_max(col2,col3));
		
		if ((col1 > 80 && col2 > 80) || (col2 > 80 && col3 > 80))
		{
			SquareRegion.ObstacleIsInnerSide = 1;
			SquareRegion.LowRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
			SquareRegion.LowRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/4;
			SquareRegion.ValidLowRightGoToPoint=1;
		
			SquareRegion.LowRightGoToPoint[0] = saturatemin(SquareRegion.LowRightGoToPoint[0],0);
			SquareRegion.LowRightGoToPoint[0] = saturatemax(SquareRegion.LowRightGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
			SquareRegion.LowRightGoToPoint[1] = saturatemin(SquareRegion.LowRightGoToPoint[1],0);
			SquareRegion.LowRightGoToPoint[1] = saturatemax(SquareRegion.LowRightGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
			SquareRegion.RelocatedLowRightGoToPoint=1;
		}
	}
	else if (Mode.Direction==TAKELEFT && SquareRegion.RelocatedTopGoToPoint)
	{
	
	
	cout << "asfsdgfldfs+pgld+prflgd+pfgld+pflg" << endl;
	unsigned char pixelvalue;
	int col,col1=0,col2=0,col3=0;
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, RobotStatus.Horizon+1, col );
			
			if (pixelvalue)
			{	
				col1= col;
				break;
			}
		}
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1,(mask_Img1->height-1)- ((mask_Img1->height-1)-(RobotStatus.Horizon+1))/2, col );
			
			if (pixelvalue)
			{
				col2= col;
				break;
			}
		}
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, mask_Img1->height-1, col );
			
			if (pixelvalue)
			{
				col3= col;
				break;
			}
		}
		
		SquareRegion.RightHorizonSpace =  - macro_max(col1,macro_max(col2,col3));
		cout << "SquareRegion.RightHorizonSpace = " << SquareRegion.RightHorizonSpace <<endl; 
		
		if (((mask_Img1->width-1 -col1 >  80) && (mask_Img1->width-1 -col2 > 80)) || ((mask_Img1->width-1 -col2 > 80) && (mask_Img1->width-1 -col3 > 80)))
		{
			SquareRegion.ObstacleIsInnerSide = 1;
			SquareRegion.LowLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
			SquareRegion.LowLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/4;
			SquareRegion.ValidLowLeftGoToPoint=1;
		
			SquareRegion.LowLeftGoToPoint[0] = saturatemin(SquareRegion.LowLeftGoToPoint[0],0);
			SquareRegion.LowLeftGoToPoint[0] = saturatemax(SquareRegion.LowLeftGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
			SquareRegion.LowLeftGoToPoint[1] = saturatemin(SquareRegion.LowLeftGoToPoint[1],0);
			SquareRegion.LowLeftGoToPoint[1] = saturatemax(SquareRegion.LowLeftGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
			SquareRegion.RelocatedLowLeftGoToPoint=1;
		}
	}
	
	#if POUT
		printf("___________________GoToPoints__________________*\n");
		printf("Both Top (x,y,valid?)=[%d,%d,%d]\n", SquareRegion.TopGoToPoint[0],SquareRegion.TopGoToPoint[1],SquareRegion.ValidTopGoToPoint);
		if (SquareRegion.RelocatedTopGoToPoint)
			cout << "Both Top RELOCATED." << endl;
		if (SquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Top INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Both Low (x,y,valid?)=[%d,%d,%d]\n", SquareRegion.LowGoToPoint[0],SquareRegion.LowGoToPoint[1],SquareRegion.ValidLowGoToPoint);
		if (SquareRegion.RelocatedLowGoToPoint)
			cout << "Both Low RELOCATED." << endl;
		if (SquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Low INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Single LowLeft (x,y,valid?) =  [%d,%d,%d] \n",SquareRegion.LowLeftGoToPoint[0] , SquareRegion.LowLeftGoToPoint[1] , SquareRegion.ValidLowLeftGoToPoint);
		if (SquareRegion.RelocatedLowLeftGoToPoint)
			cout << "LowLeft RELOCATED." << endl;
		printf("Single LowRight (x,y,valid?) = [%d,%d,%d] \n" , SquareRegion.LowRightGoToPoint[0],SquareRegion.LowRightGoToPoint[1],SquareRegion.ValidLowRightGoToPoint);
		if (SquareRegion.RelocatedLowRightGoToPoint)
			cout << "Low Right RELOCATED." << endl;
		printf("Both Left (x,y,valid?) =  [%d,%d,%d] \n",SquareRegion.BothLeftGoToPoint[0] , SquareRegion.BothLeftGoToPoint[1] , SquareRegion.ValidBothLeftGoToPoint);
		printf("Both Right (x,y,valid?) = [%d,%d,%d] \n" , SquareRegion.BothRightGoToPoint[0],SquareRegion.BothRightGoToPoint[1],SquareRegion.ValidBothRightGoToPoint);
		printf("SquareRegion.ObstacleIsInnerSide = %d\n", SquareRegion.ObstacleIsInnerSide );
		printf("_______________________________________________\n\n");
	#endif

}


/**@brief Finds first black pixel after a white is found.(Searches in a given image colum) (Look from outside).
 * 
 * @param src 
 * @param low_line line to start search from.
 * @param top_line line to end search.
 */
void NAVP_FindPixelsLFO(IplImage *src,int low_line,int top_line)
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
		for (col=2;(col<2*(src->width-1)/3)/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
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
			for (col=2;(col<2*(src->width-1)/3)/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
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
		for (col=2;(col<2*(src->width-1)/3)/*(col<(src->width-1)/2)*//*because previous image treatment has blacked col=0*/ ;col++)
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
		//printf("******** LINE=*d **********\n",line);
		/*because previous image treatment has blacked col=img.width-1*/
		for (col=src->width-1-1;(col>(src->width-1)/3)/*col>((src->width-1)/2)*/ ;col--)
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
			for (col=src->width-1-1;(col>(src->width-1)/3)/*col>((src->width-1)/2)*/ ;col--)
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
	
	
	
	
	
	
	
	
	}
	
	blackfound=0;
	continuesearch=1;
	for (line=top_line;line<low_line;line++)  // look for top  right corner
	{
		//printf("******** LINE=*d **********\n",line);
		/*because previous image treatment has blacked col=img.width-1*/
		for (col=src->width-1-5;(col>(src->width-1)/3)/*col>((src->width-1)/2)*/ ;col--)
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
	
	
	#if POUT
		
		printf("\n***** Box Outside Search Results *******\n");
		printf("****** Left [x,y,valid]**** Right [x,y,valid]***\n");
		printf("* Top       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.OSearch.topl.val[0],SquareRegion.OSearch.topl.val[1] , SquareRegion.OSearch.topl.valid, SquareRegion.OSearch.topr.val[0], SquareRegion.OSearch.topr.val[1], SquareRegion.OSearch.topr.valid);
		printf("* Low       [%d,%d,%d]            [%d,%d,%d]\n", SquareRegion.OSearch.lowl.val[0],SquareRegion.OSearch.lowl.val[1] , SquareRegion.OSearch.lowl.valid, SquareRegion.OSearch.lowr.val[0], SquareRegion.OSearch.lowr.val[1], SquareRegion.OSearch.lowr.valid);
		printf("*** Box Outside Search Results END*****\n\n");
		
	#endif
	
	
	
}





/**@brief Calculates possible drive angles in the ox analysis and validates the corresponding flag.
 * 
 */
void NAVP_GetDAbyBox()
{
float x0,y0,x1,y1;	



	if(SquareRegion.ValidLowGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.LowGoToPoint[1];
		y1=(float)(240-SquareRegion.LowGoToPoint[0]);
		
		RobotStatus.DA.BothLow[0]=(int)(180/M_PI*atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.BothLow[0]<0)
			RobotStatus.DA.BothLow[0]=90-(fabs(RobotStatus.DA.BothLow[0]))/2;
		else
			RobotStatus.DA.BothLow[0]=90-(90+(90-RobotStatus.DA.BothLow[0]))/2;
		RobotStatus.DA.BothLow[1]=1;
		
	}	

	if(SquareRegion.ValidTopGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.TopGoToPoint[1];
		y1=(float)(240-SquareRegion.TopGoToPoint[0]);
		
		RobotStatus.DA.BothTop[0]=(int)(180/M_PI*atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.BothTop[0]<0)
			RobotStatus.DA.BothTop[0]=90-(fabs(RobotStatus.DA.BothTop[0]))/2;
		else
			RobotStatus.DA.BothTop[0]=90-(90+(90-RobotStatus.DA.BothTop[0]))/2;
	
		RobotStatus.DA.BothTop[1]=1;
		
	}
	
	if(SquareRegion.ValidLowLeftGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.LowLeftGoToPoint[1];
		y1=(float)(240-SquareRegion.LowLeftGoToPoint[0]);
		
		RobotStatus.DA.LowLeft[0] =(int) (180/M_PI* atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.LowLeft[0]<0)
			RobotStatus.DA.LowLeft[0]=90-(fabs(RobotStatus.DA.LowLeft[0]))/2;
		else
			RobotStatus.DA.LowLeft[0]=90-(90+(90-RobotStatus.DA.LowLeft[0]))/2;
		
		RobotStatus.DA.LowLeft[1]=1;
		
	}
	
	if(SquareRegion.ValidLowRightGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.LowRightGoToPoint[1];
		y1=(float)(240-SquareRegion.LowRightGoToPoint[0]);
		
		RobotStatus.DA.LowRight[0] = (int)(180/M_PI* atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.LowRight[0]<0)
			RobotStatus.DA.LowRight[0]=90-(fabs(RobotStatus.DA.LowRight[0]))/2;
		else
			RobotStatus.DA.LowRight[0]=90-(90+(90-RobotStatus.DA.LowRight[0]))/2;
		
		RobotStatus.DA.LowRight[1]=1;
		
	}
	
	if(SquareRegion.ValidBothLeftGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.BothLeftGoToPoint[1];
		y1=(float)(240-SquareRegion.BothLeftGoToPoint[0]);
		
		RobotStatus.DA.BothLeft[0] =(int) (180/M_PI* atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.BothLeft[0]<0)
			RobotStatus.DA.BothLeft[0]=90-(fabs(RobotStatus.DA.BothLeft[0]))/2;
		else
			RobotStatus.DA.BothLeft[0]=90-(90+(90-RobotStatus.DA.BothLeft[0]))/2;
		
		RobotStatus.DA.BothLeft[1]=1;
		
	}
	
	if(SquareRegion.ValidBothRightGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y);
		x1=(float)SquareRegion.BothRightGoToPoint[1];
		y1=(float)(240-SquareRegion.BothRightGoToPoint[0]);
		
		RobotStatus.DA.BothRight[0] =(int) (180/M_PI* atan((y1-y0)/(x1-x0)));
		if (RobotStatus.DA.BothRight[0]<0)
			RobotStatus.DA.BothRight[0]=90-(fabs(RobotStatus.DA.BothRight[0]))/2;
		else
			RobotStatus.DA.BothRight[0]=90-(90+(90-RobotStatus.DA.BothRight[0]))/2;
		
		RobotStatus.DA.BothRight[1]=1;
		
	}
	
	
	#if NPOUT
		printf("\n***********DA Display***************\n");
		printf("DAbyBothLow(values,valid?)=(%g , %g)\n",RobotStatus.DA.BothLow[0],RobotStatus.DA.BothLow[1]);
		printf("DAbyBothTop(values,valid?)=(%g , %g)\n",RobotStatus.DA.BothTop[0],RobotStatus.DA.BothTop[1]);
		printf("DAbyLowLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA.LowLeft[0],RobotStatus.DA.LowLeft[1]);
		printf("DAbyLowRight(values,valid?)=(%g , %g)\n",RobotStatus.DA.LowRight[0],RobotStatus.DA.LowRight[1]);
		printf("DAbyBothLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA.BothLeft[0],RobotStatus.DA.BothLeft[1]);
		printf("DAbyBothRight(values,valid?)=(%g , %g)\n",RobotStatus.DA.BothRight[0],RobotStatus.DA.BothRight[1]);
		printf("**********DA Display END************\n");
	#endif

}

void NAVP_GetDAbyPP1()
{
float x1,y1,D=800;		

	if(SquareRegion.ValidLowGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.LowGoToPoint[0];
		x1 =  SquareRegion.LowGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.BothLow[0] = nearbyint((45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D))))));
		RobotStatus.DA_PP.BothLow[1]=1;
	}	
	
	if(SquareRegion.ValidTopGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.TopGoToPoint[0];
		x1 =  SquareRegion.TopGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.BothTop[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.BothTop[1]=1;
	}
	
	if(SquareRegion.ValidLowLeftGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.LowLeftGoToPoint[0];
		x1 =  SquareRegion.LowLeftGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.LowLeft[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.LowLeft[1]=1;
	}
	
	if(SquareRegion.ValidLowRightGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.LowRightGoToPoint[0];
		x1 =  SquareRegion.LowRightGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.LowRight[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.LowRight[1]=1;
	}
	
	if(SquareRegion.ValidLowLeftGoToPoint_MoveClose)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.LowLeftGoToPoint_MoveClose[0];
		x1 =  SquareRegion.LowLeftGoToPoint_MoveClose[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.LowLeft_MoveClose[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.LowLeft_MoveClose[1]=1;
	}
	
	if(SquareRegion.ValidLowRightGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.LowRightGoToPoint_MoveClose[0];
		x1 =  SquareRegion.LowRightGoToPoint_MoveClose[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.LowRight_MoveClose[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.LowRight_MoveClose[1]=1;
	}
	
	if(SquareRegion.ValidBothLeftGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.BothLeftGoToPoint[0];
		x1 =  SquareRegion.BothLeftGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.BothLeft[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.BothLeft[1]=1;
	}
	
	if(SquareRegion.ValidBothRightGoToPoint)
	{		
		y1 = ImagesParams.NavImgSize.height - SquareRegion.BothRightGoToPoint[0];
		x1 =  SquareRegion.BothRightGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		RobotStatus.DA_PP.BothRight[0] = nearbyint(45+ 1*(180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D)))));
		RobotStatus.DA_PP.BothRight[1]=1;
	}
	
	#if POUT
		printf("\n***********DA_PP Display***************\n");
		printf("DA_PPbyBothLow(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothLow[0],RobotStatus.DA_PP.BothLow[1]);
		printf("DA_PPbyBothTop(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothTop[0],RobotStatus.DA_PP.BothTop[1]);
		printf("DA_PPbyLowLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.LowLeft[0],RobotStatus.DA_PP.LowLeft[1]);
		printf("DA_PPbyLowRight(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.LowRight[0],RobotStatus.DA_PP.LowRight[1]);
		printf("DA_PPbyBothLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothLeft[0],RobotStatus.DA_PP.BothLeft[1]);
		printf("DA_PPbyBothRight(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothRight[0],RobotStatus.DA_PP.BothRight[1]);
		printf("**********DA_PP Display END************\n");
	#endif

}


void NAVP_GetDAbyPP()
{
float x0,y0,x1,y1,yc,r,d=131;	



	if(SquareRegion.ValidLowGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.LowGoToPoint[1];
		y1=(float)(240-SquareRegion.LowGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.DA_PP.BothLow[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.DA_PP.BothLow[0]<0)
			RobotStatus.DA_PP.BothLow[0]=45-(90-(fabs(RobotStatus.DA_PP.BothLow[0])))/2;
		else
			RobotStatus.DA_PP.BothLow[0]=45+(90-(fabs(RobotStatus.DA_PP.BothLow[0])))/2;
		RobotStatus.DA_PP.BothLow[1]=1;
		
	}	
	if(SquareRegion.ValidTopGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.TopGoToPoint[1];
		y1=(float)(240-SquareRegion.TopGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.DA_PP.BothTop[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.DA_PP.BothTop[0]<0)
			RobotStatus.DA_PP.BothTop[0]=45-(90-(fabs(RobotStatus.DA_PP.BothTop[0])))/2;
		else
			RobotStatus.DA_PP.BothTop[0]=45+(90-(fabs(RobotStatus.DA_PP.BothTop[0])))/2;
		
		RobotStatus.DA_PP.BothTop[1]=1;
		
	}
	if(SquareRegion.ValidLowLeftGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.LowLeftGoToPoint[1];
		y1=(float)(240-SquareRegion.LowLeftGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.DA_PP.LowLeft[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.DA_PP.LowLeft[0]<0)
			RobotStatus.DA_PP.LowLeft[0]=45-(90-(fabs(RobotStatus.DA_PP.LowLeft[0])))/2;
		else
			RobotStatus.DA_PP.LowLeft[0]=45+(90-(fabs(RobotStatus.DA_PP.LowLeft[0])))/2;
		RobotStatus.DA_PP.LowLeft[1]=1;
		
	}
	if(SquareRegion.ValidLowRightGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.LowRightGoToPoint[1];
		y1=(float)(240-SquareRegion.LowRightGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.DA_PP.LowRight[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.DA_PP.LowRight[0]<0)
			RobotStatus.DA_PP.LowRight[0]=45-(90-(fabs(RobotStatus.DA_PP.LowRight[0])))/2;
		else
			RobotStatus.DA_PP.LowRight[0]=45+(90-(fabs(RobotStatus.DA_PP.LowRight[0])))/2;
		RobotStatus.DA_PP.LowRight[1]=1;
		
	}
	
	
	if(SquareRegion.ValidBothLeftGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.BothLeftGoToPoint[1];
		y1=(float)(240-SquareRegion.BothLeftGoToPoint[0]);
		yc=(float)-179;
		
		
		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
		
		RobotStatus.DA_PP.BothLeft[0]=(int)((180.0/M_PI*atan(r/d)));
		if (RobotStatus.DA_PP.BothLeft[0]<0)
			RobotStatus.DA_PP.BothLeft[0]=45-(90-(fabs(RobotStatus.DA_PP.BothLeft[0])))/2;
		else
			RobotStatus.DA_PP.BothLeft[0]=45+(90-(fabs(RobotStatus.DA_PP.BothLeft[0])))/2;
		RobotStatus.DA_PP.BothLeft[1]=1;
		
	}	
	
	if(SquareRegion.ValidBothRightGoToPoint)
	{
		x0=(float)SquareRegion.PresentPoint.x;
		y0=(float)(240-SquareRegion.PresentPoint.y)-179; //y value of back wheels center 
		x1=(float)SquareRegion.BothRightGoToPoint[1];
		y1=(float)(240-SquareRegion.BothRightGoToPoint[0]);
		yc=(float)-131;
		
		
		float D = -yc;
		y1 = 240 - SquareRegion.BothRightGoToPoint[0];
		x1 =  SquareRegion.BothRightGoToPoint[1] - ImagesParams.NavImgSize.width/2;
		
		
		RobotStatus.DA_PP.BothRight[0] = 45+ (180.0/M_PI*atan((-2.0*D*x1) /(x1*x1+(y1+D)*(y1+D))));
		
		//cout << "RobotStatus.DA_PP.BothRight[0] = " << RobotStatus.DA_PP.BothRight[0] << endl;
// 		r=(sqrt(x0*x0-2*x0*x1+x1*x1+y0*y0-2*y0*y1+y1*y1)*sqrt(4*yc*yc-4*(y0+y1)*yc+x0*x0-2*x0*x1+x1*x1+y0*y0+2*y0*y1+y1*y1))/(2*(x0-x1));
// 		
// 		RobotStatus.DA_PP.BothRight[0]=(int)((180.0/M_PI*atan(r/d)));
// 		if (RobotStatus.DA_PP.BothRight[0]<0)
// 			RobotStatus.DA_PP.BothRight[0]=45-(90-(fabs(RobotStatus.DA_PP.BothRight[0])))/2;
// 		else
// 			RobotStatus.DA_PP.BothRight[0]=45+(90-(fabs(RobotStatus.DA_PP.BothRight[0])))/2;
// 		RobotStatus.DA_PP.BothRight[1]=1;
		
		
// 		cout << "((180.0/M_PI*atan(r/d)))" << ((180.0/M_PI*atan(r/d))) << endl;
// 		cout << "(fabs(RobotStatus.DA_PP.BothRight[0]))) = " << (fabs(RobotStatus.DA_PP.BothRight[0])) << endl;
// 		cout << "RobotStatus.DA_PP.BothRight[0] = " << RobotStatus.DA_PP.BothRight[0] << endl;
	}
	
	#if POUT
		printf("\n***********DA_PP Display***************\n");
		printf("DA_PPbyBothLow(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothLow[0],RobotStatus.DA_PP.BothLow[1]);
		printf("DA_PPbyBothTop(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothTop[0],RobotStatus.DA_PP.BothTop[1]);
		printf("DA_PPbyLowLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.LowLeft[0],RobotStatus.DA_PP.LowLeft[1]);
		printf("DA_PPbyLowRight(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.LowRight[0],RobotStatus.DA_PP.LowRight[1]);
		printf("DA_PPbyBothLeft(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothLeft[0],RobotStatus.DA_PP.BothLeft[1]);
		printf("DA_PPbyBothRight(values,valid?)=(%g , %g)\n",RobotStatus.DA_PP.BothRight[0],RobotStatus.DA_PP.BothRight[1]);
		printf("**********DA_PP Display END************\n");
	#endif

}

void NAVP_CheckLineSteping_1(IplImage *src,int *TLN,int *TRW)
{int i;
	IplImage *tmp;
	
	tmp = cvCloneImage(src);
	
	
		for (i=0;i<239;i++)
			if (TLN[i]==SquareRegion.PresentPoint.y)
				break;
		
		RobotStatus.DA_PP.BothTopLineSteping.LowRight[0]=SquareRegion.PresentPoint.y;
		RobotStatus.DA_PP.BothTopLineSteping.LowRight[1]=(int)(SquareRegion.PresentPoint.x+TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0]=SquareRegion.PresentPoint.y;
		RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1]=(int)(SquareRegion.PresentPoint.x-TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);

		for (i=0;i<239;i++)
			if (TLN[i]==SquareRegion.TopGoToPoint[0])
				break;
		
		RobotStatus.DA_PP.BothTopLineSteping.TopRight[0]=SquareRegion.TopGoToPoint[0];
		RobotStatus.DA_PP.BothTopLineSteping.TopRight[1]=(int)(SquareRegion.TopGoToPoint[1]+TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		RobotStatus.DA_PP.BothTopLineSteping.TopLeft[0]=SquareRegion.TopGoToPoint[0];
		RobotStatus.DA_PP.BothTopLineSteping.TopLeft[1]=(int)(SquareRegion.TopGoToPoint[1]-TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		
	//________________________________Test Right stepping_____________________________________
	cvZero(tmp);	
	cvLine(tmp, cvPoint(RobotStatus.DA_PP.BothTopLineSteping.LowRight[1],RobotStatus.DA_PP.BothTopLineSteping.LowRight[0]), cvPoint(RobotStatus.DA_PP.BothTopLineSteping.TopRight[1],RobotStatus.DA_PP.BothTopLineSteping.TopRight[0]),cvScalar(255),1, 8, 0);
	cvXor(src,tmp,tmp,tmp);
	RobotStatus.OKRIGHT = (cvCountNonZero(tmp)<=1);
	
	//________________________________Test Left stepping_____________________________________
	cvZero(tmp);	
	cvLine(tmp, cvPoint(RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1],RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0]), cvPoint(RobotStatus.DA_PP.BothTopLineSteping.TopLeft[1],RobotStatus.DA_PP.BothTopLineSteping.TopLeft[0]),cvScalar(255),1, 8, 0);
	cvXor(src,tmp,tmp,tmp);
	RobotStatus.OKLEFT = (cvCountNonZero(tmp)<=1);
	
	cvReleaseImage(&tmp);
	
	
	#if IPAINT
	char str[255];
		sprintf(str,"LS.OKRIGHT = %d",RobotStatus.OKRIGHT );
		cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-100,120),&smallfont, CV_RGB(0,120,255));	
		sprintf(str,"LS.OKLEFT = %d",RobotStatus.OKLEFT );
		cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-100,130),&smallfont, CV_RGB(0,120,255));	
	#endif
}


/**@brief Unvalidates BothTop analisys in case this forces the robot to step on the line.
 * 
 * @param src source image.
 * @param TLN dont remember.
 * @param TRW dont remember.
 */
void NAVP_CheckLineSteping(IplImage *src,int *TLN,int *TRW)
{
int i;
unsigned char a;
int pixelvalue_right=0,pixelvalue_left=0;	
	
	
	if (!RobotStatus.DA_PP.BothTop[1])
	{
		a=0;
	}
	else  //if the robot is steping...
	{
	
		for (i=0;i<239;i++)
			if (TLN[i]==SquareRegion.PresentPoint.y)
				break;
		
		RobotStatus.DA_PP.BothTopLineSteping.LowRight[0]=SquareRegion.PresentPoint.y;
		RobotStatus.DA_PP.BothTopLineSteping.LowRight[1]=(int)(SquareRegion.PresentPoint.x+TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		
		RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0]=SquareRegion.PresentPoint.y;
		RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1]=(int)(SquareRegion.PresentPoint.x-TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
				
		
		for (i=0;i<239;i++)
			if (TLN[i]==SquareRegion.TopGoToPoint[0])
				break;
		
		RobotStatus.DA_PP.BothTopLineSteping.TopRight[0]=SquareRegion.TopGoToPoint[0];
		RobotStatus.DA_PP.BothTopLineSteping.TopRight[1]=(int)(SquareRegion.TopGoToPoint[1]+TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		
		
		
		RobotStatus.DA_PP.BothTopLineSteping.TopLeft[0]=SquareRegion.TopGoToPoint[0];
		RobotStatus.DA_PP.BothTopLineSteping.TopLeft[1]=(int)(SquareRegion.TopGoToPoint[1]-TRW[i]*ROBOTWIDTHFACTORTOROADWITH/2);
		
		
		pixelvalue_right=(int) cvGetReal2D( src,RobotStatus.DA_PP.BothTopLineSteping.LowRight[0],RobotStatus.DA_PP.BothTopLineSteping.LowRight[1]);	
		
		pixelvalue_left=(int) cvGetReal2D( src,RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0],RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1]);	
		
		//cout << "pixelvalue_left = " << pixelvalue_left << endl;
		//cout << "pixelvalue_right = " << pixelvalue_right << endl;
		
		if (!pixelvalue_left  || !pixelvalue_right)
		{
			RobotStatus.DA_PP.BothTop[1]=0;
			a=1;	
		}
		else 
			a=2;
		}
	
		
		
	#if NPOUT
		printf("_____________CheckLineSteping Display__________\n");
		if (a==0)
		printf("No Both Top Analisys. Nothing to check.\n");
		else if (a==1)
		printf("Test Failed. Disabling DA_PP.BothTop validation flag]\n");
		else if (a==2)
		printf("Test Passed.\n");
		printf("_______________________________________________\n");
	#endif	
		
}


void NAVP_GetOcupation(IplImage *src)
{
CvScalar totalpts;
unsigned int ab=0;
Ocupation.BiggestPercentArea =0;

//_________________Get Ocupation for Area1___________________________		
for (int i=0;i<8;i++)
{
	cvSetImageROI( src, cvRect(Ocupation.N[i].Area[0].x, Ocupation.N[i].Area[1].y, Ocupation.N[i].Area[3].x - Ocupation.N[i].Area[0].x , Ocupation.N[i].Area[0].y - Ocupation.N[i].Area[1].y));
	totalpts=cvSum(src);
	ab=(unsigned int)totalpts.val[0]/255;
	Ocupation.N[i].AreaOcupationPercent = (ab*100) / Ocupation.N[i].AreaPixelNum ;

	//printf("Ocupation.N[%d].AreaOcupationPercent = %d\n", i,(int)Ocupation.N[i].AreaOcupationPercent);
	//cvWaitKey(0);
	
	if (macro_max((int)Ocupation.N[i].AreaOcupationPercent, (int)Ocupation.N[Ocupation.BiggestPercentArea ].AreaOcupationPercent) == Ocupation.N[i].AreaOcupationPercent)
			Ocupation.BiggestPercentArea = i;		
}
	cvResetImageROI(src);
	
	#if NPOUT
		printf("Ocupation.BiggestPercentArea = %d\n", Ocupation.BiggestPercentArea);
	#endif
}
#endif







void NAVP_GetGoToPoint_1(int *TLN,int *TRW)
{
int i=0;
int cont1=0,cont2=0;
int pixelval;

	
	//____________________________________________________________________________________
	//______________________________if BOTH LOW points are valid__________________________
	//____________________________________________________________________________________
	if(SquareRegion.OSearch.lowl.valid && SquareRegion.OSearch.lowr.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(SquareRegion.OSearch.lowl.val[0], SquareRegion.OSearch.lowr.val[0]))
			break;	
	
		//______________________Get distance between points___________________________
		SquareRegion.Boxlow.lenght=(int) sqrt((SquareRegion.OSearch.lowl.val[0]-SquareRegion.OSearch.lowr.val[0]) *(SquareRegion.OSearch.lowl.val[0]-SquareRegion.OSearch.lowr.val[0]) + (SquareRegion.OSearch.lowl.val[1]- SquareRegion.OSearch.lowr.val[1]) * (SquareRegion.OSearch.lowl.val[1]-SquareRegion.OSearch.lowr.val[1]));
		
		//________________If left corner is under the right corner____________________
		if (SquareRegion.OSearch.lowl.val[0]>SquareRegion.OSearch.lowr.val[0]) 
		{
			SquareRegion.LowGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+SquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			SquareRegion.LowGoToPoint[0]=SquareRegion.OSearch.lowl.val[0]; //x coords of lowgotopoint
		}
		else//________________If right corner is under the left corner___________ 
		{
			SquareRegion.LowGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-SquareRegion.Boxlow.lenght/2; //y coords of lowgotopoint
			SquareRegion.LowGoToPoint[0]=SquareRegion.OSearch.lowr.val[0]; //x coords of lowgotopoint
		}
		SquareRegion.ValidLowGoToPoint=1;//Validate Corresponding flag
	
		//__________________Saturate point coords value_____________________________	
		SquareRegion.LowGoToPoint[0] = saturatemin(SquareRegion.LowGoToPoint[0],0);
		SquareRegion.LowGoToPoint[0] = saturatemax(SquareRegion.LowGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowGoToPoint[1] = saturatemin(SquareRegion.LowGoToPoint[1],0);
		SquareRegion.LowGoToPoint[1] = saturatemax(SquareRegion.LowGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		
		
		//teste ponto preto
		
		//______________________Get Road width at this point__________________________
		if (SquareRegion.ValidLowGoToPoint) 
		{	
			cont1=0;
			do 
			{	if (SquareRegion.LowGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowGoToPoint[0] ,SquareRegion.LowGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			cont2=0;
			do 
			{	if (SquareRegion.LowGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.LowGoToPoint[0] ,SquareRegion.LowGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.LowGoToPointWidth = cont2 + cont1;	
			SquareRegion.AdmissibleLowGoToPointWidth = (int)(MAXADMISSIBLEROADWIDTH * TRW[i]);
			
			//MAXADMISSIBLEROADWIDTH test
			if (SquareRegion.LowGoToPointWidth > SquareRegion.AdmissibleLowGoToPointWidth)
			{
				SquareRegion.ValidLowGoToPoint = 0;
				SquareRegion.MaxRoadWidthExcededLowGoToPoint = 1;
 			}
		
		}
		
		
// 		
// 			cout << "MAXADMISSIBLEROADWIDTH = " << MAXADMISSIBLEROADWIDTH << endl; 
// 			
// 		//	cvWaitKey(0);
// 			if (macro_min(cont1,cont2) <  TRW[i] * ROBOTWIDTHFACTORTOROADWITH /2)
// 			{	
// 				SquareRegion.RelocatedLowGoToPoint=1;
// 				SquareRegion.LowGoToPoint[1] = (SquareRegion.LowGoToPoint[1] - cont1) + (SquareRegion.LowGoToPointWidth/2);
// 				if (SquareRegion.LowGoToPoint[0] < 239-30)
// 					SquareRegion.LowGoToPoint[0] = SquareRegion.LowGoToPoint[0] +10;
// 			}
// 			else if (SquareRegion.LowGoToPointWidth > MAXADMISSIBLEROADWIDTH * TRW[i])
// 			{
// 				SquareRegion.ValidLowGoToPoint = 0;
// 				SquareRegion.MaxRoadWidthExcededLowGoToPoint = 1;
// 				//cout << "MAXADMISSIBLEROADWIDTH Exceded " << endl;
// 			}
// 		}
 	}


	//_____________________________________________________________________________________
	//___________________________________if BOTH TOP points are valid______________________
	//_____________________________________________________________________________________
	if(SquareRegion.OSearch.topl.valid && SquareRegion.OSearch.topr.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i] == macro_min(SquareRegion.OSearch.topl.val[0], SquareRegion.OSearch.topr.val[0]))
				break;
		
		//______________________Get distance between points___________________________
		SquareRegion.Boxtop.lenght=(int) sqrt((SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topr.val[0]) *(SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topr.val[0]) + (SquareRegion.OSearch.topl.val[1]- SquareRegion.OSearch.topr.val[1]) * (SquareRegion.OSearch.topl.val[1]-SquareRegion.OSearch.topr.val[1])); 
		
		 //If left corner is under the right corner
		if (SquareRegion.OSearch.topl.val[0]>SquareRegion.OSearch.topr.val[0])
		{
			SquareRegion.TopGoToPoint[1]=SquareRegion.OSearch.topl.val[1]+SquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			SquareRegion.TopGoToPoint[0]=SquareRegion.OSearch.topl.val[0]; //x coords of lowgotopoint
// 			IplImage *TEMP= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
// 			cvCopy(mask_Img1,TEMP);
// 			
// 			cvSetImageROI(mask_Img1,cvRect(SquareRegion.OSearch.topl.val[1],SquareRegion.OSearch.topl.val[0],-(SquareRegion.OSearch.topl.val[1]-SquareRegion.OSearch.topr.val[1]),-(SquareRegion.OSearch.topl.val[0]-SquareRegion.OSearch.topl.val[0])));
// 			cvRectangle( TEMP, cvPoint(SquareRegion.OSearch.topl.val[1],SquareRegion.OSearch.topl.val[0]), cvPoint(SquareRegion.OSearch.topr.val[1],SquareRegion.OSearch.topr.val[0]), cvScalar(128),
// 							1, 8,0 );
// 			
// 			cvNamedWindow("Display",1);
// 			cvShowImage("Display",TEMP);
// 			
// 			cvWaitKey(0);
// 			cvResetImageROI(mask_Img1);
		}
		else//If right corner is under the left corner
		{
			SquareRegion.TopGoToPoint[1]=SquareRegion.OSearch.topr.val[1]-SquareRegion.Boxtop.lenght/2; //y coords of lowgotopoint
			SquareRegion.TopGoToPoint[0]=SquareRegion.OSearch.topr.val[0]; //x coords of lowgotopoint
// 			IplImage *TEMP= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
// 			cvCopy(mask_Img1,TEMP);
// 			
// 			
// 			int line=SquareRegion.OSearch.topl.val[0];
// 			int col=SquareRegion.OSearch.topl.val[1];
// 			int w=SquareRegion.OSearch.topr.val[1] - SquareRegion.OSearch.topl.val[1];
// 			int h=SquareRegion.OSearch.topr.val[0] - SquareRegion.OSearch.topl.val[0];
// 			
// 			if (!h)h++;
// 			
// 			cvSetImageROI(mask_Img1,cvRect(col,line,w,h));
// 			 			
// 			cvRectangle( TEMP, cvPoint(col,line), cvPoint(col+w,line+h), cvScalar(128),1, 8,0 );
// 			CvMoments moments;
// 			cvMoments( mask_Img1, &moments, 1 );
// 			double spxmed=0;
// 			double M00=0;
// 			
// 			M00 = cvGetSpatialMoment( &moments, 0, 0);
// 			spxmed = cvGetSpatialMoment( &moments, 1, 0)/M00;
// 			cvResetImageROI(mask_Img1);
// 			
// 			GF_DrawCross(RoadImg_rgb_hough,(int)line,(int)(col+spxmed),8,Color.darkblue);
// 			
// 			cvLine( mask_Img1, cvPoint(col,line), cvPoint(col+spxmed,line), cvScalar(100), 3, 8 );
// 			#if NPOUT
// 			cout << "line = " << line << endl;		
// 			cout << "col = " << col << endl;		
// 			cout << "w" << w << endl;
// 			cout << "h" << h << endl;
// 			
// 			cvNamedWindow("D",1);
// 			cvShowImage("D",mask_Img1);
// 			cvNamedWindow("Display",1);
// 			cvShowImage("Display",TEMP);
// 			cvNamedWindow("D1",1);
// 			cvShowImage("D1",mask_Img1);
// 			
// 	
// 			#endif
			
		}
		
		SquareRegion.ValidTopGoToPoint=1;//Validate corresponding flag
		
		//__________________Saturate point coords value_____________________________
		SquareRegion.TopGoToPoint[0] = saturatemin(SquareRegion.TopGoToPoint[0],0);
		SquareRegion.TopGoToPoint[0] = saturatemax(SquareRegion.TopGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.TopGoToPoint[1] = saturatemin(SquareRegion.TopGoToPoint[1],0);
		SquareRegion.TopGoToPoint[1] = saturatemax(SquareRegion.TopGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		
		
		
// 		int a=40;
// 		a = (int)cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] , SquareRegion.TopGoToPoint[1]);
// 		cout << "a = " << a << endl;
// 		
// 		if (a==0)
// 		{
// 				SquareRegion.ValidTopGoToPoint=0;
// 				cout << "TopGoToPoint Invalid not a white pix" << endl;
// 				//cvWaitKey(0);
// 		}
		
		//______________________Get Road width at this point__________________________
		if (SquareRegion.ValidTopGoToPoint)
		{
			cont1=0;
			do 
			{
				if (SquareRegion.TopGoToPoint[1]-cont1 <0)
					break;
				
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] ,SquareRegion.TopGoToPoint[1]-cont1);
				cont1++;
			}while(pixelval);
			
			int cont2=0;
			do 
			{
				if (SquareRegion.TopGoToPoint[1]+cont2 >=mask_Img1->width)
					break;
					
				pixelval = (int) cvGetReal2D(mask_Img1, SquareRegion.TopGoToPoint[0] ,SquareRegion.TopGoToPoint[1]+cont2);
				cont2++;
				
			}while(pixelval);
			
			SquareRegion.TopGoToPointWidth = cont2 + cont1;
			SquareRegion.AdmissibleTopGoToPointWidth = (int)(MAXADMISSIBLEROADWIDTH * TRW[i]);
			
					
			//__________________MAXADMISSIBLEROADWIDTH Test________________________
			if (SquareRegion.TopGoToPointWidth >  SquareRegion.AdmissibleTopGoToPointWidth )
			{				
				SquareRegion.ValidTopGoToPoint = 0;
				SquareRegion.MaxRoadWidthExcededTopGoToPoint = 1;
			}
			
// 			//Relocation condition
// 			if (macro_min(cont1,cont2) <  TRW[i] * (ROBOTWIDTHFACTORTOROADWITH /2))
// 			{
// 			
// 				
// 				SquareRegion.RelocatedTopGoToPoint=1;
// 				SquareRegion.TopGoToPoint[1] = (SquareRegion.TopGoToPoint[1] - cont1) + (SquareRegion.TopGoToPointWidth/2);
// 				if (SquareRegion.TopGoToPoint[0] < 239-10)
// 					SquareRegion.TopGoToPoint[0] = SquareRegion.TopGoToPoint[0] +10;
// 				else 
// 					SquareRegion.TopGoToPoint[0]=239;
// 			}
// 			else 
		}
	}
	
	//__________________________________________________________________________________
	//___________________________________if LOWLEFT point is valid______________________
	//__________________________________________________________________________________
	if(SquareRegion.OSearch.lowl.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowl.val[0])
				break;
		
		//_____________________________Decide Point location________________________
		SquareRegion.LowLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
		SquareRegion.LowLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/2;
		
		SquareRegion.ValidLowLeftGoToPoint=1;//validate corresponding flag

		//__________________Saturate point coords value_____________________________
		SquareRegion.LowLeftGoToPoint[0] = saturatemin(SquareRegion.LowLeftGoToPoint[0],0);
		SquareRegion.LowLeftGoToPoint[0] = saturatemax(SquareRegion.LowLeftGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowLeftGoToPoint[1] = saturatemin(SquareRegion.LowLeftGoToPoint[1],0);
		SquareRegion.LowLeftGoToPoint[1] = saturatemax(SquareRegion.LowLeftGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
		
		
	}
	
	//___________________________________________________________________________________
	//___________________________________if LOWRIGHT point is valid______________________
	//___________________________________________________________________________________
	if(SquareRegion.OSearch.lowr.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowr.val[0])
				break;
		
		//_____________________________Decide Point location________________________	
		SquareRegion.LowRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
		SquareRegion.LowRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/2;
		
		SquareRegion.ValidLowRightGoToPoint=1;//validate corresponding flag
		
		//__________________Saturate point coords value_____________________________
		SquareRegion.LowRightGoToPoint[0] = saturatemin(SquareRegion.LowRightGoToPoint[0],0);
		SquareRegion.LowRightGoToPoint[0] = saturatemax(SquareRegion.LowRightGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowRightGoToPoint[1] = saturatemin(SquareRegion.LowRightGoToPoint[1],0);
		SquareRegion.LowRightGoToPoint[1] = saturatemax(SquareRegion.LowRightGoToPoint[1] ,ImagesParams.NavImgSize.width-1);		
	}
	
	//___________________________________________________________________________________
	//_____________________________if BOTH LEFT are valid________________________________
	//___________________________________________________________________________________
	if(SquareRegion.OSearch.lowl.valid && SquareRegion.OSearch.topl.valid)
	{
	float alfa,x0,x1,y0,y1;	
		//__________Get current point line coord corresponding ROADWIDTH_____________
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowl.val[0])
				break;
		
		//______________________Get angle between the points_________________________
		x0=(float)SquareRegion.OSearch.lowl.val[1];
		y0=(float)(240-SquareRegion.OSearch.lowl.val[0]);
		x1=(float)SquareRegion.OSearch.topl.val[1];
		y1=(float)(240-SquareRegion.OSearch.topl.val[0]);
		alfa=(float)(180/M_PI*atan((y1-y0)/(x1-x0)));
		SquareRegion.LeftLineAngle=(float)(180/M_PI*atan2((y1-y0),(x1-x0)));
				
		//________________________Decide Goto Point__________________________________
		SquareRegion.BothLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
		SquareRegion.BothLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/2+(int)(TRW[i]/2*sin(M_PI/180*(ANGLEATSTRAIGHTLEFTLINE-alfa)));
		SquareRegion.ValidBothLeftGoToPoint=1;
	}
	
	//_____________________________________________________________________________________
	//_____________________________if BOTH RIGHT points are valid__________________________
	//_____________________________________________________________________________________
	if(SquareRegion.OSearch.lowr.valid && SquareRegion.OSearch.topr.valid)
	{
	float alfa,x0,x1,y0,y1;	
		//__________Get current point line coord corresponding ROADWIDTH_____________
		for (i=0;i<161;i++)
			if (TLN[i]==SquareRegion.OSearch.lowr.val[0])
				break;
	
		//______________________Get angle between the points_________________________
		x0=(float)SquareRegion.OSearch.lowr.val[1];
		y0=(float)(240-SquareRegion.OSearch.lowr.val[0]);
		x1=(float)SquareRegion.OSearch.topr.val[1];
		y1=(float)(240-SquareRegion.OSearch.topr.val[0]);
		alfa=(float)(180/M_PI*atan((y1-y0)/(x1-x0)));
		SquareRegion.RightLineAngle=(float)(180/M_PI*atan2((y1-y0),(x1-x0)));
		
		//________________________Decide Goto Point_________________________________ 
		SquareRegion.BothRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
		SquareRegion.BothRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/2+(int)(TRW[i]/2*sin(M_PI/180*(ANGLEATSTRAIGHTRIGHTLINE-alfa)));
		SquareRegion.ValidBothRightGoToPoint=1;
	}
	//______________________________________________________________________________________
	
	
	//__________________________________________________________________________________
	//___________________________________if LOWLEFT Closepoint is valid______________________
	//__________________________________________________________________________________
	if(SquareRegion.OSearch.lowl.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowl.val[0])
				break;
		
		//_____________________________Decide Point location________________________
		SquareRegion.LowLeftGoToPoint_MoveClose[0]=SquareRegion.OSearch.lowl.val[0];
		SquareRegion.LowLeftGoToPoint_MoveClose[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/4;
		
		SquareRegion.ValidLowLeftGoToPoint_MoveClose=1;//validate corresponding flag

		//__________________Saturate point coords value_____________________________
		SquareRegion.LowLeftGoToPoint_MoveClose[0] = saturatemin(SquareRegion.LowLeftGoToPoint_MoveClose[0],0);
		SquareRegion.LowLeftGoToPoint_MoveClose[0] = saturatemax(SquareRegion.LowLeftGoToPoint_MoveClose[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowLeftGoToPoint_MoveClose[1] = saturatemin(SquareRegion.LowLeftGoToPoint_MoveClose[1],0);
		SquareRegion.LowLeftGoToPoint_MoveClose[1] = saturatemax(SquareRegion.LowLeftGoToPoint_MoveClose[1] ,ImagesParams.NavImgSize.width-1);
		
		
	}
	
	//___________________________________________________________________________________
	//___________________________________if LOWRIGHT Close point is valid________________
	//___________________________________________________________________________________
	if(SquareRegion.OSearch.lowr.valid)
	{
		//___________Set index i for ROADWIDTH at this points line coord______________
		for (i=0;i<TLNSize;i++)
			if (TLN[i]==SquareRegion.OSearch.lowr.val[0])
				break;
		
		//_____________________________Decide Point location________________________	
		SquareRegion.LowRightGoToPoint_MoveClose[0]=SquareRegion.OSearch.lowr.val[0];
		SquareRegion.LowRightGoToPoint_MoveClose[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/4;
		
		SquareRegion.ValidLowRightGoToPoint_MoveClose=1;//validate corresponding flag
		
		//__________________Saturate point coords value_____________________________
		SquareRegion.LowRightGoToPoint_MoveClose[0] = saturatemin(SquareRegion.LowRightGoToPoint_MoveClose[0],0);
		SquareRegion.LowRightGoToPoint_MoveClose[0] = saturatemax(SquareRegion.LowRightGoToPoint_MoveClose[0] ,ImagesParams.NavImgSize.height-1);
		SquareRegion.LowRightGoToPoint_MoveClose[1] = saturatemin(SquareRegion.LowRightGoToPoint_MoveClose[1],0);
		SquareRegion.LowRightGoToPoint_MoveClose[1] = saturatemax(SquareRegion.LowRightGoToPoint_MoveClose[1] ,ImagesParams.NavImgSize.width-1);		
	}
	
	/*
	//Test for obstacle position
	if (Mode.Direction==TAKERIGHT && SquareRegion.RelocatedTopGoToPoint)
	{
	unsigned char pixelvalue;
	int col,col1=0,col2=0,col3=0;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, RobotStatus.Horizon+1, col );
			
			if (pixelvalue)
				break;
		}
		col1= col;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1,(mask_Img1->height-1)- ((mask_Img1->height-1)-(RobotStatus.Horizon+1))/2, col );
			
			if (pixelvalue)
				break;
		}
		col2= col;
		
		for (col=1;col<mask_Img1->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, mask_Img1->height-1, col );
			
			if (pixelvalue)
				break;
		}
		col3= col;
		
		SquareRegion.LeftHorizonSpace = macro_max(col1,macro_max(col2,col3));
		
		if ((col1 > 80 && col2 > 80) || (col2 > 80 && col3 > 80))
		{
			SquareRegion.ObstacleIsInnerSide = 1;
			SquareRegion.LowRightGoToPoint[0]=SquareRegion.OSearch.lowr.val[0];
			SquareRegion.LowRightGoToPoint[1]=SquareRegion.OSearch.lowr.val[1]-TRW[i]/4;
			SquareRegion.ValidLowRightGoToPoint=1;
		
			SquareRegion.LowRightGoToPoint[0] = saturatemin(SquareRegion.LowRightGoToPoint[0],0);
			SquareRegion.LowRightGoToPoint[0] = saturatemax(SquareRegion.LowRightGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
			SquareRegion.LowRightGoToPoint[1] = saturatemin(SquareRegion.LowRightGoToPoint[1],0);
			SquareRegion.LowRightGoToPoint[1] = saturatemax(SquareRegion.LowRightGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
			SquareRegion.RelocatedLowRightGoToPoint=1;
		}
	}
	else if (Mode.Direction==TAKELEFT && SquareRegion.RelocatedTopGoToPoint)
	{
	
	
	cout << "asfsdgfldfs+pgld+prflgd+pfgld+pflg" << endl;
	unsigned char pixelvalue;
	int col,col1=0,col2=0,col3=0;
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, RobotStatus.Horizon+1, col );
			
			if (pixelvalue)
			{	
				col1= col;
				break;
			}
		}
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1,(mask_Img1->height-1)- ((mask_Img1->height-1)-(RobotStatus.Horizon+1))/2, col );
			
			if (pixelvalue)
			{
				col2= col;
				break;
			}
		}
		
		for (col=mask_Img1->width-1;col>=0;col--)
		{
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, mask_Img1->height-1, col );
			
			if (pixelvalue)
			{
				col3= col;
				break;
			}
		}
		
		SquareRegion.RightHorizonSpace =  - macro_max(col1,macro_max(col2,col3));
		cout << "SquareRegion.RightHorizonSpace = " << SquareRegion.RightHorizonSpace <<endl; 
		
		if (((mask_Img1->width-1 -col1 >  80) && (mask_Img1->width-1 -col2 > 80)) || ((mask_Img1->width-1 -col2 > 80) && (mask_Img1->width-1 -col3 > 80)))
		{
			SquareRegion.ObstacleIsInnerSide = 1;
			SquareRegion.LowLeftGoToPoint[0]=SquareRegion.OSearch.lowl.val[0];
			SquareRegion.LowLeftGoToPoint[1]=SquareRegion.OSearch.lowl.val[1]+TRW[i]/4;
			SquareRegion.ValidLowLeftGoToPoint=1;
		
			SquareRegion.LowLeftGoToPoint[0] = saturatemin(SquareRegion.LowLeftGoToPoint[0],0);
			SquareRegion.LowLeftGoToPoint[0] = saturatemax(SquareRegion.LowLeftGoToPoint[0] ,ImagesParams.NavImgSize.height-1);
			SquareRegion.LowLeftGoToPoint[1] = saturatemin(SquareRegion.LowLeftGoToPoint[1],0);
			SquareRegion.LowLeftGoToPoint[1] = saturatemax(SquareRegion.LowLeftGoToPoint[1] ,ImagesParams.NavImgSize.width-1);
			SquareRegion.RelocatedLowLeftGoToPoint=1;
		}
	}*/
	
	#if POUT
		printf("___________________GoToPoints__________________*\n");
		printf("Both Top (x,y,valid?)=[%d,%d,%d]\n", SquareRegion.TopGoToPoint[0],SquareRegion.TopGoToPoint[1],SquareRegion.ValidTopGoToPoint);
		if (SquareRegion.RelocatedTopGoToPoint)
			cout << "Both Top RELOCATED." << endl;
		if (SquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Top INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Both Low (x,y,valid?)=[%d,%d,%d]\n", SquareRegion.LowGoToPoint[0],SquareRegion.LowGoToPoint[1],SquareRegion.ValidLowGoToPoint);
		if (SquareRegion.RelocatedLowGoToPoint)
			cout << "Both Low RELOCATED." << endl;
		if (SquareRegion.MaxRoadWidthExcededLowGoToPoint)
			cout << "Both Low INVALIDATED (MAXADMISSIBLEROADWIDTH exceded)." << endl;
		printf("Single LowLeft (x,y,valid?) =  [%d,%d,%d] \n",SquareRegion.LowLeftGoToPoint[0] , SquareRegion.LowLeftGoToPoint[1] , SquareRegion.ValidLowLeftGoToPoint);
		if (SquareRegion.RelocatedLowLeftGoToPoint)
			cout << "LowLeft RELOCATED." << endl;
		printf("Single LowRight (x,y,valid?) = [%d,%d,%d] \n" , SquareRegion.LowRightGoToPoint[0],SquareRegion.LowRightGoToPoint[1],SquareRegion.ValidLowRightGoToPoint);
		if (SquareRegion.RelocatedLowRightGoToPoint)
			cout << "Low Right RELOCATED." << endl;
		printf("Both Left (x,y,valid?) =  [%d,%d,%d] \n",SquareRegion.BothLeftGoToPoint[0] , SquareRegion.BothLeftGoToPoint[1] , SquareRegion.ValidBothLeftGoToPoint);
		printf("Both Right (x,y,valid?) = [%d,%d,%d] \n" , SquareRegion.BothRightGoToPoint[0],SquareRegion.BothRightGoToPoint[1],SquareRegion.ValidBothRightGoToPoint);
		printf("SquareRegion.ObstacleIsInnerSide = %d\n", SquareRegion.ObstacleIsInnerSide );
		printf("_______________________________________________\n\n");
	#endif
	
	#if IPAINT
	char str[255];
		
		sprintf(str,"Top: RW=%d < TRW*MARW=%d ",SquareRegion.TopGoToPointWidth ,SquareRegion.AdmissibleTopGoToPointWidth );	
		if (SquareRegion.MaxRoadWidthExcededTopGoToPoint)
		{
			cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-170,10),&smallfont, CV_RGB(255,0,0));
		}
		else
		{
			cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-170,10),&smallfont, CV_RGB(0,255,0));
		
		}
		
		sprintf(str,"Low: RW=%d < TRW*MARW=%d ",SquareRegion.LowGoToPointWidth ,SquareRegion.AdmissibleLowGoToPointWidth );	
		if (SquareRegion.MaxRoadWidthExcededLowGoToPoint)
		{
			cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-170,20),&smallfont, CV_RGB(255,0,0));
		}
		else
		{
			cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-170,20),&smallfont, CV_RGB(0,255,0));
		
		}
	
		
		
	
	#endif

}


