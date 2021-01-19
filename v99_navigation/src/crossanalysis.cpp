/***************************************************************************
 *   Copyright (C) 2006 by Miguel Armando Riem de Oliveira                 *
 *   mike@mecn096                                                          *
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

 
#ifndef _CROSSANALYSIS_
#define _CROSSANALYSIS_

#include "header.h"
 
/**@brief Cross finding function version 5. Uses ocupation analysis to decide where to search for seed point.
 * Searches cross starting from the image top
 * @param src pointer to source image
 * @param hor horizon
 */
void NAVP_FindCross5(IplImage *src,int hor)
{
int col=0,line=0;
int endline=0,endcol=src->width-5;
int a=77,tobreak=0;;
int itsacross=0;

char str[255];

double dpixelvalue=0;
unsigned int npts=0;		
CvPoint pt0;


int ab;


//cout << endl << "Ocupation.N[Ocupation.BiggestPercentageArea].Area[1].y" << Ocupation.N[Ocupation.BiggestPercentArea].Area[1].y<< endl;

int startline=Ocupation.N[Ocupation.BiggestPercentArea].Area[1].y;

for (CrossAnalisys.AttemptedSpots=0 ;CrossAnalisys.AttemptedSpots<3; CrossAnalisys.AttemptedSpots++)
{	
	ab = cvCountNonZero( GI.Cross.InnerSpotsImg);
	//cout << "ab" << ab << endl;
	
	if (ab>4000)
	{
		
		CrossAnalisys.PixelNum=0;
		tobreak=0;
		cvClearSeq(CrossAnalisys.pts);	
		
		
	// 	TM_UpdateTick(&Time.Tick.Tick3);
		
//    		cvNamedWindow("FIND InnerSpotsImgWithSpots",1);
//    		cvShowImage("FIND InnerSpotsImgWithSpots",GI.Cross.InnerSpotsImg);		
		
		
		
		//update InnerSpotsMat
		cvGetMat(GI.Cross.InnerSpotsImg,GI.Cross.test2);
 		cvConvertScale( GI.Cross.test2, GI.Cross.InnerSpotsMat, 1.0/255.0, 0 );
		
		
// 		cvNamedWindow("FIND InnerSpotsMATWithSpots",1);
//    		cvShowImage("FIND InnerSpotsMATWithSpots",GI.Cross.InnerSpotsMat);		
		for (line=startline;line<Ocupation.N[Ocupation.BiggestPercentArea].Area[0].y;line++)//Look for a white pixel to serve as a seed point
		{
			for (col=200;col<GI.Cross.InnerSpotsMat->width-16;col=col+5)
			{
				dpixelvalue = cvmGet(GI.Cross.InnerSpotsMat,line,col);
				//cout << "dpixelvalue[" <<  line << "," << col<< "] = " << dpixelvalue << endl;
				//pixelvalue=(unsigned char) cvGetReal2D(InnerSpotsImg,line,col);
				if (dpixelvalue)
				{
					tobreak=1;
					endcol=col;
					endline=line;
					break;
				}
			}
			
			if (tobreak)
			{
				startline=line+3;
				break;
			}
			
			
		}
		//printf("endline=%d  endcol=%d\n",endline,endcol);
		
// 		printf("Ocupation.N[Ocupation.BiggestPercentArea].Area[0].y = %d  || GI.Cross.InnerSpotsMat->width-16 = %d\n",Ocupation.N[Ocupation.BiggestPercentArea].Area[0].y-1 ,GI.Cross.InnerSpotsMat->width-16-1);
// 		printf("line=%d  || col=%d\n",line,col);
		
		if (col >= GI.Cross.InnerSpotsMat->width-16-1 && line >= Ocupation.N[Ocupation.BiggestPercentArea].Area[0].y-1)break;
 		
		NAP_GetSpot(GI.Cross.InnerSpotsImg, GI.Cross.CurrentSpotsImg, endline, endcol);
		cvGetMat(GI.Cross.CurrentSpotsImg,GI.Cross.test1);
 		cvConvertScale( GI.Cross.test1, GI.Cross.CurrentSpotsMat, 1.0/255.0, 0 );
	
		CrossAnalisys.PixelNum = cvCountNonZero( GI.Cross.CurrentSpotsImg);
	 	printf("PixelNum=%d and MINIMUMCROSSSIZE=%d\n",CrossAnalisys.PixelNum,MINIMUMCROSSSIZE);
	
		if (CrossAnalisys.PixelNum>MINIMUMCROSSSIZE/*to make sure its not just an isolated pix*/ && CrossAnalisys.PixelNum<10000)
		{
			npts=0;
			
			TM_UpdateTick(&Time.Tick.Tick3);
			for (line=hor+1;line<GI.Cross.CurrentSpotsImg->height;line++)
			{
				for (col=0;col<GI.Cross.CurrentSpotsImg->width;col++)
				{	
					dpixelvalue = cvmGet(GI.Cross.CurrentSpotsMat,line,col);
					//dpixelvalue = cvGetReal2D(CurrentSpotsImg ,line ,col );
					
					if (dpixelvalue)
					{
						pt0=cvPoint(col,line);
						npts++;
						cvSeqPush(CrossAnalisys.pts,&pt0); 
					}
				}
			}
// 			TM_UpdateTick(&Time.Tick.Tick4);
// 			cout << "normal cicle: " << (Time.Tick.Tick4-Time.Tick.Tick3)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;
// 			
// 			cvWaitKey(0);
// 			
		//TM_UpdateTick(&Time.Tick.Tick4);
		
		//cout << "seqpushgeral: " << (Time.Tick.Tick4-Time.Tick.Tick3)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;
// 		cout << "totaalpts = " << line+col << "  used pts = " << npts << endl; 	
// 		
			//TM_UpdateTick(&Time.Tick.Tick1);
			
			
			
			SetCrossAnalysisStruct(cvMinAreaRect2( CrossAnalisys.pts, NULL ));			
		
		//	TM_UpdateTick(&Time.Tick.Tick2);
		//cout << "MAR calculation: " << (Time.Tick.Tick2-Time.Tick.Tick1)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl << endl << endl << endl;	
		
		
		//cvWaitKey(0);
// 		cvNamedWindow("GI.Cross.CurrentSpotsImg",1);
// 		cvShowImage("GI.Cross.CurrentSpotsImg",GI.Cross.CurrentSpotsImg);
// 			
		cout << "CrossAnalisys.HeightWidthRatio" << CrossAnalisys.HeightWidthRatio << endl;
		cout << "CrossAnalisys.MARAngle" << CrossAnalisys.MARAngle << endl;
// 		cvWaitKey(0);
			if (CrossAnalisys.HeightWidthRatio<0.37 && (CrossAnalisys.MARAngle<10.0 && CrossAnalisys.MARAngle>-30.0))
			{
				cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[0],CrossAnalisys.MARectangle.boxpts[1],cvScalar(128));
				cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[1],CrossAnalisys.MARectangle.boxpts[2],cvScalar(128));
				cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[2],CrossAnalisys.MARectangle.boxpts[3],cvScalar(128));
				cvLine(cross_Img,CrossAnalisys.MARectangle.boxpts[3],CrossAnalisys.MARectangle.boxpts[0],cvScalar(128));
				itsacross=1;
				cvShowImage("OTHER ->cross_Img",cross_Img);
				break;	
			}
		}
	}	
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
 
 

void SetCrossAnalysisStruct(CvBox2D box)
{
CvPoint2D32f pt[4];
int distancia1=0,distancia2=0;
float angulo1;			
int t;			
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
			
			CrossAnalisys.MARMaxLineVal = macro_max(CrossAnalisys.MARectangle.boxpts[0].y , macro_max( macro_max(CrossAnalisys.MARectangle.boxpts[1].y , CrossAnalisys.MARectangle.boxpts[2].y), CrossAnalisys.MARectangle.boxpts[3].y) );
			
			
			
			CrossAnalisys.HeightWidthRatio=((float)macro_min(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height))/((float)macro_max(CrossAnalisys.MARectangle.width,CrossAnalisys.MARectangle.height));
			
			
			
			distancia1= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)*(CrossAnalisys.MARectangle.boxpts[1].x-CrossAnalisys.MARectangle.boxpts[0].x)+(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y)*(CrossAnalisys.MARectangle.boxpts[1].y-CrossAnalisys.MARectangle.boxpts[0].y));
			
			distancia2= (int)sqrt( (CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)*(CrossAnalisys.MARectangle.boxpts[2].x-CrossAnalisys.MARectangle.boxpts[1].x)+(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y)*(CrossAnalisys.MARectangle.boxpts[2].y-CrossAnalisys.MARectangle.boxpts[1].y));
			
			
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
			
			cout << "CrossAnalisys.MARMaxLineVal " << CrossAnalisys.MARMaxLineVal << endl;	
			printf("width do MAR=%d\n",CrossAnalisys.MARectangle.width);
			printf("height do MAR=%d\n",CrossAnalisys.MARectangle.height);
			printf("Area do MAR=%d\n",CrossAnalisys.MARectangle.Area);
			printf("HWratio=%f\n",CrossAnalisys.HeightWidthRatio);
			printf("Angle=%f\n",CrossAnalisys.MARAngle);
			printf("Yval=%d\n",CrossAnalisys.CrossYVal);
			
			
			cvWaitKey(0);*/










}

void NAVP_IsolateInnerSpots(IplImage *src, IplImage* pre_dst2)
{
static IplImage *pre_dst= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static IplImage *pre_dst1= cvCreateImage(cvSize(src->width,src->height),8,1); //it is static so that the function is nao recursive
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here

CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
//CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????





	cvRectangle(pre_dst,cvPoint(0,0),cvPoint(pre_dst->width,pre_dst->height),cvScalar(0),-1); //to white the image
	cvCopy(src,pre_dst);



	TM_UpdateTick(&Time.Tick.Tick1);			
	//______________________Used to isolate inner road spots___________________________
	cvLine(pre_dst,cvPoint(0,RobotStatus.Horizon),cvPoint(src->width,RobotStatus.Horizon),cvScalar(0));
	cvFloodFill( pre_dst, cvPoint(3,3), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); 
	cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
	cvGetImage(mask3_tmp,pre_dst1);
	cvOr(pre_dst1,pre_dst,pre_dst2,NULL);
	cvNot(pre_dst2,pre_dst2);
	cvGetMat(pre_dst2,GI.Cross.test2);
 	cvConvertScale( GI.Cross.test2, GI.Cross.InnerSpotsMat, 1.0/255.0, 0 );
	//_________________________________________________________________________________
	
	//______________________Clear static images ___________________________
	cvZero(pre_dst);
	cvZero(pre_dst1);
	cvZero(mask1_tmp);
	cvZero(mask3_tmp);
	cvZero(GI.Cross.test1);
	//_____________________________________________________________________

	
//cvReleaseImage(&pre_dst);
//cvReleaseImage(&pre_dst1);
//cvReleaseImage(&mask1_tmp);
//cvReleaseMat(&mask3_tmp);
}




void NAP_GetSpot(IplImage* src, IplImage* dst, int endline, int endcol)
{
static IplImage *pre_dst= cvCreateImage(cvSize(src->width,src->height),8,1); //it
static IplImage *pre_dst1= cvCreateImage(cvSize(src->width,src->height),8,1); //it
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here


//static IplImage *teste= cvCreateImage(cvSize(src->width,src->height),8,1); //it
CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
//CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????


//	cvZero(teste);
	cvZero(pre_dst);
	cvZero(pre_dst1);
	cvZero(mask1_tmp);
	cvZero(mask3_tmp);
	//cvZero(dst);	

	cvCopy(src,pre_dst);
	//cvCopy(pre_dst,teste);
 	//int cor =129;
	//cvLine(teste,cvPoint(endcol-4,endline),cvPoint(endcol+4,endline),cvScalar(120),1,1); 
	//cvLine(teste,cvPoint(endcol,endline-4),cvPoint(endcol,endline+4),cvScalar(120),1,1); 
	
	//cvNamedWindow("NAP_GetSpotteste",1);
 	//cvShowImage("NAP_GetSpotteste",teste);
	
	cvFloodFill( pre_dst, cvPoint(endcol,endline), cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
	cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
	cvGetImage(mask3_tmp,dst);
	
	cvNot(dst,pre_dst1);
	cvAnd(src,pre_dst1,src,NULL);
}
#endif
