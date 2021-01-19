/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/

#ifndef _NAVPREPROCESSING_
#define _NAVPREPROCESSING_

#include "header.h"


void NAVPP_NavCamPreProcessing()
{
static IplImage *mask_Img3= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U , 1);    	
//static IplImage *test5 = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U , 1);    	
	
	NAVPP_SelectHorizon2(RoadImg_gray);
	
	cvLine(RoadImg_gray,cvPoint(0,RobotStatus.Horizon),cvPoint(RoadImg_gray->width,RobotStatus.Horizon),cvScalar(255),1,8); //draw Horizon line in RoadImg_threshold
	
	
	
	cvRectangle(RoadImg_gray,cvPoint(0,RobotStatus.Horizon-1),cvPoint(RoadImg_gray->width,0),cvScalar(0),-1); //draws a black filled rectangle above the horizon; the last parameter =-1 is because we want a filled rectangle
	
 	//cvDilate( RoadImg_gray, RoadImg_gray, NULL, 5 );
 	//cvErode( RoadImg_gray, RoadImg_gray, NULL, 5 );
 	//cvNamedWindow("ola1",1);
 	//cvShowImage("ola1",RoadImg_gray);
	
	cvCopy( RoadImg_gray, RoadImg_gray_filled,NULL );
	
	NAVPP_SelectSeed(RoadImg_gray,RobotStatus.PreferedSeed.x,RobotStatus.PreferedSeed.y,&RobotStatus.seed.x,&RobotStatus.seed.y,RobotStatus.Horizon);
	
	NAVPP_GetFilledArea_a(RoadImg_gray_filled,mask_Img,RobotStatus.seed);
	cvNot(mask_Img,mask_Img);
	
	
	cvCopy( RoadImg_gray, RoadImg_gray_filled,NULL );
	NAVPP_SelectSeed_a(RoadImg_gray,RobotStatus.PreferedSeed.x,RobotStatus.PreferedSeed.y,&RobotStatus.seed_a.x,&RobotStatus.seed_a.y,RobotStatus.Horizon);
	NAVPP_GetFilledArea_c(RoadImg_gray_filled,mask_Img3,RobotStatus.seed_a);
	cvNot(mask_Img3,mask_Img3);
	
	cvLine(mask_Img3,cvPoint(1,RobotStatus.Horizon),cvPoint(mask_Img3->width-2,RobotStatus.Horizon),cvScalar(255),1,1); //draw horizon line
	
	cvOr(mask_Img,mask_Img3,mask_Img);
	
	
	
	
	
	
	cvLine(mask_Img,cvPoint(0,0),cvPoint(0,mask_Img->height-1),cvScalar(0),1,1); //draw lateral line to the left
	cvLine(mask_Img,cvPoint(mask_Img->width-1,0),cvPoint(mask_Img->width-1,mask_Img->height-1),cvScalar(0),1,1); //draw lateral line to the left
	
	cvCopy( mask_Img, FindCross_src,NULL );
	
	
//  	cvNamedWindow("ola2",1);
//  	cvShowImage("ola2",mask_Img3);
// 	
	NAVPP_GetFilledArea_b(mask_Img,mask_Img1,RobotStatus.seed1);
// 	cvNamedWindow("ola3",1);
// 	cvShowImage("ola3",mask_Img1);
	
	//cvShowImage("STEP 4->mask_Img1",mask_Img1);
	cvCvtColor(mask_Img1,RoadImg_rgb_hough,CV_GRAY2BGR);  //convert resulting image from canny to RGB (to be able to draw color indications)	
	
	
	//cvShowImage("STEP 3->mask_Img",mask_Img);
	//cvWaitKey(0);
}

void NAVPP_GetFilledArea_a(IplImage *src,IplImage *dst,CvPoint seedpoint)
{
CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here

/*cvNamedWindow("GFA-entrada", CV_WINDOW_AUTOSIZE );
cvNamedWindow("GFA-saida", CV_WINDOW_AUTOSIZE );
cvShowImage("GFA-entrada",src);*/
		cvZero(mask1_tmp);
		cvZero(mask3_tmp);
		
		
		cvFloodFill( src, seedpoint, cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
		cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
		cvGetImage(mask3_tmp,dst);
		cvNot(dst,dst);
		
		/*cvShowImage("GFA-saida",mask1_tmp);*/
		/*DestroyImage(mask1_tmp);
		DestroyMatrix(mask3_tmp);*/
		//cvWaitKey(0);
		
}

void NAVPP_GetFilledArea_b(IplImage *src,IplImage *dst,CvPoint seedpoint)
{
CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here

//cvNamedWindow("GFA-entrada", CV_WINDOW_AUTOSIZE );
//cvNamedWindow("GFA-saida", CV_WINDOW_AUTOSIZE );
//cvShowImage("GFA-entrada",src);
		cvZero(mask1_tmp);
		cvZero(mask3_tmp);
		
		
		cvFloodFill( src, seedpoint, cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
		cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
		cvGetImage(mask3_tmp,dst);
		cvNot(dst,dst);
		
		cvShowImage("GFA-saida",mask1_tmp);
		//DestroyImage(mask1_tmp);
		//DestroyMatrix(mask3_tmp);
		//cvWaitKey(0);
		
}


void NAVPP_GetFilledArea_c(IplImage *src,IplImage *dst,CvPoint seedpoint)
{
CvConnectedComp comp_tmp;
int connectivity_tmp = 4;
int new_mask_val_tmp = 255;
CvPoint seed1_tmp=cvPoint(160,2);
int flags_tmp = connectivity_tmp + (new_mask_val_tmp << 8); //weird CV_FLOODFILL_MASK_ONLY doesnt work but this number (equal to 65284) does ????
static IplImage *mask1_tmp= cvCreateImage(cvSize(src->width+2,src->height+2),8,1); //it is static so that the function is nao recursive
static CvMat *mask3_tmp=cvCreateMat(src->height,src->width,CV_8UC1); //same here

//cvNamedWindow("GFA-entrada", CV_WINDOW_AUTOSIZE );
//cvNamedWindow("GFA-saida", CV_WINDOW_AUTOSIZE );
//cvShowImage("GFA-entrada",src);
		cvZero(mask1_tmp);
		cvZero(mask3_tmp);
		
		
		cvFloodFill( src, seedpoint, cvScalar(255), cvRealScalar(20),cvRealScalar(20), &comp_tmp, flags_tmp, mask1_tmp); //fill the image
		cvGetSubRect( mask1_tmp,mask3_tmp,(CvRect)cvRect(1,1,src->width,src->height)); //copy threshold img to gray filled img
		cvGetImage(mask3_tmp,dst);
		cvNot(dst,dst);
		
		cvShowImage("GFA-saida",mask1_tmp);
		//DestroyImage(mask1_tmp);
		//DestroyMatrix(mask3_tmp);
		//cvWaitKey(0);
		
}

//Selects seed point dinamically. Basic test is, seed is in prefered seed unless this is a white pixel. Will test a new seed point going up (<y values) until a black pixel is found
void NAVPP_SelectSeed(IplImage *src,int x_in,int y_in,int *x_out,int *y_out,int Horizon)
{
int y;
*x_out=x_in;
*y_out=y_in;
	


	if (Mode.Direction==TAKELEFT)
	{
		*x_out=x_in+50;
	}
	else if (Mode.Direction==TAKERIGHT)
	{
		*x_out=x_in-50;
	}

	
	for (y = y_in; y > Horizon ; y--)
	{
		if (0==(unsigned char) cvGetReal2D( src, y, *x_out ))
		{
			*y_out=y;
			break;
		}
	}

		
	#if NPOUT
		printf("\n*****Select SeedPont Display*******\n");
		if (y!=Horizon)
		{
			printf("x_out=%d\n",*x_out);
			printf("y_out=%d\n",*y_out);
		}
		else
		{
			printf("Cannot select seed point. It's all white!!! Will have to use PreferedSeed anyway.\n");
			printf("PreferedSeed[line,col]=[%d,%d]\n",*x_out,*y_out);
			
		}	
		printf("***Select SeedPont Display END*****\n");
	#endif

}


void NAVPP_SelectSeed_a(IplImage *src,int x_in,int y_in,int *x_out,int *y_out,int Horizon)
{
int y;
*x_out=x_in;
*y_out=y_in;


	if (Mode.Direction==TAKELEFT)
	{
		*x_out=x_in-50;
	}
	else if (Mode.Direction==TAKERIGHT)
	{
		*x_out=x_in+50;
	}

	
	for (y = y_in; y > Horizon ; y--)
	{
		if (0==(unsigned char) cvGetReal2D( src, y, *x_out ))
		{
			*y_out=y;
			break;
		}
	}

		
	#if NPOUT
		printf("\n*****Select SeedPont Display*******\n");
		if (y!=Horizon)
		{
			printf("x_out=%d\n",*x_out);
			printf("y_out=%d\n",*y_out);
		}
		else
		{
			printf("Cannot select seed point. It's all white!!! Will have to use PreferedSeed anyway.\n");
			printf("PreferedSeed[line,col]=[%d,%d]\n",*x_out,*y_out);
			
		}	
		printf("***Select SeedPont Display END*****\n");
	#endif

}
void NAVPP_SelectHorizon2(IplImage *src)
{
int line,col,whiteseq=0,n_whiteseqs=0;
RobotStatus.Horizon=RobotStatus.PreferedHorizon; //initialization			
unsigned char pixelvalue;
int n;	
int onelinetouches=0, oltline=0;	
int lfblack=0;	

	col=0; //left side check
	for (line=RobotStatus.PreferedHorizon;line<src->height;line++)
	{	
		n_whiteseqs=0;
		whiteseq=0;
		n=0;
		lfblack=0;
		
		for (col=1;col<src->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			
			if (!pixelvalue && lfblack)
			{
				lfblack=0;
			}
			if (pixelvalue && !whiteseq && !lfblack)
			{	
				whiteseq=1;
				n++;
			}
			if (pixelvalue && whiteseq)
			{
				n++;
				
				
				if (n>MINIMUMLINEWIDTH && !onelinetouches)
				{
					oltline=line;
					onelinetouches=1;
				}
				if (n>MINIMUMLINEWIDTH)
				{
					n_whiteseqs++;
					lfblack=1;
					n=0;
				}
			}
			if (!pixelvalue && whiteseq)
			{
				whiteseq=0;
				n=0;
			}
		}
		
		if (n_whiteseqs>1)
			break;
			
	}
	
		if (n_whiteseqs>1)
		{
			RobotStatus.Horizon=line;
		}
		else if (onelinetouches)
			RobotStatus.Horizon=oltline;
		
	#if NPOUT
		printf("\n*****Select Horizon Display*******\n\n");
		printf("line=%d\n",line);
		printf("otlline=%d\n",oltline);
		printf("n_whiteseqs=%d\n",n_whiteseqs);
		printf("Prefered Horizon=%d\n",RobotStatus.PreferedHorizon);
		
		printf("Horizon=%d\n",RobotStatus.Horizon);	
		printf("***Select Horizon Display END*****\n\n");
	#endif
}

void NAVPP_SelectHorizon3(IplImage *src)
{
int line,col,whiteseq=0,n_whiteseqs=0;
RobotStatus.Horizon=RobotStatus.PreferedHorizon; //initialization			
unsigned char pixelvalue;
int n;	
int onelinetouches=0, oltline=0;	

	for (line=src->height-2;line>RobotStatus.PreferedHorizon;line--)
	{	
		n_whiteseqs=0;
		whiteseq=0;
		n=0;
		for (col=1;col<src->width-1;col++)
		{
		
			
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			
			
			
			
			if (pixelvalue && !whiteseq )
			{	
				whiteseq=1;
				n++;
			}
			if (pixelvalue && whiteseq)
			{
				n++;
				
				if (n>MINIMUMLINEWIDTH)
				{
					n_whiteseqs++;
					n=0;
					if (col<src->width-1-2*MINIMUMLINEWIDTH)
					{
					col+=2*MINIMUMLINEWIDTH;
					}
					else 
						break;
				}
			}
			if (!pixelvalue && whiteseq)
			{
				whiteseq=0;
				n=0;
			}
			
			
		}
		
		
			printf("col=%d\n",col);
			printf("line=%d\n",line);
			printf("n_whiteseqs=%d\n",n_whiteseqs);
			//cvWaitKey(0);
		if (n_whiteseqs==0 && col>src->width-1-2*MINIMUMLINEWIDTH)
		{	
			RobotStatus.PreferedHorizon=line+1;
			break;
		}
	}




	
	col=0; //left side check
	for (line=RobotStatus.PreferedHorizon;line<LINETOSTARTLOWSEARCH-1;line++)
	{	
		n_whiteseqs=0;
		whiteseq=0;
		n=0;
		
		for (col=1;col<src->width-1;col++)
		{
			pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
			
			
			if (pixelvalue && !whiteseq )
			{	
				whiteseq=1;
				n++;
			}
			if (pixelvalue && whiteseq)
			{
				n++;
				
				
				if (n>MINIMUMLINEWIDTH && !onelinetouches)
				{
					oltline=line;
					onelinetouches=1;
				}
				if (n>MINIMUMLINEWIDTH)
				{
					n_whiteseqs++;
					n=0;
					if (col<src->width-1-2*MINIMUMLINEWIDTH)
					{
					col+=2*MINIMUMLINEWIDTH;
					}
					else 
						break;
				}
			}
			if (!pixelvalue && whiteseq)
			{
				whiteseq=0;
				n=0;
			}
		}
		
		if (n_whiteseqs>1)
			break;
			
	}
	
		if (n_whiteseqs>1)
		{
			RobotStatus.Horizon=line;
		}
		else if (onelinetouches)
			RobotStatus.Horizon=oltline;

					
	#if NPOUT
		printf("\n*****Select Horizon Display*******\n\n");
		printf("line=%d\n",line);
		printf("otlline=%d\n",oltline);
		printf("n_whiteseqs=%d\n",n_whiteseqs);
		printf("Prefered Horizon=%d\n",RobotStatus.PreferedHorizon);
		
		printf("Horizon=%d\n",RobotStatus.Horizon);	
		printf("***Select Horizon Display END*****\n\n");
	#endif
//cvWaitKey(0);	

}





#endif
