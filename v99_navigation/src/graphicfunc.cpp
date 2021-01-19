/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file graphicfunc.cpp
* @brief All the drawing functions are here defined. Theoreticaly, if one would not use this codes the program would still work.
*
* Here is an example of the console of windows when the program is running.
* @image html "../images/navimg1.jpg" "Navegation, Lights and Data windows"
*/
#ifndef _GRAPHICFUNC_
#define _GRAPHICFUNC_

#include "header.h"

/** @brief Draws everything that is supposed to draw.
 *
 */
void GF_DrawAll(void)
{
char str[255];			

	cvLine(RoadImg_rgb_hough,cvPoint(0,RobotStatus.PreferedHorizon),cvPoint(RoadImg_rgb_hough->width,RobotStatus.PreferedHorizon),CV_RGB(Color.orange[0],Color.orange[1],Color.orange[2]),1,1);//draw prefered horizon 
	GF_DrawX(RoadImg_rgb_hough,RobotStatus.seed.y,RobotStatus.seed.x,5,Color.orange);
	GF_DrawX(RoadImg_rgb_hough,RobotStatus.seed_a.y,RobotStatus.seed_a.x,5,Color.orange);
	GF_Join_GoToPoint_Present(RoadImg_rgb_hough,&SquareRegion,Color.orange);
	
	
	GF_DrawCross(RoadImg_rgb_hough,RobotStatus.DA_PP.BothTopLineSteping.LowRight[0],RobotStatus.DA_PP.BothTopLineSteping.LowRight[1],15,Color.orange);
	GF_DrawCross(RoadImg_rgb_hough,RobotStatus.DA_PP.BothTopLineSteping.TopRight[0],RobotStatus.DA_PP.BothTopLineSteping.TopRight[1],3,Color.orange);
	GF_DrawCross(RoadImg_rgb_hough,RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0],RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1],3,Color.orange);
	GF_DrawCross(RoadImg_rgb_hough,RobotStatus.DA_PP.BothTopLineSteping.TopLeft[0],RobotStatus.DA_PP.BothTopLineSteping.TopLeft[1],3,Color.orange);
	
	
	GF_DrawArea(cross_Img, RoadImg_rgb_hough,RobotStatus.Horizon, Color.magenta);
	GF_DrawBoxPoints(RoadImg_rgb_hough,&SquareRegion ,Color.cyan,Color.cyan);
	GF_DrawGoToPoints(RoadImg_rgb_hough,&SquareRegion,Color.green,Color.yellow,Color.magenta);

	#if USECONSTRUCTIONMODE
		GF_DrawBoxPoints(RoadImg_rgb_hough,&ConstSquareRegion ,Color.blue,Color.blue);
		GF_DrawGoToPoints(RoadImg_rgb_hough,&ConstSquareRegion,Color.darkgreen,Color.darkyellow,Color.darkmagenta);
		sprintf(str,"C.All %d",(int )RobotStatus.PinsPixNumAll);
		cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-70,40),&smallfont, CV_RGB(128,255,0));
		sprintf(str,"C.Down %d",(int )RobotStatus.PinsPixNumDown);
		cvPutText(RoadImg_rgb_hough, str, cvPoint(ImagesParams.NavImgSize.width-70,50),&smallfont,CV_RGB(128,255,0));	
	#endif
	

	for (int i=0;i<8;i++)
	{
		cvLine( RoadImg_rgb_hough, Ocupation.N[i].Area[0],cvPoint(Ocupation.N[i].Area[0].x+20,Ocupation.N[i].Area[0].y), CV_RGB(255,255,0),1, 8, 0 );
		sprintf(str,"%d",(int) Ocupation.N[i].AreaOcupationPercent);
		if (Ocupation.BiggestPercentArea==i)
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,Ocupation.N[i].Area[0].y-5),&smallfont, CV_RGB(255,0,0));	
		else
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,Ocupation.N[i].Area[0].y-5),&smallfont, CV_RGB(255,255,0));	
	}	
	
}

/** @brief Draws everything in the SandV_Filter.
 *
 */
void GF_DrawAll_SandV(void)
{

	cvLine(GI.Lights.SandV_Filter,cvPoint(0,RobotStatus.PreferedHorizon),cvPoint(RoadImg_rgb_hough->width,RobotStatus.PreferedHorizon),CV_RGB(Color.orange[0],Color.orange[1],Color.orange[2]),1,1);//draw prefered horizon 
	GF_DrawX(GI.Lights.SandV_Filter,RobotStatus.seed.y,RobotStatus.seed.x,5,Color.orange);
	GF_Join_GoToPoint_Present(RoadImg_rgb_hough,&SquareRegion,Color.orange);
	GF_DrawCross(GI.Lights.SandV_Filter,RobotStatus.DA_PP.BothTopLineSteping.LowRight[0],RobotStatus.DA_PP.BothTopLineSteping.LowRight[1],3,Color.orange);
	GF_DrawCross(GI.Lights.SandV_Filter,RobotStatus.DA_PP.BothTopLineSteping.TopRight[0],RobotStatus.DA_PP.BothTopLineSteping.TopRight[1],3,Color.orange);
	GF_DrawCross(GI.Lights.SandV_Filter,RobotStatus.DA_PP.BothTopLineSteping.LowLeft[0],RobotStatus.DA_PP.BothTopLineSteping.LowLeft[1],3,Color.orange);
	GF_DrawCross(GI.Lights.SandV_Filter,RobotStatus.DA_PP.BothTopLineSteping.TopLeft[0],RobotStatus.DA_PP.BothTopLineSteping.TopLeft[1],3,Color.orange);
	
	GF_DrawBoxPoints(GI.Lights.SandV_Filter,&SquareRegion ,Color.cyan,Color.cyan);
	GF_DrawGoToPoints(GI.Lights.SandV_Filter,&SquareRegion,Color.green,Color.yellow,Color.magenta);



}


/**@brief Draws a cross in the src image.
 * 
 * @param src pointer to the source image.
 * @param line pixel line index for the cross's center.
 * @param column pixel column index for the cross's center.
 * @param lenght cross lenght.
 * @param color cross color.
 */
void GF_DrawCross(IplImage *src,int line,int column,int lenght, int *color)
{
	cvLine(src,cvPoint(column-lenght,line),cvPoint(column+lenght,line),CV_RGB(color[0],color[1],color[2]),1,1); 
	cvLine(src,cvPoint(column,line-lenght),cvPoint(column,line+lenght),CV_RGB(color[0],color[1],color[2]),1,1); 
}

/**@brief Draws an X in the src image.
 * 
 * @param src pointer to the source image.
 * @param line pixel line index for the X's center.
 * @param column pixel column index for the X's center.
 * @param lenght X's lenght.
 * @param color X's color.
 */
void GF_DrawX(IplImage *src,int line,int column,int lenght, int *color)
{
	cvLine(src,cvPoint(column-lenght,line-lenght),cvPoint(column+lenght,line+lenght),CV_RGB(color[0],color[1],color[2]),1,1); 
	cvLine(src,cvPoint(column+lenght,line-lenght),cvPoint(column-lenght,line+lenght),CV_RGB(color[0],color[1],color[2]),1,1); 
}



/**@brief Draws the box points.
 * 
 * @param src pointer to the source image.
 * @param ST SquareTest struct.
 * @param Msearchcolor color to draw Msearch pixels.
 * @param Osearchcolor color to draw Osearch pixels.
 */
void GF_DrawBoxPoints(IplImage *src,SquareTest *ST,int *Msearchcolor,int *Osearchcolor)
{
	if (ST->MSearch.lowl.valid)
		GF_DrawCross(src,ST->MSearch.lowl.val[0],ST->MSearch.lowl.val[1],5,Msearchcolor);
	if (ST->MSearch.topl.valid)
		GF_DrawCross(src,ST->MSearch.topl.val[0],ST->MSearch.topl.val[1],5,Msearchcolor);
	if (ST->MSearch.lowr.valid)
		GF_DrawCross(src,ST->MSearch.lowr.val[0],ST->MSearch.lowr.val[1],5,Msearchcolor);
	if (ST->MSearch.topr.valid)
		GF_DrawCross(src,ST->MSearch.topr.val[0],ST->MSearch.topr.val[1],5,Msearchcolor);
	
	
	
	if (ST->OSearch.lowl.valid)
		GF_DrawCross(src,ST->OSearch.lowl.val[0],ST->OSearch.lowl.val[1],5,Osearchcolor);	
	if (ST->OSearch.topl.valid)
		GF_DrawCross(src,ST->OSearch.topl.val[0],ST->OSearch.topl.val[1],5,Osearchcolor);
	if (ST->OSearch.lowr.valid)
		GF_DrawCross(src,ST->OSearch.lowr.val[0],ST->OSearch.lowr.val[1],5,Osearchcolor);
	if (ST->OSearch.topr.valid)
		GF_DrawCross(src,ST->OSearch.topr.val[0],ST->OSearch.topr.val[1],5,Osearchcolor);

}

/**@brief Draws go to points.
 * 
 * @param src pointer to the source image.
 * @param ST SquareTest struct.
 * @param both_tl_color color for this point.
 * @param singlecolor color for this point.
 * @param both_lr_color color for this point.
 */
void GF_DrawGoToPoints(IplImage *src,SquareTest *ST,int *both_tl_color,int *singlecolor,int *both_lr_color)
{
	//cvLine(RoadImg_rgb_hough,SquareRegion.PresentPoint,SquareRegion.PresentPoint,CV_RGB(128,0,0),3,4); //draw red present point 
	GF_DrawCross(src,ST->PresentPoint.y,ST->PresentPoint.x,5,both_tl_color);
	
	
	
	if (ST->ValidLowGoToPoint)
	{
		GF_DrawCross(src,ST->LowGoToPoint[0],ST->LowGoToPoint[1],5,both_tl_color);
		if (SquareRegion.RelocatedLowGoToPoint)	
			cvPutText(src, "R", cvPoint(ST->LowGoToPoint[1],ST->LowGoToPoint[0]) ,&smallfont ,CV_RGB(255,0,0));
	}
	if (ST->ValidTopGoToPoint)
	{
		GF_DrawCross(src,ST->TopGoToPoint[0],ST->TopGoToPoint[1],5,both_tl_color);
		if (SquareRegion.RelocatedTopGoToPoint)	
			cvPutText(src, "R", cvPoint(ST->TopGoToPoint[1],ST->TopGoToPoint[0]) ,&smallfont ,CV_RGB(255,0,0));
	}
	if (ST->ValidLowLeftGoToPoint)
	{
		 GF_DrawCross(src,ST->LowLeftGoToPoint[0],ST->LowLeftGoToPoint[1],5,singlecolor);
		 if (SquareRegion.RelocatedLowLeftGoToPoint)	
			cvPutText(src, "R", cvPoint(ST->LowLeftGoToPoint[1],ST->LowLeftGoToPoint[0]) ,&smallfont ,CV_RGB(255,0,0));
	}
	if (ST->ValidLowRightGoToPoint)
	{
		GF_DrawCross(src,ST->LowRightGoToPoint[0],ST->LowRightGoToPoint[1],5,singlecolor);
		if (SquareRegion.RelocatedLowRightGoToPoint)	
			cvPutText(src, "R", cvPoint(ST->LowRightGoToPoint[1],ST->LowRightGoToPoint[0]) ,&smallfont ,CV_RGB(255,0,0));
	}
	if (ST->ValidBothRightGoToPoint)
	{
		GF_DrawCross(src,ST->BothRightGoToPoint[0],ST->BothRightGoToPoint[1],5,both_lr_color);	
	}
	if (ST->ValidBothLeftGoToPoint)
	{	
		GF_DrawCross(src,ST->BothLeftGoToPoint[0],ST->BothLeftGoToPoint[1],5,both_lr_color);
	}
	if (ST->ValidLowLeftGoToPoint_MoveClose)
	{	 
		int colorMoveClose[3] ={100,80,100};
		 GF_DrawCross(src,ST->LowLeftGoToPoint_MoveClose[0],ST->LowLeftGoToPoint_MoveClose[1],5,colorMoveClose);
	}
	if (ST->ValidLowRightGoToPoint_MoveClose)
	{
		int colorMoveClose[3] ={100,80,100};
		 GF_DrawCross(src,ST->LowRightGoToPoint_MoveClose[0],ST->LowRightGoToPoint_MoveClose[1],5,colorMoveClose);
	}
	
}

/**@brief Dont rememer???
 * 
 * @param mask 
 * @param dst 
 * @param hor 
 * @param color 
 */
void GF_DrawArea(IplImage *mask, IplImage *dst,int hor, int *color)
{
int line,col;
unsigned char pixelvalue_mask;

	for (line=hor+1;line<mask->height-1;line++) //dont need to waist time with black erased part  
	{	
		for (col=0;col<mask->width-1;col++)
		{		
			pixelvalue_mask=(unsigned char) cvGetReal2D( mask, line, col );
			
			if (pixelvalue_mask) 
				GF_DrawCross(dst,line,col,0/*draw just the point*/, color);			
		}	
	}	
}

/**@brief Draws a line that unites the  go to point and the present point.
 * 
 * @param src pointer to the source image.
 * @param ST SquareTest struct.
 * @param Color line color.
 */
void GF_Join_GoToPoint_Present(IplImage *src,SquareTest *ST,int *Color)
{
	cvLine(src,cvPoint(ST->PresentPoint.x,ST->PresentPoint.y),cvPoint(RobotStatus.ChosenGoToPoint[1],RobotStatus.ChosenGoToPoint[0]),CV_RGB(Color[0],Color[1],Color[2]),1,1);//draw prefered
}

/**@brief Write various flags to the data window.
 * 
 */
void GF_WriteFlags()
{
char str[255];
	
	cvLine(GI.Data.Info,cvPoint(478,15),cvPoint(478,200),CV_RGB(255,255,0),1,8);
	cvPutText(GI.Data.Info, "_____FLAGS____", cvPoint(480,15),&font, CV_RGB(255,255,0));
	
	if (USELIGHTSREADING)
		cvPutText(GI.Data.Info, "Lights: Enabled", cvPoint(480,30),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Lights: Disabled", cvPoint(480,30),&font, CV_RGB(255,0,0));
	if (USETUNNELANALISYS)
		cvPutText(GI.Data.Info, "Tunnel: Enabled", cvPoint(480,45),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Tunnel: Disabled", cvPoint(480,45),&font, CV_RGB(255,0,0));
	if (USESIGNALS)
		cvPutText(GI.Data.Info, "Signals: Enabled", cvPoint(480,60),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Signals: Disabled", cvPoint(480,60),&font, CV_RGB(255,0,0));
	if (USEPIC1COMS)
		cvPutText(GI.Data.Info, "Pic 1: Enabled", cvPoint(480,75),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Pic 1: Disabled", cvPoint(480,75),&font, CV_RGB(255,0,0));
	if (USEPIC2COMS)
		cvPutText(GI.Data.Info, "Pic 2: Enabled", cvPoint(480,90),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Pic 2: Disabled", cvPoint(480,90),&font, CV_RGB(255,0,0));
	if (USECONSTRUCTIONMODE)
		cvPutText(GI.Data.Info, "Const.: Enabled", cvPoint(480,105),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "Const.: Disabled", cvPoint(480,105),&font, CV_RGB(255,0,0));
	if (USEDISTANTLIGHTSREADING)
		cvPutText(GI.Data.Info, "D. Lights: Enabled", cvPoint(480,120),&font, CV_RGB(0,255,0));	
	else 
		cvPutText(GI.Data.Info, "D. Lights: Disabled", cvPoint(480,120),&font, CV_RGB(255,0,0));
		
	sprintf(str,"Lap %d of %d",RobotStatus.LapNumber,TOTALLAPS);		
	cvPutText(GI.Data.Info, str, cvPoint(480,135),&font, CV_RGB(255,255,0));
	
	if (Mode.Direction==TAKELEFT)
		cvPutText(GI.Data.Info, "Take: Left", cvPoint(480,150),&font, CV_RGB(255,255,0));
	else if (Mode.Direction==TAKERIGHT)
		cvPutText(GI.Data.Info, "Take: Right", cvPoint(480,150),&font, CV_RGB(255,255,0));
	else if ((Mode.Direction==NONE))
		cvPutText(GI.Data.Info, "Take: None", cvPoint(480,150),&font, CV_RGB(255,255,0));
		
}

/**@brief Shows images in the proper windows.
 * 
 */
void GF_ShowImages()
{
	
	cvShowImage("STEP 1->Original_Img",RoadImg_gray);
	cvShowImage("LightsCamera",LightsImg_Original);
 	cvShowImage("S_Filter",GI.Lights.S_Filter);
 	cvShowImage("V_Filter",GI.Lights.V_Filter);
 	cvShowImage("SandV_Filter",GI.Lights.SandV_Filter);

	cvShowImage("Red_Filter",GI.Lights.Red_Filter);
	cvShowImage("Green_Filter",GI.Lights.Green_Filter);
	cvShowImage("Yellow_Filter",GI.Lights.Yellow_Filter);
	
	cvShowImage("LightsFinal",img1);			
	cvShowImage("STEP 5->Final Image",RoadImg_rgb_hough);
	cvShowImage("OTHER ->cross_Img",cross_Img);	
	cvShowImage("Robot Data",GI.Data.Info);			
	cvShowImage("STEP 4->mask_Img1",mask_Img1);
/*	cvNamedWindow("Const_S_Filter",1);
	cvResizeWindow("Const_S_Filter", 320, 240);
	cvNamedWindow("Const_V_Filter",1);
	cvResizeWindow("Const_V_Filter", 320, 240);
	cvNamedWindow("Const_Orange",1);
	cvResizeWindow("Const_Orange", 320, 240 );


	cvNamedWindow("Const_Pins",1);
	cvResizeWindow("Const_Pins", 320, 240 );
	cvShowImage("Const_S_Filter",GI.Const.S_Filter);
	cvShowImage("Const_V_Filter",GI.Const.V_Filter);
	cvShowImage("Const_Orange",GI.Const.Orange);
	
	
 	cvShowImage("Const_Pins",GI.Const.Pins);*/
}
#endif
