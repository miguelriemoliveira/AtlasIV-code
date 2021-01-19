/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: tunnelnav.cpp********************************************/
/***************************************************************/

#ifndef _TUNNELNAV_
#define _TUNNELNAV_

#include "header.h"

/*SIDE SENSORS VALUES CORRESPONDING DISTANCE
____________________________________________
Com Values	|	Distance
	0	|	10
	1	|	13
	2	|	16
	3	|	18
	4	|	21
	5	|	24
	6	|	27
	7	|	30
	8	|	32
	9	|	35
	10	|	38
	11	|	41
	12	|	44//needs to be updated
	13	|	46
	14	|	49
	15	|	50
______________________________________________
*/

void TN_TunnelNavegation(void)
{
char i=0;
	
	TN_GetSidesAvg();
		
	if (TunnelAnalisys.SensorDistances.LeftSideAvg<=35 && TunnelAnalisys.SensorDistances.RightSideAvg<=35)
	{
		TunnelAnalisys.Diference=TunnelAnalisys.SensorDistances.LeftSideAvg-TunnelAnalisys.SensorDistances.RightSideAvg;
		i=1;
	}
	else if (TunnelAnalisys.SensorDistances.RightSideAvg<=35)
	{
		TunnelAnalisys.Diference=(30-TunnelAnalisys.SensorDistances.RightSideAvg);
		i=2;
	}
	else if (TunnelAnalisys.SensorDistances.LeftSideAvg<=35)
	{
		TunnelAnalisys.Diference=-(30-TunnelAnalisys.SensorDistances.LeftSideAvg);
		i=3;
	}
	else if (Mode.Direction==TAKELEFT)
	{
	
		TunnelAnalisys.Diference=-50;
		i=4;
	}
	else if (Mode.Direction==TAKERIGHT)
	{
	
		TunnelAnalisys.Diference=50;
		i=5;
	}
	
	TunnelAnalisys.Diference=0.8*TunnelAnalisys.Diference;
	
	#if IPAINT
	char str[255];

	if (i==0)
		sprintf(str,"Diference [no selection info]=%g",TunnelAnalisys.Diference);
	else if (i==1)
		sprintf(str,"Diference [Both are valid]=%g",TunnelAnalisys.Diference);
	else if (i==2)
		sprintf(str,"Diference [Only right is valid]=%g",TunnelAnalisys.Diference);
	else if (i==3)
		sprintf(str,"Diference [Only left is valid]=%g",TunnelAnalisys.Diference);
	else if (i==4)
		sprintf(str,"Diference [nothing ETL]=%g",TunnelAnalisys.Diference);
	else if (i==5)
		sprintf(str,"Diference [nothing ETR]=%g",TunnelAnalisys.Diference);
		
	cvPutText(RoadImg_rgb_hough, str, cvPoint(0,225),&font, CV_RGB(0,200,255));
	
	
	#endif

}


void TN_GetSidesAvg(void)
{
	if (TunnelAnalisys.SensorDistances.FrontRight>12)
		TunnelAnalisys.SensorDistances.RightSideAvg=TunnelAnalisys.SensorDistances.FrontRight;
	else 
		TunnelAnalisys.SensorDistances.RightSideAvg=70;
		
	if (TunnelAnalisys.SensorDistances.FrontLeft>12)
		TunnelAnalisys.SensorDistances.LeftSideAvg=TunnelAnalisys.SensorDistances.FrontLeft;
	else 
		TunnelAnalisys.SensorDistances.LeftSideAvg=70;
#if IPAINT
char str[255];

	sprintf(str,"RightAvg=%g",TunnelAnalisys.SensorDistances.RightSideAvg);
	cvPutText(RoadImg_rgb_hough, str, cvPoint(0,195),&font, CV_RGB(0,200,255));
	sprintf(str,"LeftAvg=%g",TunnelAnalisys.SensorDistances.LeftSideAvg);
	cvPutText(RoadImg_rgb_hough, str, cvPoint(0,210),&font, CV_RGB(0,200,255));
	
#endif

}

void TN_GetSideSensorsDistances(void)
{
	TunnelAnalisys.SensorDistances.RightFront=Table_SideSensorsDistances[SensorValues.RightFrontFilt];
	TunnelAnalisys.SensorDistances.RightBack=Table_SideSensorsDistances[SensorValues.RightBackFilt];
	TunnelAnalisys.SensorDistances.FrontRight=Table_SideSensorsDistances[SensorValues.FrontRightFilt];
	TunnelAnalisys.SensorDistances.FrontLeft=Table_SideSensorsDistances[SensorValues.FrontLeftFilt];
	TunnelAnalisys.SensorDistances.LeftFront=Table_SideSensorsDistances[SensorValues.LeftFrontFilt];
	TunnelAnalisys.SensorDistances.LeftBack=Table_SideSensorsDistances[SensorValues.LeftBackFilt];
	TunnelAnalisys.SensorDistances.Tunnel=Table_SideSensorsDistances[SensorValues.Tunnel_Filtered];

#if IPAINT
char str[255];

	
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.RightFront);
	cvPutText(GI.Data.Info, str, cvPoint(120,45),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.RightBack);
	cvPutText(GI.Data.Info, str, cvPoint(120,60),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.FrontRight);
	cvPutText(GI.Data.Info, str, cvPoint(120,75),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.FrontLeft);
	cvPutText(GI.Data.Info, str, cvPoint(120,90),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.LeftFront);
	cvPutText(GI.Data.Info, str, cvPoint(120,105),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.LeftBack);
	cvPutText(GI.Data.Info, str, cvPoint(120,120),&font, CV_RGB(0,0,255));
	sprintf(str,"(%d)",TunnelAnalisys.SensorDistances.Tunnel);
	cvPutText(GI.Data.Info, str, cvPoint(120,135),&font, CV_RGB(0,0,255));
	
#endif

}




#endif

