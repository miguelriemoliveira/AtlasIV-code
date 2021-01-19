/***************************************************************/
/**TITLE: ROBOT navegation main src file************************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file main.cpp
* @brief Program main.
*/

#ifndef _MAIN_FILE_
#define _MAIN_FILE_

#include "header.h"

/*
SPEED	|	LEFT	|	RIGHT
3	|	60	|	33
4	|	60	|	33
5	|	60	|	33
6	|	60	|	33
7	|	60	|	33
8	|	60	|	33
9	|	60	|	33
10	|	60	|	33	
*/

int main(int argc, char** argv)
{


IN_InitEverything(0/*time to show logo (0 not to show)*/); //Every initialization needed is done here

Mode.Direction=TAKELEFT;
Mode.Mode=NORMALMODE;
Mode.PreviousMode=DUMMYMODE;

while(1)	//infinite main cicle (its breaked out of error or the end of run)
{
	
	TM_UpdateTick(&Time.Tick.LastIteration);
	
	
	AF_GetNavImage();		
	
	TM_UpdateTick(&Time.Tick.Tick7);
	NAVPP_NavCamPreProcessing();
	TM_UpdateTick(&Time.Tick.Tick8);
	cout << "PreProcessing: " << (Time.Tick.Tick8-Time.Tick.Tick7)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;
	
	
	
	
	NAVP_IsolateInnerSpots(FindCross_src, GI.Cross.InnerSpotsImg);
	
	
	NAVP_GetOcupation(GI.Cross.InnerSpotsImg);
	NAVP_FindCross5(FindCross_src,RobotStatus.Horizon);
	NAVP_NavCamProcessing();
	
	
		
  	AF_GetLightsImage();	
 	RL_ApplyFilter_1(1/*1=Saturationfilter*/);	
	RL_ApplyFilter_1(2/*2=Intensityfilter*/);	
	RL_MergeSandVFilter();	
	RL_ColorRecognition_1();
	RL_ShapeRecognition();
	RL_COLORandShapeMerge();	
// // 	
// 	
	
	
	
	AF_SpeedControl();
	AF_DirectionControl_1(&SquareRegion); //chooses one DA from all of them	
	AF_ConvertDA(Table_DirToSend, Table_DirAngle);
	
	
	

	//CN_FindPins();
	/*CN_DetectPins();
	CN_FindPixelsLFO(SandV_Filter,220,140);
	NAVP_GetGoToPoint(Table_LineNumber,Table_RoadWidth);
	NAVP_GetDAbyPP();*/
				
	//CN_DrawPins();
	GF_DrawAll();
	
	
	#if USESIGNALS
		
		AF_LightsControl();
		SignalsState.HeadLights=1;
	#endif
	
	
	
	//cvSaveImage( "NavCamObstacle_1.jpg", RoadImg_gray);
	//cvSaveImage( "NavCamObstacle_Filled_1.jpg", RoadImg_hough);
	//break;
	TM_UpdateSince();
	//AF_ApplyTunnelSensorFilter();
	//TN_GetSideSensorsDistances();
// 	switch (Mode.Mode)
// 	{
// 		
// 		case DUMMYMODE:  //____________________DUMMY MODE_________________________________	
// 		{
// 		break;
// 		}
// 		case NORMALMODE:  //____________________NORMAL MODE_______________________________	
// 		{
// 			
// 			#if USECONSTRUCTIONMODE
// 				CN_FindPins();
// 				CN_DetectPins();	
// 			#endif
// 			
// 			NAVPP_NavCamPreProcessing();
// 			NAVP_NavCamProcessing();
// 			
// 			#if USEVISIONTUNNELNAV
// 			
// 			if ((Mode.Direction==TAKELEFT && !SquareRegion.OSearch.lowl.valid) || (Mode.Direction==TAKERIGHT && !SquareRegion.OSearch.lowr.valid))
// 			{	
// 				VTN_GetTunnelImage();
// 				cvDilate( TunnelImg, TunnelImg, NULL, 1);
// 				
// 				VTN_FindPixelsLFM(TunnelImg,220,170);
// 				NAVP_GetGoToPoint(Table_LineNumber,Table_RoadWidth);
// 				NAVP_GetDAbyPP();
// 				
// 			
// 			}
// 			
// 			
// 			#endif
// 			
// 			#if USEDISTANTLIGHTSREADING
// 				DRL_ApplyFilter(1/*1=Saturationfilter*/);	
// 				DRL_ApplyFilter(2/*2=Intensityfilter*/);	
// 				DRL_MergeSandVFilter();	
// 				DRL_ColorRecognition();
// 				DRL_ShapeRecognition();
// 				DRL_COLORandShapeMerge();			
// 			#endif
// 				
// 				
// 				
// 				
// 				
// 			/*«««««««««««««««««««««««««««« Graphical section «««««««««««««««««««««««««««««««*/
// 			GF_DrawAll();
// 			
// 		break;
// 		}
// 		case STOPMODE: //_______________________STOP MODE_______________________________
// 		{
// 			
// 			#if USELIGHTSREADING
// 				RL_ApplyFilter(1/*1=Saturationfilter*/);	
// 				RL_ApplyFilter(2/*2=Intensityfilter*/);	
// 				RL_MergeSandVFilter();	
// 				RL_ColorRecognition();
// 				RL_ShapeRecognition();
// 				RL_COLORandShapeMerge();	
// 			#endif
// 
// 		break;
// 		}
// 		#if USETUNNELANALISYS
// 			case TUNNELMODE:  //____________________TUNNEL MODE______________________
// 			{
// 				TN_TunnelNavegation();
// 			
// 			break;
// 			}
// 		#endif		
// 		#if USECONSTRUCTIONMODE
// 			case CONSTRUCTIONMODE:  //_______________CONSTRUCTION MODE________________
// 			{
// 			
// 				CN_FindPins();
// 				CN_DetectPins();
// 				
// 				CN_FindPixelsLFO(SandV_Filter,220,140);
// 				NAVP_GetGoToPoint(Table_LineNumber,Table_RoadWidth);
// 				NAVP_GetDAbyPP();
// 				
// 				CN_DrawPins();
// 				GF_DrawAll();
// 			
// 			break;
// 			}
// 		#endif
// 		#if USEPARKING
// 			case PARKINGMODE:  //_______________PARKING MODE___________________________
// 			{
// 				
// 			
// 			if(Time.Since.LastModeChange<ParkAnalisys.ManouveringTime1)
// 			{
// 				ParkAnalisys.ManouverNumber=0;
// 			}
// 			else if(Time.Since.LastModeChange < ParkAnalisys.ManouveringTime2+ParkAnalisys.ManouveringTime1)
// 			{
// 				ParkAnalisys.ManouverNumber=1;
// 			}
// 			else if(Time.Since.LastModeChange < ParkAnalisys.ManouveringTime3 + ParkAnalisys.ManouveringTime2 + ParkAnalisys.ManouveringTime1)
// 			{
// 				ParkAnalisys.ManouverNumber=2;
// 			}	
// 			else if(Time.Since.LastModeChange > ParkAnalisys.ManouveringTime3 + ParkAnalisys.ManouveringTime2 + ParkAnalisys.ManouveringTime1)
// 			{
// 				ParkAnalisys.ManouverNumber=3;
// 			}
// 				
// 			break;
// 			}
// 		#endif
// 	}


	MM_CheckForModeChange_1(); //MODE MANAGEMENT
	
	#if USESIGNALS
		
		AF_LightsControl();
		SignalsState.HeadLights=1;
	#endif
	
	#if USEPIC1COMS
		SendDirAndSpeed(RobotStatus.ChosenDA_PIC,RobotStatus.ChosenSpeed,Handle.ComPort_0);
	#endif
	
	#if USEPIC2COMS
		RS_SendAllOrdersToPIC2(Handle.ComPort_1);
		RS_ReadSensorsFromPIC2(Handle.ComPort_1);	
	#endif
	
	
	
	
	
/*	TM_UpdateSince();
	AF_ApplyTunnelSensorFilter();
	TN_GetSideSensorsDistances();

	
	
	if((RobotStatus.LapNumber>=TOTALLAPS) && (Mode.Mode==STOPMODE))
	{
	
		SendDirAndSpeed(45,0,Handle.ComPort_0);
		#if POUT
			printf("\n\nTOTALLAPSREACHED. EXITING\n\n");
		#endif
		
		#if IPAINT
			CvFont exitfont;
			*/
	//		cvInitFont( &exitfont, CV_FONT_HERSHEY_SIMPLEX, 2/*normal horizontal size*/,2/*normal vertical size*/,0/*non italic font*/,3/*thickness*/,4/*line type*/ );
/*			cvPutText(RoadImg_rgb_hough, "FINAL LAP", cvPoint(0,100),&exitfont, CV_RGB(255,0,0));
			cvPutText(RoadImg_rgb_hough, " EXITING  ", cvPoint(0,200),&exitfont, CV_RGB(255,0,0));
			GF_ShowImages();
			
		#endif
		cvWaitKey(5000);
		exit(1);
		
	}*/
	
	
	
	
	
	
	
	#if IPAINT
		GF_WriteFlags();
	#endif
	
	
	cvShowImage("TunnelImg",TunnelImg);
	GF_ShowImages();

	
	
		int FWP,LWP;
int Distances[2][500];
int line,col,i;
uchar pixelvalue;
int firstwp=0;
	
	int fwp_found=0;
	i=0;
	int continuesearch=1;
	for (line=RobotStatus.Horizon+1;line<mask_Img->height;line++)  
	{
		cout << "line " << line << endl;
		for (col=0;col<mask_Img1->width;col++)
		{	
			pixelvalue=(unsigned char) cvGetReal2D( mask_Img1, line, col );
			
			if (pixelvalue && !fwp_found)
			{
				firstwp=col;
				fwp_found=1;
			}
			else if(!pixelvalue && fwp_found)
			{
				Distances[0][i]=line;
				Distances[1][i]=col-firstwp;
				i++;
				break;
			}
			else if (col==mask_Img->width-1)
			{
				Distances[0][i]=line;
				Distances[1][i]=col-firstwp;
				i++;
				break;
			}
		}		
		fwp_found=0;
	}
	
	
	
	
	int n;
	
	
	
	int g=0;
	
	printf("Line Numbers\n");
	for (g=0;g<i;g++)
	{
		printf("%d ,",Distances[0][g]);
	}
	printf("\n");
	
	
	
	printf("Road Width\n");
	for (g=0;g<i;g++)
	{
		printf("%d ,",Distances[1][g]/*+10 comentei mo porque n percebi porque*/);
	}
	printf("\n");
	
	//___________________________________Save to File_________________________________________
	FILE *fp = fopen("RoadWidth.cfg", "w");
	fprintf(fp, "#RoadWidthIndex.cfg - Lines | Width\n");
	
	
	for (g=0;g<i;g++)
	{
		fprintf(fp, "line%d\t",Distances[0][g]);
		fprintf(fp, "%d\n",Distances[1][g]/*+10 comentei mo porque n percebi porque*/);
	}
	printf("\n");
	
	fclose(fp);
	
	
	cvWaitKey(0);
	exit(0);
	
			
	
		
	RL_ClearLightsAnalisysStruct();
	AF_ClearBoxStruct(&SquareRegion);
	AF_ClearRobotStatusStruct();
	Mode.DistantCross=NOCROSSFOUND;
	cvZero(RoadImg_rgb_hough);
	cvZero(cross_Img);
	cvZero(GI.Data.Info);
	
	
}
		
}

#endif
