/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file auxiliaryfunc.cpp
* @brief All the auxiliary functions or miscelaneous are placed here.
*/

#ifndef _AUXILIARYFUNC_
#define _AUXILIARYFUNC_

#include "header.h"


/** @brief Deallocates the memory reserved for image img.
 * 
 * @param Img: points to the image to be released.
 * @return nothing
 */
void AF_DestroyImage(IplImage *Img)
{
	cvReleaseImageHeader(&Img);
	cvReleaseImage(&Img);
}

/** @brief Deallocates the memory reserved for matrix Matrix.
 * 
 * @param Matrix pointer to the matrix to be released.
 * @return nothing
 */
void AF_DestroyMatrix(CvMat *Matrix)
{
	cvReleaseMatHeader(&Matrix);
	cvReleaseMat(&Matrix);
}


/** @brief Clears the validation variables used to discard or not the box analysis.
 * 
 * @param *ST pointer to the SquareTest struct.
 * @return nothing
 */
void AF_ClearBoxStruct(SquareTest *ST)
{
	ST->MSearch.lowl.valid=0;
	ST->MSearch.topl.valid=0;
	ST->MSearch.lowr.valid=0;
	ST->MSearch.topr.valid=0;
	
	ST->OSearch.lowl.valid=0;
	ST->OSearch.topl.valid=0;
	ST->OSearch.lowr.valid=0;
	ST->OSearch.topr.valid=0;
	
	ST->ValidLowGoToPoint=0;
	ST->ValidTopGoToPoint=0;
	ST->ValidLowLeftGoToPoint=0;
	ST->ValidLowRightGoToPoint=0;
	ST->ValidBothLeftGoToPoint=0;
	ST->ValidBothRightGoToPoint=0;
	
	ST->RelocatedLowGoToPoint=0;
	ST->RelocatedTopGoToPoint=0;
	ST->RelocatedLowLeftGoToPoint=0;
	ST->RelocatedLowRightGoToPoint=0;
	
	ST->MaxRoadWidthExcededLowGoToPoint=0;
	ST->MaxRoadWidthExcededTopGoToPoint=0;
}

/** @brief Clears some variables used in the RobotStatus struct
 * 
 * @param none
 * @return nothing
 */
void AF_ClearRobotStatusStruct()
{
	RobotStatus.DA.BothLow[1]=0;
	RobotStatus.DA.BothTop[1]=0;
	RobotStatus.DA.LowLeft[1]=0;
	RobotStatus.DA.LowRight[1]=0;
	RobotStatus.DA.BothLeft[1]=0;
	RobotStatus.DA.BothRight[1]=0;
	
	
	RobotStatus.DA_PP.BothLow[1]=0;
	RobotStatus.DA_PP.BothTop[1]=0;
	RobotStatus.DA_PP.LowLeft[1]=0;
	RobotStatus.DA_PP.LowRight[1]=0;
	RobotStatus.DA_PP.BothLeft[1]=0;
	RobotStatus.DA_PP.BothRight[1]=0;
	
	RobotStatus.PreferedHorizon=HORIZON;
	RobotStatus.PinsDetectedDown=0;
	RobotStatus.PinsDetectedAll=0;
	RobotStatus.SandV_sumDown.val[0]=0;
	RobotStatus.SandV_sumAll.val[0]=0;
	
}

/** @brief Converts the calculated drive angle (DA) to the pic protocol using the pointers to the TDS and TDA tables.
 * 
 * @param TDS pointer to the Table_DirAngle defined in header.h .
 * @param TDA pointer to the Table_DirToSend defined in header.h .
 */
void AF_ConvertDA(int *TDS, int *TDA)
{
int i;
double dadist=0; 

	for (i=0;i<TDTSSize;i++)
	{
		if ((int)RobotStatus.ChosenDA<=TDA[i]) 
			break;			
	}
		
	if (i==0)
		RobotStatus.ChosenDA_PIC=TDS[i];
	else if (i==TDTSSize)
		RobotStatus.ChosenDA_PIC=TDS[i-1];
	else
	{
		dadist=(double)((double)TDA[i]-RobotStatus.ChosenDA)/(double)(TDA[i]-TDA[i-1]);
		RobotStatus.ChosenDA_PIC=(int)(((double)TDS[i]-dadist*(double)(TDS[i]-TDS[i-1])));
	
	}

	#if NPOUT
		printf("___________ConvertDAtoPIC Display______________\n");
		printf("TDA[%d]=%d  || TDA[%d]=%d\n",i,TDA[i],i-1,TDA[i-1]);
		printf("i=%d\n",i);
		printf("dadist=%g\n",dadist);
		printf("RobotStatus.ChosenDA_PIC=%d\n",RobotStatus.ChosenDA_PIC);
		printf("_______________________________________________\n");
	#endif
}

/** @brief Decides whether to turn all lights on or off based in the current mode and also some preprocessing directives.
 * 
 */
void AF_LightsControl()
{
	
	if (Mode.Mode==NORMALMODE)
	{
		SignalsState.HeadLights=0;	
		
		if (RobotStatus.ChosenSpeed==CROSSAPROACHSPEED)
			SignalsState.TailLights=1;	
		else 
			SignalsState.TailLights=0;
					
		#if USELIGHTSREADING
		if (Mode.LastMode==STOPMODE && Time.Since.LastModeChange<2000)
		{
			
			if (Mode.Direction==TAKERIGHT)
			{
				SignalsState.TurnRight=1;
			}
			else if (Mode.Direction==TAKELEFT)
			{
			 	SignalsState.TurnLeft=1;
			}
		}
		else 
		#endif
		{
			SignalsState.TurnRight=0;
			SignalsState.TurnLeft=0;
		}
	}
	else if (Mode.Mode==STOPMODE)
	{
		SignalsState.TailLights=1;
		SignalsState.TurnRight=0;
		SignalsState.TurnLeft=0;
		SignalsState.TurnLeft=0;
	}
	#if USETUNNELANALISYS
	else if (Mode.Mode==TUNNELMODE)
	{
		SignalsState.TailLights=0;
		SignalsState.HeadLights=1;
		SignalsState.TurnRight=1;
		SignalsState.TurnLeft=1;
	}
	#endif
	#if USECONSTRUCTIONMODE
	else if (Mode.Mode==CONSTRUCTIONMODE)
	{
		SignalsState.TailLights=0;
		SignalsState.HeadLights=1;
		SignalsState.TurnRight=1;
		SignalsState.TurnLeft=1;
	}
	#endif
	#if USEPARKING
	else if (Mode.Mode==PARKINGMODE)
	{
		SignalsState.TailLights=0;
		SignalsState.HeadLights=0;
		SignalsState.TurnRight=1;
		SignalsState.TurnLeft=1;
	}
	#endif
}

/** @brief Decides the robot's speed based in the current mode and also some preprocessing directives.
 * 
 */
void AF_SpeedControl()
{
int i=99;
	
	if (Mode.Mode==NORMALMODE)
	{
		#if USELIGHTSREADING
		if (Time.Since.LastLapCount<TIMETOSTARTLOOKINGFORCROSS && Mode.LastMode==STOPMODE)
		{
			//printf("oi1\n");
		
			/*if (RobotStatus.BlindTurn && Mode.Direction==TAKELEFT)
			{	
				RobotStatus.ChosenSpeed=LEFTBLINDTURNSPEED;
				i=8;
			}
			else if (RobotStatus.BlindTurn && Mode.Direction==TAKERIGHT)
			{
				RobotStatus.ChosenSpeed=RIGHTBLINDTURNSPEED;
				i=9;
			}
			else*/
			{
				RobotStatus.ChosenSpeed=NORMALSPEED;
				i=7;
			}
		}
		else if (Mode.DistantCross==NOCROSSFOUND && Time.Since.LastCross>TIMETOWAITAFTERLASTCROSS)
		{	
			RobotStatus.ChosenSpeed=NORMALSPEED;
			i=0;
		}	
		else if (Mode.DistantCross==CROSSFOUND  && Time.Since.LastModeChange>TIMETOSTARTLOOKINGFORCROSS )
		{
			RobotStatus.ChosenSpeed = CROSSAPROACHSPEED;
			i=1;		
		}
		else if (Mode.DistantCross==NOCROSSFOUND && Time.Since.LastCross<=TIMETOWAITAFTERLASTCROSS)
		{	
			
			RobotStatus.ChosenSpeed=CROSSAPROACHSPEED;
			i=2;
		}
		else 
		{
			RobotStatus.ChosenSpeed=NORMALSPEED;
			i=10;
		}
		#endif
		
		#if !USELIGHTSREADING
		if((RobotStatus.LapNumber>=(TOTALLAPS-1)))
		{	
			if (Time.Since.LastModeChange<TIMETOSTARTLOOKINGFORCROSS && Mode.LastMode==STOPMODE)
			{
				RobotStatus.ChosenSpeed=NORMALSPEED;
				i=7;
				
			}
			else if (Mode.DistantCross==NOCROSSFOUND && Time.Since.LastCross>TIMETOWAITAFTERLASTCROSS)
			{	
				
				RobotStatus.ChosenSpeed=NORMALSPEED;
				i=0;
				
			}	
			else if (Mode.DistantCross==CROSSFOUND  && Time.Since.LastModeChange>1500)
			{
				RobotStatus.ChosenSpeed=CROSSAPROACHSPEED;
				i=1;
					
			}
			else if (Mode.DistantCross==NOCROSSFOUND && Time.Since.LastCross<=TIMETOWAITAFTERLASTCROSS)
			{	
				
				RobotStatus.ChosenSpeed=CROSSAPROACHSPEED;
				i=2;
			}
		}
		else 
		{
			RobotStatus.ChosenSpeed=NORMALSPEED;
		}
		#endif
		
		
	}
	else if (Mode.Mode==STOPMODE)
	{
		RobotStatus.ChosenSpeed=STOPPEDSPEED;
		i=3;
	}
	#if USETUNNELANALISYS
	else if (Mode.Mode==TUNNELMODE)
	{
		RobotStatus.ChosenSpeed=TUNNELSPEED;
		i=4;
	}
	#endif
	#if USECONSTRUCTIONMODE
	else if (Mode.Mode==CONSTRUCTIONMODE)
	{
		RobotStatus.ChosenSpeed=CONSTRUCTIONSPEED;
		i=5;
	}
	#endif
	#if USEPARKING
	else if (Mode.Mode==PARKINGMODE)
	{
		if (ParkAnalisys.ManouverNumber==0 || ParkAnalisys.ManouverNumber==1 || ParkAnalisys.ManouverNumber==2 )
			RobotStatus.ChosenSpeed=PARKINGSPEED;
		else if (ParkAnalisys.ManouverNumber==3)
			RobotStatus.ChosenSpeed=0;
		
		i=6;
	}
	#endif
	
	
	
	
	
	
	#if NPOUT
		printf("______________Speed Control Display____________\n");
		printf("Chosen Speed = %d ",RobotStatus.ChosenSpeed);
		printf("[ selected  ");
		if (i==0) printf("Normal Speed]\n");
		else if (i==1) printf("Cross Aproach Speed] ]\n");
		else if (i==2) printf("Cross Aproach Speed (timeout not over)]]\n");
		else if (i==3) printf("Stopped Speed ]]\n");
		else if (i==4) printf("Tunnel Speed ]]\n");
		else if (i==5) printf("Const. Speed ]]\n");
		else printf("(no selection info) ]\n");
		printf("_______________________________________________\n");
	#endif	
	
	#if IPAINT
	char str[255];
	
		if (i==0) 
		{	
			sprintf(str,"SPEED: Normal [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==1) 
		{
			sprintf(str,"SPEED: Cross Aproach [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==2) 
		{
			sprintf(str,"SPEED: Cross Aproach (timeout) [%d]",RobotStatus.ChosenSpeed);	
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==3) 
		{
			sprintf(str,"SPEED: Stopped [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==4) 
		{
			sprintf(str,"SPEED: Tunnel [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==5) 
		{
			sprintf(str,"SPEED: Const. [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==6) 
		{
			sprintf(str,"SPEED: Parking [%d]",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==7) 
		{
			sprintf(str,"SPEED: Normal [%d] (not looking for cross)",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==8) 
		{
			sprintf(str,"SPEED: LeftBlindTurn [%d] ",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==9) 
		{
			sprintf(str,"SPEED: RightBlindTurn [%d] ",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else if (i==10) 
		{
			sprintf(str,"SPEED: [%d] (no if is true) ",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
		else 
		{
			sprintf(str,"SPEED: [%d] (no selection info)",RobotStatus.ChosenSpeed);
			cvPutText(RoadImg_rgb_hough, str, cvPoint(0,30),&font, CV_RGB(0,30,180));
		}
	#endif	
	
	
}


void AF_DirectionControl_1(SquareTest *ST)
{
int i=0;


// if (Mode.Mode==NORMALMODE)
// {
// 	if ((Mode.Direction==TAKELEFT && !RobotStatus.DA_PP.LowLeft[1] && !RobotStatus.BlindTurn) || (Mode.Direction==TAKERIGHT && !RobotStatus.DA_PP.LowRight[1] && !RobotStatus.BlindTurn))
// 	{
// 		//printf("blindturn=%d\n",RobotStatus.BlindTurn);
// 		TM_UpdateTick(&Time.Tick.StartBlindTurn);
// 		RobotStatus.BlindTurn=1;
// 	}
// 	else if ((Mode.Direction==TAKELEFT && RobotStatus.DA_PP.LowLeft[1]) || (Mode.Direction==TAKERIGHT && RobotStatus.DA_PP.LowRight[1]))
// 	{
// 		RobotStatus.BlindTurn=0;
// 	}
// }


if (Mode.Mode==NORMALMODE || Mode.Mode==APROACHINGCROSSMODE)
{	
	switch (Mode.Direction)
	{	
		case TAKELEFT:
		{
			
			/*if(!RobotStatus.DA_PP.LowLeft[1])
			{
				
				RobotStatus.ChosenDA=60;//RobotStatus.DA.BothLeft[0];
			
				//to signal a vertical line (emergency situation)
				RobotStatus.ChosenGoToPoint[0]=0;
				RobotStatus.ChosenGoToPoint[1]=0;
				i=13;
			}
			else */
			if (RobotStatus.DA_PP.BothTop[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
				i=1;
				RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.BothLow[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
				i=3;
				RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.LowLeft[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
				i=5;
				RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.LowRight[1])
			{	
				RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
				i=7;
				RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
			}
			else 
			{
				RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
				i=14;
			}
		break;
		}
		case TAKERIGHT:
		{
			
			/*if(!RobotStatus.DA_PP.LowLeft[1])
			{
				
				RobotStatus.ChosenDA=60;//RobotStatus.DA.BothLeft[0];
			
				//to signal a vertical line (emergency situation)
				RobotStatus.ChosenGoToPoint[0]=0;
				RobotStatus.ChosenGoToPoint[1]=0;
				i=13;
			}
			else */
			if (RobotStatus.DA_PP.BothTop[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
				i=1;
				RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.BothLow[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
				i=3;
				RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.LowRight[1])
			{	
				RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
				i=7;
				RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
			}
			else if(RobotStatus.DA_PP.LowLeft[1])
			{
				RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
				i=5;
				RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
			}
			else 
			{
				RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
				i=14;
			}
		break;		
		}
		case NONE:
		{
			//This is Not Comtempleted
		}
	}	
}
if (Mode.Mode==CONSTRUCTIONMODE)
{	
		

	/*if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
	{
		if (RobotStatus.DA_PP.BothTop[0])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
			i=1;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
			i=2;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
	}
	else*/ if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
	{
		if (RobotStatus.DA_PP.BothLow[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
			i=3;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
			i=4;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
	}
	/*else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
	{
		if (RobotStatus.DA_PP.BothLeft[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
			i=9;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
			i=10;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
	}
	else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
	{
		if (RobotStatus.DA_PP.BothRight[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
			i=11;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
			i=12;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
	}
	*/else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
	{
		if (RobotStatus.DA_PP.LowLeft[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0]-10;
			i=5;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0]-10;
			i=6;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
	}
	else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
	{	
		if (RobotStatus.DA_PP.LowRight[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0]+20;
			i=7;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0]+20;
			i=8;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
	}
	else 
	{
		RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
		i=14;
	}
		
	
		
}
else if (Mode.Mode==STOPMODE)
{
	RobotStatus.ChosenDA=45;
}
#if USETUNNELANALISYS
else if (Mode.Mode==TUNNELMODE)
{
	RobotStatus.ChosenDA=45+TunnelAnalisys.Diference;
}
#endif
#if USEPARKING
else if (Mode.Mode==PARKINGMODE)
{
	if (ParkAnalisys.ManouverNumber==0)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir1;
	else if (ParkAnalisys.ManouverNumber==1)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir2;
	else if (ParkAnalisys.ManouverNumber==2)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir3;
	else if (ParkAnalisys.ManouverNumber==3)
		RobotStatus.ChosenDA=45;	
}
#endif


	
	RobotStatus.ChosenAnalisys=i; //to be used by a graphical function
	
	RobotStatus.LastDA[4]=RobotStatus.LastDA[3];
	RobotStatus.LastDA[3]=RobotStatus.LastDA[2];
	RobotStatus.LastDA[2]=RobotStatus.LastDA[1];
	RobotStatus.LastDA[1]=RobotStatus.LastDA[0];
	RobotStatus.LastDA[0]=RobotStatus.ChosenDA;
	
	RobotStatus.ChosenDA=(RobotStatus.LastDA[4]+RobotStatus.LastDA[3]+RobotStatus.LastDA[2]+RobotStatus.LastDA[1]+RobotStatus.LastDA[0])/5;
	
	
	#if POUT
		printf("__________Direction Control Display____________\n");
		printf("Chosen DA = %g ",RobotStatus.ChosenDA);
		printf("[ selected  ");
		if (i==1) printf("DA BothTop (PosPursuit) ]\n");
		else if (i==2) printf("DA BothTop (StraightAngle) ]\n");
		else if (i==3) printf("DA BothLow (PosPursuit) ]\n");
		else if (i==4) printf("DA BothLow (StraightAngle) ]\n");
		else if (i==5) printf("DA LowLeft (PosPursuit) ]\n");
		else if (i==6) printf("DA LowLeft (StraightAngle) ]\n");
		else if (i==7) printf("DA LowRight (PosPursuit) ]\n");
		else if (i==8) printf("DA LowRight (StraightAngle) ]\n");
		else if (i==9) printf("DA BothLeft (PosPursuit) ]\n");
		else if (i==10) printf("DA BothLeft (StraightAngle) ]\n");
		else if (i==11) printf("DA BothRight (PosPursuit) ]\n");
		else if (i==12) printf("DA BothRight (StraightAngle) ]\n");
		else if (i==13) printf("EMERGENCY (no left line) ]\n");
		else if (i==15) printf("EMERGENCY (no right line) ]\n");
		else if (i==14) printf("Last DA (no angles this time) ]\n");
		else printf("(no selection info) ]\n");
		cout << "i = " << i << endl;
		printf("_______________________________________________\n");
	#endif

	
	
	#if IPAINT
	char str[255];
	
		sprintf(str,"Direction: [%g]",RobotStatus.ChosenDA);
		cvPutText(RoadImg_rgb_hough, str, cvPoint(0,75),&font, CV_RGB(0,30,180));
		
		if (Mode.Direction==TAKELEFT)
			cvPutText(RoadImg_rgb_hough, "(Take Left)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		else if (Mode.Direction==TAKERIGHT)
			cvPutText(RoadImg_rgb_hough, "(Take Right)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		else if ((Mode.Direction==NONE))
			cvPutText(RoadImg_rgb_hough, "(Take None)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		
	#endif	

}

/**@brief Decides the robot's DA based in the current mode and also some preprocessing directives.
 * @param *ST SquareTest type struc.
 */
void AF_DirectionControl(SquareTest *ST)
{
int i=0;


if (Mode.Mode==NORMALMODE)
{
	if ((Mode.Direction==TAKELEFT && !RobotStatus.DA_PP.LowLeft[1] && !RobotStatus.BlindTurn) || (Mode.Direction==TAKERIGHT && !RobotStatus.DA_PP.LowRight[1] && !RobotStatus.BlindTurn))
	{
		//printf("blindturn=%d\n",RobotStatus.BlindTurn);
		TM_UpdateTick(&Time.Tick.StartBlindTurn);
		RobotStatus.BlindTurn=1;
	}
	else if ((Mode.Direction==TAKELEFT && RobotStatus.DA_PP.LowLeft[1]) || (Mode.Direction==TAKERIGHT && RobotStatus.DA_PP.LowRight[1]))
	{
		RobotStatus.BlindTurn=0;
	}
}







if (Mode.Mode==NORMALMODE)
{	
	switch (Mode.Direction)
	{	
		case NONE:
		{
			if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
			{
				if (RobotStatus.DA_PP.BothTop[0])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
					i=1;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
					i=2;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
			}
			else if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
			{
				if (RobotStatus.DA_PP.BothLow[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
					i=3;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
					i=4;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
			}
			else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
			{
				if (RobotStatus.DA_PP.BothLeft[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
					i=9;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
					i=10;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
			}
			else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
			{
				if (RobotStatus.DA_PP.BothRight[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
					i=11;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
					i=12;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
			}
			else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
			{
				if (RobotStatus.DA_PP.LowLeft[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
					i=5;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0];
					i=6;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
			}
			else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
			{	
				if (RobotStatus.DA_PP.LowRight[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
					i=7;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0];
					i=8;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
			}
			else 
			{
				RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
				i=14;
			}
				
		break;	
		}
		case TAKELEFT:
		{
			
			if(!RobotStatus.DA_PP.LowLeft[1])
			{
				
				RobotStatus.ChosenDA=60;//RobotStatus.DA.BothLeft[0];
			
				//to signal a vertical line (emergency situation)
				RobotStatus.ChosenGoToPoint[0]=0;
				RobotStatus.ChosenGoToPoint[1]=0;
				i=13;
			}
			else if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
			{
				if (RobotStatus.DA_PP.BothTop[0])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
					i=1;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
					i=2;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
			}
			else if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
			{
				if (RobotStatus.DA_PP.BothLow[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
					i=3;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
					i=4;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
			}
			/*else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
			{
				if (RobotStatus.DA_PP.BothLeft[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
					i=9;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
					i=10;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
			}
			else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
			{
				if (RobotStatus.DA_PP.BothRight[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
					i=11;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
					i=12;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
			}
			*/else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
			{
				if (RobotStatus.DA_PP.LowLeft[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
					i=5;
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0];
					i=6;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
			}
			else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
			{	
				if (RobotStatus.DA_PP.LowRight[1])
				{
					RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
					i=7;
				}
				else
				{
					RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0];
					i=8;
				}
				RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
				RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
			}
			else 
			{
				RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
				i=14;
			}
		break;
		}
		case TAKERIGHT:
		{
			
			if (RobotStatus.ChosenSpeed==CROSSAPROACHSPEED)
			{
				if(!RobotStatus.DA_PP.LowRight[1] )
				{
					
						RobotStatus.ChosenDA=33;//RobotStatus.DA.BothLeft[0];
					
					//to signal a vertical line (emergency situation)
					RobotStatus.ChosenGoToPoint[0]=0;
					RobotStatus.ChosenGoToPoint[1]=319;
					i=15;
				}
				else if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
				{
					if (RobotStatus.DA_PP.BothTop[0])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
						i=1;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
						i=2;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
				}
				else if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
				{
					if (RobotStatus.DA_PP.BothLow[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
						i=3;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
						i=4;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
				}
				/*else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
				{
					if (RobotStatus.DA_PP.BothLeft[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
						i=9;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
						i=10;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
				}
				else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
				{
					if (RobotStatus.DA_PP.BothRight[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
						i=11;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
						i=12;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
				}
				*/else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
				{
					if (RobotStatus.DA_PP.LowLeft[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
						i=5;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0];
						i=6;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
				}
				else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
				{	
					if (RobotStatus.DA_PP.LowRight[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
						i=7;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0];
						i=8;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
					i=14;
				}
			}
			else
			{
				if(!RobotStatus.DA_PP.LowRight[1] )
				{
					
						RobotStatus.ChosenDA=33;//RobotStatus.DA.BothLeft[0];
					
					//to signal a vertical line (emergency situation)
					RobotStatus.ChosenGoToPoint[0]=0;
					RobotStatus.ChosenGoToPoint[1]=319;
					i=15;
				}
				else if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
				{
					if (RobotStatus.DA_PP.BothTop[0])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
						i=1;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
						i=2;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
				}
				else if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
				{
					if (RobotStatus.DA_PP.BothLow[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
						i=3;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
						i=4;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
				}
				/*else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
				{
					if (RobotStatus.DA_PP.BothLeft[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
						i=9;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
						i=10;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
				}
				else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
				{
					if (RobotStatus.DA_PP.BothRight[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
						i=11;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
						i=12;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
				}
				*/else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
				{
					if (RobotStatus.DA_PP.LowLeft[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0];
						i=5;
					}
					else 
					{
						RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0];
						i=6;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
				}
				else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
				{	
					if (RobotStatus.DA_PP.LowRight[1])
					{
						RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0];
						i=7;
					}
					else
					{
						RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0];
						i=8;
					}
					RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
					RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
				}
				else 
				{
					RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
					i=14;
				}
			}
		}
		break;
			
	}	
}
if (Mode.Mode==CONSTRUCTIONMODE)
{	
		

	/*if (RobotStatus.DA.BothTop[1] || RobotStatus.DA_PP.BothTop[1])
	{
		if (RobotStatus.DA_PP.BothTop[0])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothTop[0];
			i=1;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothTop[0];
			i=2;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->TopGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->TopGoToPoint[1];
	}
	else*/ if(RobotStatus.DA.BothLow[1] || RobotStatus.DA_PP.BothLow[1])
	{
		if (RobotStatus.DA_PP.BothLow[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLow[0];
			i=3;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothLow[0];
			i=4;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowGoToPoint[1];
	}
	/*else if(RobotStatus.DA.BothLeft[1] || RobotStatus.DA_PP.BothLeft[1])
	{
		if (RobotStatus.DA_PP.BothLeft[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothLeft[0];
			i=9;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothLeft[0];
			i=10;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->BothLeftGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->BothLeftGoToPoint[1];
	}
	else if(RobotStatus.DA.BothRight[1] || RobotStatus.DA_PP.BothRight[1])
	{
		if (RobotStatus.DA_PP.BothRight[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.BothRight[0];
			i=11;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.BothRight[0];
			i=12;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->BothRightGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->BothRightGoToPoint[1];
	}
	*/else if(RobotStatus.DA.LowLeft[1] || RobotStatus.DA_PP.LowLeft[1])
	{
		if (RobotStatus.DA_PP.LowLeft[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.LowLeft[0]-10;
			i=5;
		}
		else 
		{
			RobotStatus.ChosenDA=RobotStatus.DA.LowLeft[0]-10;
			i=6;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowLeftGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowLeftGoToPoint[1];
	}
	else if(RobotStatus.DA.LowRight[1] || RobotStatus.DA_PP.LowRight[1])
	{	
		if (RobotStatus.DA_PP.LowRight[1])
		{
			RobotStatus.ChosenDA=RobotStatus.DA_PP.LowRight[0]+20;
			i=7;
		}
		else
		{
			RobotStatus.ChosenDA=RobotStatus.DA.LowRight[0]+20;
			i=8;
		}
		RobotStatus.ChosenGoToPoint[0]=ST->LowRightGoToPoint[0];
		RobotStatus.ChosenGoToPoint[1]=ST->LowRightGoToPoint[1];
	}
	else 
	{
		RobotStatus.ChosenDA=RobotStatus.LastDA[0];	
		i=14;
	}
		
	
		
}
else if (Mode.Mode==STOPMODE)
{
	RobotStatus.ChosenDA=45;
}
#if USETUNNELANALISYS
else if (Mode.Mode==TUNNELMODE)
{
	RobotStatus.ChosenDA=45+TunnelAnalisys.Diference;
}
#endif
#if USEPARKING
else if (Mode.Mode==PARKINGMODE)
{
	if (ParkAnalisys.ManouverNumber==0)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir1;
	else if (ParkAnalisys.ManouverNumber==1)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir2;
	else if (ParkAnalisys.ManouverNumber==2)
		RobotStatus.ChosenDA=ParkAnalisys.ManouveringDir3;
	else if (ParkAnalisys.ManouverNumber==3)
		RobotStatus.ChosenDA=45;	
}
#endif


	
	RobotStatus.ChosenAnalisys=i; //to be used by a graphical function
	
	RobotStatus.LastDA[4]=RobotStatus.LastDA[3];
	RobotStatus.LastDA[3]=RobotStatus.LastDA[2];
	RobotStatus.LastDA[2]=RobotStatus.LastDA[1];
	RobotStatus.LastDA[1]=RobotStatus.LastDA[0];
	RobotStatus.LastDA[0]=RobotStatus.ChosenDA;
	
	RobotStatus.ChosenDA=(RobotStatus.LastDA[4]+RobotStatus.LastDA[3]+RobotStatus.LastDA[2]+RobotStatus.LastDA[1]+RobotStatus.LastDA[0])/5;
	
	
	#if POUT
		printf("__________Direction Control Display____________\n");
		printf("Chosen DA = %g ",RobotStatus.ChosenDA);
		printf("[ selected  ");
		if (i==1) printf("DA BothTop (PosPursuit) ]\n");
		else if (i==2) printf("DA BothTop (StraightAngle) ]\n");
		else if (i==3) printf("DA BothLow (PosPursuit) ]\n");
		else if (i==4) printf("DA BothLow (StraightAngle) ]\n");
		else if (i==5) printf("DA LowLeft (PosPursuit) ]\n");
		else if (i==6) printf("DA LowLeft (StraightAngle) ]\n");
		else if (i==7) printf("DA LowRight (PosPursuit) ]\n");
		else if (i==8) printf("DA LowRight (StraightAngle) ]\n");
		else if (i==9) printf("DA BothLeft (PosPursuit) ]\n");
		else if (i==10) printf("DA BothLeft (StraightAngle) ]\n");
		else if (i==11) printf("DA BothRight (PosPursuit) ]\n");
		else if (i==12) printf("DA BothRight (StraightAngle) ]\n");
		else if (i==13) printf("EMERGENCY (no left line) ]\n");
		else if (i==15) printf("EMERGENCY (no right line) ]\n");
		else if (i==14) printf("Last DA (no angles this time) ]\n");
		else printf("(no selection info) ]\n");
		printf("_______________________________________________\n");
	#endif


	#if IPAINT
	char str[255];
	
		sprintf(str,"Direction: [%g]",RobotStatus.ChosenDA);
		cvPutText(RoadImg_rgb_hough, str, cvPoint(0,75),&font, CV_RGB(0,30,180));
		
		if (Mode.Direction==TAKELEFT)
			cvPutText(RoadImg_rgb_hough, "(Take Left)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		else if (Mode.Direction==TAKERIGHT)
			cvPutText(RoadImg_rgb_hough, "(Take Right)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		else if ((Mode.Direction==NONE))
			cvPutText(RoadImg_rgb_hough, "(Take None)", cvPoint(150,75),&font, CV_RGB(0,30,180));
		
	#endif	





}

int isodata(unsigned int *histarray)
{
int T0=128,Tk;
int Mbk,Mfk;
int i,b,a;

while(1)
{
	a=0;b=0;


	for (i=0;i<T0;i++)
	{
		a+=i*histarray[i];
		b+=histarray[i];
		//printf("FOR 1  a=%d | b=%d\n",a,b);		
		
	}
	//printf("FOR 1 over\n");
	Mbk=a/b;
	
	
	
	a=0;b=0;
	
	for (i=T0;i<255;i++)
	{
		a+=i*histarray[i];
		b+=histarray[i];
		//printf("FOR 2  a=%d | b=%d\n",a,b);
	}
	//printf("FOR 2 over\n");
	Mfk=a/b;
	
	Tk=(Mbk+Mfk)/2;
	
	printf("T0=%d | Tk=%d\n",T0,Tk);
	
	
	if (T0==Tk)
		break;
	else
		T0=Tk;
}

printf("binlimit=%d\n",Tk);
return(Tk);

}

void AF_GetHistogram(unsigned int *histarray, IplImage *src)
{
unsigned char pixelvalue ;
int line,col;

for (line=0;line<src->height;line++)
	for (col=0;col<src->width;col++)
	{	
		//printf("line=%d | col=%d   |  ",line, col);
		pixelvalue=(unsigned char) cvGetReal2D( src, line, col );
		//printf("pixelvalue=%d\n", pixelvalue);
		histarray[(int)pixelvalue]=histarray[pixelvalue]+1;
	}
}


/** @brief Uses lights cam handle to grab an image, converts BGR to RGB ( LightsImg_Original ), and calculates also HSV_img , H_img , S_img , V_img.
 * 
 */
void AF_GetLightsImage()
{
	*LightsImg_Original = Handle.LightsCam->myGetImageColor();
	

	cvConvertImage(LightsImg_Original,LightsImg_Original,CV_CVTIMG_SWAP_RB);
	cvCvtColor(LightsImg_Original,GI.Lights.HSV,CV_RGB2HSV);	
	cvCvtPixToPlane(GI.Lights.HSV,GI.Lights.H,GI.Lights.S,GI.Lights.V,0);	

	
}

/** @brief Uses navegation cam handle to grab an image (RoadImg_original), takes G component, thresholds and removes isolated pixels (RoadImg_gray). If distant lights reading is on it also gets HSV images and it's components similarly to the AF_GetLightsImage().
 * 
 */
void AF_GetNavImage()
{

	TM_UpdateTick(&Time.Tick.Tick1);
	
	* GI.Orig.LeftCam = Handle.LeftCam->myGetImageColor();
	cvConvertImage(GI.Orig.LeftCam ,GI.Orig.LeftCam ,CV_CVTIMG_SWAP_RB);
	cvCvtPixToPlane(GI.Orig.LeftCam,NULL, GI.Orig.LeftCam_1ch,NULL,0);	
	UndistortImage(GI.Orig.LeftCam, GI.Orig.LeftCam_Undistorted ,  CamCalibparams.distDataL, & CamCalibparams.mapxL, & CamCalibparams.mapyL ) ;  //apply maps
	
	*GI.Orig.RightCam = Handle.RightCam->myGetImageColor();
	cvConvertImage(GI.Orig.RightCam,GI.Orig.RightCam,CV_CVTIMG_SWAP_RB);
	cvCvtPixToPlane(GI.Orig.RightCam,NULL,GI.Orig.RightCam_1ch,NULL,0);		
	UndistortImage(GI.Orig.RightCam, GI.Orig.RightCam_Undistorted , CamCalibparams.distDataR, & CamCalibparams.mapxR, & CamCalibparams.mapyR ) ;  //apply maps
		
	if(! joinImages( GI.Orig.LeftCam_Undistorted , GI.Orig.RightCam_Undistorted , GI.Orig.RoadImg, CamCalibparams.overlapH, CamCalibparams.bin, CamCalibparams.binOper ))
    	{
	    printf("Could not join images. Aborting\n");
	    exit(0); 
    	}

	cvCvtPixToPlane(GI.Orig.RoadImg,NULL,RoadImg_gray,NULL,0);		
	//cvThreshold( RoadImg_gray, RoadImg_gray, CamCalibparams.bin & 0xFF, 255, CV_THRESH_BINARY );
	
	
	TM_UpdateTick(&Time.Tick.Tick2);
	cout << "aquire<&undistort: " << (Time.Tick.Tick2-Time.Tick.Tick1)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;

	TM_UpdateTick(&Time.Tick.Tick1);
	
	AF_RemoveIsolatedPix(RoadImg_gray,RoadImg_gray,RobotStatus.PreferedHorizon);
	
	TM_UpdateTick(&Time.Tick.Tick2);
	cout << "Remove: " << (Time.Tick.Tick2-Time.Tick.Tick1)/(Time.Tickspermicrosec*1000/*to get milisecs*/) << "msecs." << endl;
	
	//cvWaitKey(0);
	#if USEDISTANTLIGHTSREADING
		
		cvCvtColor(RoadImg_original,HSV_Roadimg,CV_RGB2HSV);	
		cvRectangle(HSV_Roadimg,cvPoint(0,RobotStatus.Horizon-1),cvPoint(HSV_Roadimg->width,HSV_Roadimg->height-1),CV_RGB(0,0,0),-1); 
		cvRectangle(HSV_Roadimg,cvPoint(0,RobotStatus.Horizon-1),cvPoint(HSV_Roadimg->width/3,0),CV_RGB(0,0,0),-1); //
		cvCvtPixToPlane(HSV_Roadimg,H_Roadimg,S_Roadimg,V_Roadimg,0);		
			
	#endif
}
	

/** Outputs to the windows or the shell information concerning robot mode. Outputs are controled in the standard way:
	- for shell output use NPOUT = 1.
 	- for windows output use IPAINT = 1.
 * @param i tells the function which mode is currently selected.
 */
void AF_ModeOutput(unsigned char i)
{
	#if NPOUT
		printf("_______________________________________________\n");
	if (Mode.Mode==0)
		printf("___________________NORMAL MODE_________________\n");
	else if (Mode.Mode==1)
		printf("___________________STOP MODE___________________\n");
	if (Mode.Mode==2)
		printf("___________________CROSSAPROACHING MODE________\n");
	if (Mode.Mode==3)
		printf("___________________PARKING MODE________________\n");
	if (Mode.Mode==4)
		printf("___________________TUNNEL MODE_________________\n");
	if (Mode.Mode==5)
		printf("___________________CONSTRUCTION MODE____________\n");
	printf("_______________________________________________\n");
	#endif	
		
	#if IPAINT
		
		if (Mode.Mode==-1)
			cvPutText(RoadImg_rgb_hough, "MODE: DUMMY", cvPoint(0,60),&font, CV_RGB(120,128,100));	
		else if (Mode.Mode==0)
			cvPutText(RoadImg_rgb_hough, "MODE: NORMAL", cvPoint(0,15),&font, CV_RGB(120,128,100));
		else if (Mode.Mode==1)
			cvPutText(RoadImg_rgb_hough, "MODE: STOP", cvPoint(0,15),&font, CV_RGB(0,128,100));
		else if (Mode.Mode==2)
			cvPutText(RoadImg_rgb_hough, "MODE: APROACHING CROSS", cvPoint(0,15),&font, CV_RGB(0,128,100));
		else if (Mode.Mode==3)
			cvPutText(RoadImg_rgb_hough, "MODE: PARKING", cvPoint(0,15),&font, CV_RGB(0,128,100));
		else if (Mode.Mode==4)
			cvPutText(RoadImg_rgb_hough, "MODE: TUNNEL", cvPoint(0,15),&font, CV_RGB(0,128,100));
		else if (Mode.Mode==5)
			cvPutText(RoadImg_rgb_hough, "MODE: CONSTRUCTION", cvPoint(0,15),&font, CV_RGB(190,8,10));
			
		if (Mode.LastMode==-1)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: DUMMY", cvPoint(0,60),&font, CV_RGB(120,128,100));	
		else if (Mode.LastMode==0)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: NORMAL", cvPoint(0,60),&font, CV_RGB(120,128,100));
		else if (Mode.LastMode==1)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: STOP", cvPoint(0,60),&font, CV_RGB(0,128,100));
		else if (Mode.LastMode==2)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: APROACHING CROSS", cvPoint(0,60),&font, CV_RGB(0,128,100));
		else if (Mode.LastMode==3)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: PARKING", cvPoint(0,60),&font, CV_RGB(0,128,100));
		else if (Mode.LastMode==4)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: TUNNEL", cvPoint(0,60),&font, CV_RGB(0,128,100));
		else if (Mode.LastMode==5)
			cvPutText(RoadImg_rgb_hough, "LASTMODE: CONSTRUCTION", cvPoint(0,60),&font, CV_RGB(120,128,100));
			
		if (i==0) 
		{	
			cvPutText(RoadImg_rgb_hough, "No Reason", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==1) 
		{
			cvPutText(RoadImg_rgb_hough, "SM timeout", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==2) 
		{
			cvPutText(RoadImg_rgb_hough, "Lights GAU", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==3) 
		{
			cvPutText(RoadImg_rgb_hough, "Lights YAL", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==4) 
		{
			cvPutText(RoadImg_rgb_hough, "Close Cross", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==5) 
		{
			cvPutText(RoadImg_rgb_hough, "Tunnel Sensor", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==6) 
		{
			cvPutText(RoadImg_rgb_hough, "Left Sensors", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==7) 
		{
			cvPutText(RoadImg_rgb_hough, "Right Sensors", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}
		else if (i==8) 
		{
			cvPutText(RoadImg_rgb_hough, "Tunnel Sensor", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}		
		else if (i==9) 
		{
			cvPutText(RoadImg_rgb_hough, "Lights YAR", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}	
		else if (i==10) 
		{
			cvPutText(RoadImg_rgb_hough, "Pins Detected", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}	
		else if (i==11) 
		{
			cvPutText(RoadImg_rgb_hough, "No Pins Detected", cvPoint(190,15),&font, CV_RGB(255,255,0));
		}	
			
	#endif
}
/** Averages the last ten values measured in each sensor.
*
*/
void AF_ApplyTunnelSensorFilter(void)
{
int i;
	SensorValues.RightFrontFilt=0;
	for (i=0;i<10;i++)
		SensorValues.RightFrontFilt+=SensorValues.RightFront[i];
	SensorValues.RightFrontFilt=SensorValues.RightFrontFilt/10;

	SensorValues.RightBackFilt=0;
	for (i=0;i<10;i++)
		SensorValues.RightBackFilt+=SensorValues.RightBack[i];
	SensorValues.RightBackFilt=SensorValues.RightBackFilt/10;
	
	SensorValues.FrontRightFilt=0;
	for (i=0;i<10;i++)
		SensorValues.FrontRightFilt+=SensorValues.FrontRight[i];
	SensorValues.FrontRightFilt=SensorValues.FrontRightFilt/10;
	
	//SensorValues.FrontRightFilt=SensorValues.FrontRight[0];
	
	SensorValues.FrontLeftFilt=0;
	for (i=0;i<10;i++)
		SensorValues.FrontLeftFilt+=SensorValues.FrontLeft[i];
	SensorValues.FrontLeftFilt=SensorValues.FrontLeftFilt/10;

	SensorValues.LeftFrontFilt=0;
	for (i=0;i<10;i++)
		SensorValues.LeftFrontFilt+=SensorValues.LeftFront[i];
	SensorValues.LeftFrontFilt=SensorValues.LeftFrontFilt/10;
	
	SensorValues.LeftBackFilt=0;
	for (i=0;i<10;i++)
		SensorValues.LeftBackFilt+=SensorValues.LeftBack[i];
	SensorValues.LeftBackFilt=SensorValues.LeftBackFilt/10;
	
	SensorValues.Tunnel_Filtered=(SensorValues.Tunnel[9]+SensorValues.Tunnel[8]+SensorValues.Tunnel[7]+SensorValues.Tunnel[6]+SensorValues.Tunnel[5]+SensorValues.Tunnel[4]+SensorValues.Tunnel[3]+SensorValues.Tunnel[2]+SensorValues.Tunnel[1]+SensorValues.Tunnel[0])/10;	

	
	#if NPOUT

	printf("_______________Sensor Values Display______________\n");	
	if (n==0)
		printf("Read From PIC1: Nothing to read\n");
	else
		printf("Read From PIC1: %d bytes\n",n);
	printf("SensorValues.Cross=%d\n",SensorValues.Cross);	
	printf("SensorValues.RightFront=%d\n",SensorValues.RightFront);
	printf("SensorValues.RightBack=%d\n",SensorValues.RightBack);
	printf("SensorValues.FrontRight=%d\n",SensorValues.FrontRight);
	printf("SensorValues.FrontLeft=%d\n",SensorValues.FrontLeft);
	printf("SensorValues.LeftFront=%d\n",SensorValues.LeftFront);
	printf("SensorValues.LeftBack=%d\n",SensorValues.LeftBack);
	printf("SensorValues.Tunnel_Filtered=%d\n",SensorValues.Tunnel_Filtered);
	printf("_____________Sensor Values Display END____________\n");	
				
#endif

#if IPAINT
char str[255];

	cvPutText(GI.Data.Info, "__SENSORS___(cm)__", cvPoint(0,15),&font, CV_RGB(0,0,255));
	sprintf(str,"Cross      %d",SensorValues.Cross);
	cvPutText(GI.Data.Info, str, cvPoint(0,30),&font, CV_RGB(0,0,255));
	sprintf(str,"RightFront %d",SensorValues.RightFrontFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,45),&font, CV_RGB(0,0,255));
	sprintf(str,"RightBack  %d",SensorValues.RightBackFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,60),&font, CV_RGB(0,0,255));
	sprintf(str,"FrontRight %d",SensorValues.FrontRightFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,75),&font, CV_RGB(0,0,255));
	sprintf(str,"FrontLeft  %d",SensorValues.FrontLeftFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,90),&font, CV_RGB(0,0,255));
	sprintf(str,"LeftFront  %d",SensorValues.LeftFrontFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,105),&font, CV_RGB(0,0,255));
	sprintf(str,"LeftBack   %d",SensorValues.LeftBackFilt);
	cvPutText(GI.Data.Info, str, cvPoint(0,120),&font, CV_RGB(0,0,255));
	sprintf(str,"Tunnel Filt. %d",SensorValues.Tunnel_Filtered);
	cvPutText(GI.Data.Info, str, cvPoint(0,135),&font, CV_RGB(0,0,255));
#endif	
}


/** Removes every isolated pixel from an image.
* 
* @param src pointer to the source image.
* @param dst pointer to the destination image.
* @param startline line from which to start executing the operation (it's a limited ROI).
*/
void AF_RemoveIsolatedPix(IplImage *src,IplImage *dst,int startline)
{
int line,col;
int pix1,pix2,pix3,pix4,pix5,pix6,pix7,pix8,pix9;
int totalvalue=0;



cvCopy(src,dst);
cvCopy(src,GI.Cross.TempImg);
cvGetMat(GI.Cross.TempImg,GI.Cross.test3);
cvConvertScale( GI.Cross.test3, GI.Cross.TempMat, 1.0/255.0, 0 );

	pix1=0;pix2=0;pix3=0;pix4=0;pix5=0;pix6=0;pix7=0;pix8=0;pix9=0;
	for (line=startline;line<src->height-1;line++)
	{
		totalvalue=0;
		for (col=1;col<src->width-1;col++)
		{	
			pix5=(int) cvmGet( GI.Cross.TempMat, line, col);
			if (pix5)	
			{
				pix1=(int) cvmGet( GI.Cross.TempMat, line-1, col-1);
				pix2=(int) cvmGet( GI.Cross.TempMat, line-1, col);
				pix3=(int) cvmGet( GI.Cross.TempMat, line-1, col+1);
				pix4=(int) cvmGet( GI.Cross.TempMat, line, col-1);
				pix6=(int) cvmGet( GI.Cross.TempMat, line, col+1);
				pix7=(int) cvmGet( GI.Cross.TempMat, line+1, col-1);
				pix8=(int) cvmGet( GI.Cross.TempMat, line+1, col);
				pix9=(int) cvmGet( GI.Cross.TempMat, line+1, col+1);
				totalvalue=pix1+pix2+pix3+pix4+pix6+pix7+pix8+pix9;
				if (!totalvalue)
					cvLine(dst,cvPoint(col,line),cvPoint(col,line),cvScalar(0),1,8); //draw Horizon line in RoadImg_threshold		
			}
		}
	}
}

#endif
