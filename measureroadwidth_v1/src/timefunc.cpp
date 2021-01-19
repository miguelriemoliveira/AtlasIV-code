/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: main.cpp********************************************/
/***************************************************************/
/*! @file timefunc.cpp
* @brief All the functions regarding timers. All functions within this group start with "TM_"
*/
#ifndef _TIMEFUNC_
#define _TIMEFUNC_


#include "header.h"


/** @brief Updates ticks for various events.
 * 
 * @param Tick2Update: uapsodapodk
 */
void TM_UpdateTick(int64 *Tick2Update)
{
	*Tick2Update=cvGetTickCount( );
}


/** @brief Calculates how many secs have passed from the present moment to the Tick .
 * 
 * @param Tick Tick used to find out how many msecs have passed.
 * @return How many msecs have passed since the Tick .
 */
int TM_TimeSince(int64 *Tick)
{
	TM_UpdateTick(&Time.Tick.Now);
	
	return((int)((Time.Tick.Now-(*Tick))/(Time.Tickspermicrosec*1000/*to get milisecs*/)));
}


/** @brief Updates the time that has passed since various events.
 * 
 */
void TM_UpdateSince()
{char str[255];

	if (Mode.PreviousMode!=Mode.Mode)
	{
		TM_UpdateTick(&Time.Tick.LastModeChange);
		Mode.PreviousMode=Mode.Mode;
	}
	
	
	Time.Since.ProgramStart= TM_TimeSince(&Time.Tick.ProgramStart);
	Time.Since.LastIteration= TM_TimeSince(&Time.Tick.LastIteration);
	Time.Since.LastCross= TM_TimeSince(&Time.Tick.LastCross);
	Time.Since.LastModeChange= TM_TimeSince(&Time.Tick.LastModeChange);
	Time.Since.LastLapCount= TM_TimeSince(&Time.Tick.LastLapCount);
	Time.Since.LastBlindTurn= TM_TimeSince(&Time.Tick.StartBlindTurn);
	
	#if NPOUT
		printf("____________Time Display (milisecs)____________\n");
		printf("Time.Since.ProgramStart=%d\n",Time.Since.ProgramStart);
		printf("Time.Since.LastIteration(Frames per sec)=%d (%d)\n",Time.Since.LastIteration, 1000/Time.Since.LastIteration);
		printf("Time.Since.LastModeChange=%d\n",Time.Since.LastModeChange);
		printf("_______________________________________________\n");
	#endif

	#if IPAINT
		
		cvLine(GI.Data.Info,cvPoint(158,15),cvPoint(158,200),CV_RGB(250,120,10),1,8);
		
		cvPutText(GI.Data.Info, "_______TIMERS______", cvPoint(160,15),&font, CV_RGB(250,120,10));
		sprintf(str,"Prog. Start %d",Time.Since.ProgramStart);
		cvPutText(GI.Data.Info, str, cvPoint(160,30),&font, CV_RGB(250,120,10));
		if (Time.Since.LastIteration>0) 
			sprintf(str,"Cycle (fps)= %d (%d)",Time.Since.LastIteration,1000/Time.Since.LastIteration);
		else
			sprintf(str,"Cycle (fps)= %d (--)",Time.Since.LastIteration);
			
		cvPutText(GI.Data.Info, str, cvPoint(160,45),&font, CV_RGB(250,120,10));
		
		sprintf(str,"Distant Cross= %d",Time.Since.LastCross);
		cvPutText(GI.Data.Info, str, cvPoint(160,60),&font, CV_RGB(250,120,10));
		
		sprintf(str,"ModeChange= %d",Time.Since.LastModeChange);
		cvPutText(GI.Data.Info, str, cvPoint(160,75),&font, CV_RGB(250,120,10));
		
		sprintf(str,"LapCount= %d",Time.Since.LastLapCount);
		cvPutText(GI.Data.Info, str, cvPoint(160,90),&font, CV_RGB(250,120,10));
		
		sprintf(str,"BlindTurn= %d",Time.Since.LastBlindTurn);
		cvPutText(GI.Data.Info, str, cvPoint(160,105),&font, CV_RGB(250,120,10));
		
		sprintf(str,"ILNormal= %d",Time.Since.InNormalMode);
		cvPutText(GI.Data.Info, str, cvPoint(160,120),&font, CV_RGB(250,120,10));
		
		sprintf(str,"ILStop= %d",Time.Since.InStopMode);
		cvPutText(GI.Data.Info, str, cvPoint(160,135),&font, CV_RGB(250,120,10));
		
		sprintf(str,"ILTunnel= %d",Time.Since.InTunnelMode);
		cvPutText(GI.Data.Info, str, cvPoint(160,150),&font, CV_RGB(250,120,10));
		
		sprintf(str,"ILConst.= %d",Time.Since.InConstructionMode);
		cvPutText(GI.Data.Info, str, cvPoint(160,165),&font, CV_RGB(250,120,10));
		
	#endif
	


}

#endif

