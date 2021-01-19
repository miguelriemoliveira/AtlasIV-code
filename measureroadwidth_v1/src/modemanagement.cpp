/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: modemanagement.c ***********************************/
/***************************************************************/
/** @file modemanagement.cpp
* @brief Check all RobotStatus variables and current mode to decide what should be the next mode.
*
*Modes can be the following:
*
	- DUMMYMODE=-1,
	- NORMALMODE,
	- STOPMODE,
	- APROACHINGCROSSMODE,
	- PARKINGMODE,
	- TUNNELMODE,
	- CONSTRUCTIONMODE
*/

#ifndef _MODEMANAGEMENT_
#define _MODEMANAGEMENT_

#include "header.h"

void MM_CheckForModeChange_1()
{
static unsigned char i=0;


	if (Mode.Mode==STOPMODE) 
	{
		if (CR_IsStopModeTimeout())	//wait 7 secs stopped then go to normal mode
		{
			Mode.Mode=NORMALMODE;
			Mode.LastMode=STOPMODE;
			i=1;
		}
		#if USELIGHTSREADING
			else if (CR_IsGreenArrowUp())
			{
				Mode.Direction=TAKERIGHT;			
				Mode.Mode=NORMALMODE;
				Mode.LastMode=STOPMODE;
				i=2;
			}
			else if (CR_IsYellowArrowLeft())
			{
				Mode.Direction=TAKELEFT;			
				Mode.Mode=NORMALMODE;
				Mode.LastMode=STOPMODE;
				i=3;
			}
		#endif
		#if USEPARKING
			else if (CR_IsYellowArrowRight())
			{
				Mode.Mode=PARKINGMODE;
				Mode.LastMode=STOPMODE;
				i=9;
			}
		#endif
	
	}
	else if(Mode.Mode==NORMALMODE)
	{
		
		#if USELIGHTSREADING
			if (CR_IsCloseCross_1() && Time.Since.LastLapCount>2000 )
			{
				Mode.Mode=STOPMODE;
				Mode.LastMode=NORMALMODE;
				RobotStatus.LapNumber++;
				TM_UpdateTick(&Time.Tick.LastLapCount);
				i=4;
			}
		#endif
		#if !USELIGHTSREADING 
			if (CR_IsCloseCross_1() && Time.Since.LastLapCount>2000)
			{
				if ((RobotStatus.LapNumber>=(TOTALLAPS-1)))
				{
					
					Mode.Mode=STOPMODE;
					Mode.LastMode=NORMALMODE;
					RobotStatus.LapNumber++;
					TM_UpdateTick(&Time.Tick.LastLapCount);
					i=4;
				}
				else 
				{
					Mode.LastMode=NORMALMODE;
					RobotStatus.LapNumber++;	
					TM_UpdateTick(&Time.Tick.LastLapCount);
					i=4;
				}
			}
		#endif
		#if USETUNNELANALISYS
			else if (CR_IsTunnelUp())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=5;
				//printf("SensorValues.Tunnel=%d\n",SensorValues.Tunnel_Filtered);
				//cvWaitKey(0);
			}
			/*else if (CR_IsTunnelLeft())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=6;
			}
			else if (CR_IsTunnelRight())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=7;
			}*/
		#endif
		#if USECONSTRUCTIONMODE
			else if(RobotStatus.PinsDetectedDown)
			{
				Mode.Mode=CONSTRUCTIONMODE;
				Mode.LastMode=NORMALMODE;
				i=10;
			}
		#endif
	}
	#if USETUNNELANALISYS
		else if(Mode.Mode==TUNNELMODE)
		{
			
			if (!CR_IsTunnelUp()/*CR_IsNotTunnel()*/)
			{
				Mode.Mode=NORMALMODE;
				Mode.LastMode=TUNNELMODE;
				i=8;
			}
		}
	#endif	
	#if USECONSTRUCTIONMODE
		else if(Mode.Mode==CONSTRUCTIONMODE)
		{
			
			if (!RobotStatus.PinsDetectedAll)
			{
				Mode.Mode=NORMALMODE;
				Mode.LastMode=CONSTRUCTIONMODE;
				i=11;
			}
		}
	#endif	

	
	
	if (Mode.Mode!=Mode.PreviousMode)
	{
		
		if (Mode.PreviousMode==NORMALMODE)
		{	
			TM_UpdateTick(&Time.Tick.NormalModeEnd);
			Time.Since.InNormalMode=(int)( ((Time.Tick.NormalModeEnd-(Time.Tick.NormalModeStart))/(Time.Tickspermicrosec*1000)));	
		}
		else if (Mode.PreviousMode==STOPMODE)
		{	
			TM_UpdateTick(&Time.Tick.StopModeEnd);
			Time.Since.InStopMode=(int)(((Time.Tick.StopModeEnd-(Time.Tick.StopModeStart))/(Time.Tickspermicrosec*1000)));	
		}
		else if (Mode.PreviousMode==TUNNELMODE)
		{	
			TM_UpdateTick(&Time.Tick.TunnelModeEnd);
			Time.Since.InTunnelMode=(int)(((Time.Tick.TunnelModeEnd-(Time.Tick.TunnelModeStart))/(Time.Tickspermicrosec*1000)));	
		}	
		else if (Mode.PreviousMode==CONSTRUCTIONMODE)
		{
			TM_UpdateTick(&Time.Tick.ConstructionModeEnd);
			Time.Since.InConstructionMode=(int)(((Time.Tick.ConstructionModeEnd-(Time.Tick.ConstructionModeStart))/(Time.Tickspermicrosec*1000)));
		}
		
		
		if (Mode.Mode==NORMALMODE)
			TM_UpdateTick(&Time.Tick.NormalModeStart);
		else if (Mode.Mode==STOPMODE)
			TM_UpdateTick(&Time.Tick.StopModeStart);
		else if (Mode.Mode==TUNNELMODE)
			TM_UpdateTick(&Time.Tick.TunnelModeStart);
		else if (Mode.Mode==CONSTRUCTIONMODE)
			TM_UpdateTick(&Time.Tick.ConstructionModeStart);
		
	}
	
	
	
	
	AF_ModeOutput(i);
		
}


/** @brief Chain of if's and elseif's that decides the robot's current mode.
 * 
 */
void MM_CheckForModeChange()
{
static unsigned char i=0;


	if (Mode.Mode==STOPMODE) 
	{
		if (CR_IsStopModeTimeout())	//wait 7 secs stopped then go to normal mode
		{
			Mode.Mode=NORMALMODE;
			Mode.LastMode=STOPMODE;
			i=1;
		}
		#if USELIGHTSREADING
			else if (CR_IsGreenArrowUp())
			{
				Mode.Direction=TAKERIGHT;			
				Mode.Mode=NORMALMODE;
				Mode.LastMode=STOPMODE;
				i=2;
			}
			else if (CR_IsYellowArrowLeft())
			{
				Mode.Direction=TAKELEFT;			
				Mode.Mode=NORMALMODE;
				Mode.LastMode=STOPMODE;
				i=3;
			}
		#endif
		#if USEPARKING
			else if (CR_IsYellowArrowRight())
			{
				Mode.Mode=PARKINGMODE;
				Mode.LastMode=STOPMODE;
				i=9;
			}
		#endif
	
	}
	else if(Mode.Mode==NORMALMODE)
	{
		
		#if USELIGHTSREADING
			if (CR_IsCloseCross() && Time.Since.LastLapCount>2000 )
			{
				Mode.Mode=STOPMODE;
				Mode.LastMode=NORMALMODE;
				RobotStatus.LapNumber++;
				TM_UpdateTick(&Time.Tick.LastLapCount);
				i=4;
			}
		#endif
		#if !USELIGHTSREADING 
			if (CR_IsCloseCross() && Time.Since.LastLapCount>2000)
			{
				if ((RobotStatus.LapNumber>=(TOTALLAPS-1)))
				{
					
					Mode.Mode=STOPMODE;
					Mode.LastMode=NORMALMODE;
					RobotStatus.LapNumber++;
					TM_UpdateTick(&Time.Tick.LastLapCount);
					i=4;
				}
				else 
				{
					Mode.LastMode=NORMALMODE;
					RobotStatus.LapNumber++;	
					TM_UpdateTick(&Time.Tick.LastLapCount);
					i=4;
				}
			}
		#endif
		#if USETUNNELANALISYS
			else if (CR_IsTunnelUp())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=5;
				//printf("SensorValues.Tunnel=%d\n",SensorValues.Tunnel_Filtered);
				//cvWaitKey(0);
			}
			/*else if (CR_IsTunnelLeft())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=6;
			}
			else if (CR_IsTunnelRight())
			{
				Mode.Mode=TUNNELMODE;
				Mode.LastMode=NORMALMODE;
				i=7;
			}*/
		#endif
		#if USECONSTRUCTIONMODE
			else if(RobotStatus.PinsDetectedDown)
			{
				Mode.Mode=CONSTRUCTIONMODE;
				Mode.LastMode=NORMALMODE;
				i=10;
			}
		#endif
	}
	#if USETUNNELANALISYS
		else if(Mode.Mode==TUNNELMODE)
		{
			
			if (!CR_IsTunnelUp()/*CR_IsNotTunnel()*/)
			{
				Mode.Mode=NORMALMODE;
				Mode.LastMode=TUNNELMODE;
				i=8;
			}
		}
	#endif	
	#if USECONSTRUCTIONMODE
		else if(Mode.Mode==CONSTRUCTIONMODE)
		{
			
			if (!RobotStatus.PinsDetectedAll)
			{
				Mode.Mode=NORMALMODE;
				Mode.LastMode=CONSTRUCTIONMODE;
				i=11;
			}
		}
	#endif	

	
	
	if (Mode.Mode!=Mode.PreviousMode)
	{
		
		if (Mode.PreviousMode==NORMALMODE)
		{	
			TM_UpdateTick(&Time.Tick.NormalModeEnd);
			Time.Since.InNormalMode=(int)( ((Time.Tick.NormalModeEnd-(Time.Tick.NormalModeStart))/(Time.Tickspermicrosec*1000)));	
		}
		else if (Mode.PreviousMode==STOPMODE)
		{	
			TM_UpdateTick(&Time.Tick.StopModeEnd);
			Time.Since.InStopMode=(int)(((Time.Tick.StopModeEnd-(Time.Tick.StopModeStart))/(Time.Tickspermicrosec*1000)));	
		}
		else if (Mode.PreviousMode==TUNNELMODE)
		{	
			TM_UpdateTick(&Time.Tick.TunnelModeEnd);
			Time.Since.InTunnelMode=(int)(((Time.Tick.TunnelModeEnd-(Time.Tick.TunnelModeStart))/(Time.Tickspermicrosec*1000)));	
		}	
		else if (Mode.PreviousMode==CONSTRUCTIONMODE)
		{
			TM_UpdateTick(&Time.Tick.ConstructionModeEnd);
			Time.Since.InConstructionMode=(int)(((Time.Tick.ConstructionModeEnd-(Time.Tick.ConstructionModeStart))/(Time.Tickspermicrosec*1000)));
		}
		
		
		if (Mode.Mode==NORMALMODE)
			TM_UpdateTick(&Time.Tick.NormalModeStart);
		else if (Mode.Mode==STOPMODE)
			TM_UpdateTick(&Time.Tick.StopModeStart);
		else if (Mode.Mode==TUNNELMODE)
			TM_UpdateTick(&Time.Tick.TunnelModeStart);
		else if (Mode.Mode==CONSTRUCTIONMODE)
			TM_UpdateTick(&Time.Tick.ConstructionModeStart);
		
	}
	
	
	
	
	AF_ModeOutput(i);
		
}

#endif

