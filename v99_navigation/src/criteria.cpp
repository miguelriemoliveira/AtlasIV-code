/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: tunnelnav.cpp********************************************/
/***************************************************************/
/*! @file criteria.cpp
* @brief All the criterias used to make bolean decisions are here defined (perhaps not all of them :) ).
*/

#ifndef _CRITERIA_
#define _CRITERIA_

#include "header.h"


/**@brief Criteria used to decide if the robot is in the tunnel or not.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsTunnelUp(void)
{

	if (SensorValues.Tunnel_Filtered< MAXDISTTOBETUNNEL)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot has the tunnel on its left side. Uses averages between the side values of the sensors.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsTunnelLeft(void)
{

	if (SensorValues.FrontLeftFilt<= 12 && SensorValues.LeftFrontFilt<= 12)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot has the tunnel on its right side. Uses averages between the side values of the sensors.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsTunnelRight(void)
{

	if (SensorValues.FrontRightFilt<= 12 && SensorValues.RightFrontFilt<= 12)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot is not anymore inside the tunnel. Is called only when the robot is in tunnel mode and uses sensors and timeouts.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsNotTunnel(void)
{

	if (Time.Since.LastModeChange>TIMETOWAITAFTERLASTTUNNEL && ( SensorValues.Tunnel_Filtered>=MAXDISTTOBETUNNEL && !CR_IsTunnelLeft() && !CR_IsTunnelRight()))
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot is detecting the cross. Uses cross bolean sensor and timeouts.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsCloseCross(void)
{

	if (Time.Since.LastStopMode>4000 && SensorValues.Cross==1)
		return(1);
	else
		return(0);

}

int CR_IsCloseCross_1(void)
{
// 	for (int i=0;i<9;i++)
// 		cout << "Ocupation.N[" << i << "].AreaOcupationPercent = " <<  Ocupation.N[i].AreaOcupationPercent << endl;
		
// 	if (Time.Since.LastStopMode>4000 && macro_min(Ocupation.N[0].AreaOcupationPercent, Ocupation.N[1].AreaOcupationPercent)>25 /*&& Ocupation.N[2].AreaOcupationPercent < 20 */&& Mode.DistantCross==CROSSFOUND)
// 		return(1);
// 	else
// 		return(0);

	if (Time.Since.LastStopMode>4000 && CrossAnalisys.MARectangle.Center[1] > 180 && SensorValues.Cross==1)
	{
		return(1);
	}
	else
	{
		return(0);
	}

}

/**@brief Criteria used to decide if the robot is to enter normal mode because of stopmode timeout. Uses timeouts.
 *  
 * @return bolean value (1 or 0)
 */
int CR_IsStopModeTimeout(void)
{

	if (Time.Since.LastModeChange>TIMETOWAITINSTOPMODE)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot sees a GreenArrowUp. Uses LightsAnalisys.DetectedSymbol enum value.
 * 
 * @return bolean value (1 or 0)
 */
int CR_IsGreenArrowUp(void)
{

	if (LightsAnalisys.DetectedSymbol==DSy_GREENARROWUP)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot sees a YellowArrowLeft. Uses LightsAnalisys.DetectedSymbol enum value.
 *  
 * @return bolean value (1 or 0)
 */
int CR_IsYellowArrowLeft(void)
{

	if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWLEFT)
		return(1);
	else
		return(0);

}

/**@brief Criteria used to decide if the robot sees a YellowarrowRight. Uses LightsAnalisys.DetectedSymbol enum value.
 *  
 * @return bolean value (1 or 0)
 */
int CR_IsYellowArrowRight(void)
{

	if (LightsAnalisys.DetectedSymbol==DSy_YELLOWARROWRIGHT)
		return(1);
	else
		return(0);

}
#endif

