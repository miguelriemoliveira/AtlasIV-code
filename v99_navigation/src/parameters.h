/***************************************************************/
/**TITLE: ROBOT navegation fprotos header file************************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: parameters.h********************************************/
/***************************************************************/
/*! @file parameters.h
* @brief This header is used to change several code parameters (mainly preprocessor directive flags). Particular parameter information is defined below.
*/
#ifndef _PARAMETERS_HEADER_
#define _PARAMETERS_HEADER_

/*«««««««« Preprocessing Directives ««««««««««*/

/*! @define POUT
* @brief This is used to print out information to the shell.
*/
#define POUT 1 				
//flag to print out

/*! @define NPOUT
* @brief This is used not to print out information to the shell.
*/
#define NPOUT 0 			
//flag not to print out

/*! @define IPAINT
* @brief This is used to draw in the window's images.
*/
#define IPAINT 1			
//flag to paint image with values

/*! @define USESIGNALS
* @brief Defines whether robot's signals are going to be used or not. 
*/
#define USESIGNALS 1

/*! @define USELIGHTSREADING
* @brief Defines whether the program is going to try to read the lights or not.
*/
#define USELIGHTSREADING 1

/*! @define USETUNNELANALISYS
* @brief Defines whether the program is going to try to find the tunnel and use tunnel navigation.
*/
#define USETUNNELANALISYS 0

/*! @define USECONSTRUCTIONMODE
* @brief Defines whether the program is going to try to find the construction pins and navigate in construction mode.

*/
#define USECONSTRUCTIONMODE 0

/*! @define USEVISIONTUNNELANALISYS
* @brief Defines whether the program is going to try to find the tunnel by vision and use tunnel vision navigation.
*/
#define USEVISIONTUNNELNAV 0

/*! @define USEPARKING
* @brief Defines whether the program is going to park after the run.
*/
#define USEPARKING 0

/*! @define USEDISTANTLIGHTSREADING
* @brief Defines whether the program is going to try to find the lights at the distance and act accordingly.
*/
#define USEDISTANTLIGHTSREADING 0


#define USESPEEDCONTRO 0
/*----------------------------------*/

/*-------------------------------------------------------------*/
/*««««««««««« Definitions for the application «««««««««««««««««*/
/*-------------------------------------------------------------*/

/*«««««««« Communications Config««««««««««*/
/*! @define COM_DEVICE_PIC1
* @brief Defines the path to the serial port connected to pic 2
*/
#define COM_DEVICE_PIC1 "/dev/ttyUSB0"	
//device to assign to PIC1

/*! @define USEPIC1COMS
* @brief Flag to use or not pic1 comunications.
*/
#define USEPIC1COMS 1			
//flag to use PIC1 comunnications [0 or 1]

/*! @define COM_DEVICE_PIC2
* @brief Defines the path to the serial port connected to pic 2
*/
#define COM_DEVICE_PIC2 "/dev/ttyUSB1"	
//device to assign to PIC2
/*! @define USEPIC2COMS
* @brief Flag to use or not pic2 comunications.
*/
#define USEPIC2COMS 1  			
//flag to use PIC2 comunnications [0 or 1]
/*----------------------------------*/
	
			//

/*! @define RIGHTCAM_N
* @brief video1394 device that corrsponds to the right navegation camera.
*/
#define RIGHTCAM_N 2		
//

/*! @define LEFTCAM_N
* @brief video1394 device that corrsponds to the right navegation camera.
*/
#define LEFTCAM_N 0			
//



/*! @define LIGHTSCAM_N
* @brief video1394 device that corrsponds to the lights camera.
*/
#define LIGHTSCAM_N 1			
//IMPORTANT: LIGHTSCAM is 1 because it is receiving power from NAVCAM
/*----------------------------------*/

/*««««« Calculation Assuptions «««««*/
#define PREFEREDSEED cvPoint(ImagesParams.NavImgSize.width/2,ImagesParams.NavImgSize.height-5);	
#define SEED cvPoint(ImagesParams.NavImgSize.width/2,ImagesParams.NavImgSize.height-5)		
#define SEED1 cvPoint(ImagesParams.NavImgSize.width/2,2)		
#define PRESENTPOINT cvPoint((ImagesParams.NavImgSize.width-1)/2,ImagesParams.NavImgSize.height-1) 
#define IMAGESIZE cvSize(320,240)	
//WARNING: other possibilities not tested. Probably crashes

/*«««««««««« Signals ids «««««««««««*/
#define ORDERLEFTTURN 0
#define ORDERRIGHTTURN 1
#define ORDERHEADLIGHTS 2
#define ORDERTAILLIGHTS 3
#define ORDERSENDSENSORS 4
/*----------------------------------*/

/*! @define TOTALLAPS
* @brief How many half laps are to be acomplished.
*/
#define TOTALLAPS 4

/*! @define PARKINGSPEED
* @brief Speed [0 to 15] to be used when in parking manouvers.
*/
#define PARKINGSPEED 2
#define RIGHTBLINDTURNSPEED 4
#define LEFTBLINDTURNSPEED 4

/*! @define TUNNELTHRESHOLDLIMIT
* @brief Threshold limit to use with tunnel vision navegation.
*/
#define TUNNELTHRESHOLDLIMIT 120

#define TLNSize 129	
//#define TDTSSize 19	
#define TDTSSize 11	
#define MAXADMISSIBLEROADWIDTH 1.1 

#endif



