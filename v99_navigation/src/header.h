/***************************************************************/
/**TITLE: ROBOT navegation header file************************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: header.h********************************************/
/***************************************************************/
/*! @file header.h
* @brief main header. Includes all other headers and defines macros, IPLImage global variables, all general includes, some other global vars and some calibration tables.
*/

#ifndef _NAVHEADER_
#define _NAVHEADER_


//#include <qsound.h>
#include <stdlib.h>
/*Headers that need to be included*/
#include <cv.h>	     //includes opencv definitions
#include <highgui.h> // includes highGUI definitions
#include<iostream.h> //input output definitions
#include <math.h>
#include <stdio.h>
#include <fcntl.h> //para declaraçao dos O_RONLY ... etc
#include <termios.h> //declaraç~ao da estrutura termios
#include <unistd.h> //funçoes tipo read /write necessarias 
#include <string.h>  


#include <mycameraclass_v1394.h> //caetano
#include <joinimg.h> //vs
#include <pa.h> //vs


#include "typesdefinitions.h"
#include "fprotos.h"
#include "parameters.h"
#include "global_vars.h"




/**********************************/

/*macros definitions*/
#define macro_min(a,b) ((a)<=(b) ?( a) :( b))	//macro to get minimum value
#define macro_max(a,b) ((a)>=(b) ? (a) : (b))	//macro to get maximum value
#define macro_isbetween(val,low,up) ((((val)>(low)) && ((val)<(up))) ? (1) : (0))	//macro see if value is between limits

#define saturatemax(val,max) ((val)>(max) ?( max) :( val))
#define saturatemin(val,min) ((val)<(min) ?( min) :( val))

/**********************************/

		
#ifdef _MAIN_FILE_
/*global variables declaration*/
	
	TypeGlobalImages GI;
	
	IplImage *RoadImg_gray;
	IplImage *RoadImg_rgb_hough;
	IplImage *RoadImg_gray_filled;
	IplImage *FindCross_src;
	IplImage *mask_Img;
	IplImage *mask_Img1;
	IplImage *cross_Img;
	IplImage *LightsImg_Original=cvCreateImage( IMAGESIZE, IPL_DEPTH_8U, 3 );
	
	
	IplImage* SandV_Filter1 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	IplImage* workimg = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	IplImage* workimg1 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	IplImage* img1 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
	
	
	
	
	
	//#if USEDISTANTLIGHTSREADING
		IplImage* HSV_Roadimg= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
		IplImage* H_Roadimg= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
		IplImage* S_Roadimg= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
		IplImage* V_Roadimg= cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	//#endif
	
	IplImage* TunnelImg = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	
	outputdefinitions outputdef;
	
	TypeRobotStatus RobotStatus;

	//TLNSize defined in parameters.h
	int Table_LineNumber[TLNSize] = {111 ,112 ,113 ,114 ,115 ,116 ,117 ,118 ,119 ,120 ,121 ,122 ,123 ,124 ,125 ,126 ,127 ,128 ,129 ,130 ,131 ,132 ,133 ,134 ,135 ,136 ,137 ,138 ,139 ,140 ,141 ,142 ,143 ,144 ,145 ,146 ,147 ,148 ,149 ,150 ,151 ,152 ,153 ,154 ,155 ,156 ,157 ,158 ,159 ,160 ,161 ,162 ,163 ,164 ,165 ,166 ,167 ,168 ,169 ,170 ,171,172 ,173 ,174 ,175 ,176 ,177 ,178 ,179 ,180 ,181 ,182 ,183 ,184 ,185 ,186 ,187 ,188 ,189 ,190 ,191 ,192 ,193 ,194 ,195 ,196 ,197 ,198 ,199 ,200 ,201 ,202 ,203 ,204 ,205 ,206 ,207 ,208 ,209 ,210 ,211 ,212 ,213 ,214 ,215 ,216 ,217 ,218 ,219 ,220 ,221 ,222 ,223 ,224 ,225 ,226 ,227 ,228 ,229 ,230 ,231 ,232 ,233 ,234 ,235 ,236 ,237 ,238 ,239}; //warning: if camera position is changed linenumber & roadwidth have to be recalibrated. Use MeasureRoadDistance for that
	
	int Table_RoadWidth[TLNSize] = {196 ,198 ,200 ,203 ,205 ,208 ,210 ,212 ,215 ,217 ,220 ,222 ,224 ,227 ,229 ,231 ,234 ,237 ,239 ,242 ,244 ,246 ,249 ,252 ,254 ,256 ,259 ,261 ,264 ,266 ,268 ,271 ,274 ,276 ,278 ,280 ,284 ,286 ,288 ,291 ,293 ,296 ,298 ,301 ,304 ,306 ,309 ,311 ,314 ,317 ,319 ,322 ,324 ,327 ,329 ,332 ,334 ,337 ,340 ,342 ,345,348 ,350 ,353 ,356 ,358 ,360 ,364 ,366 ,368 ,371 ,374 ,376 ,380 ,382 ,385 ,387 ,390 ,393 ,395 ,398 ,401 ,403 ,406 ,409 ,411 ,414 ,417 ,419 ,422 ,425 ,428 ,430 ,433 ,436 ,439 ,442 ,444 ,447 ,450 ,452 ,456 ,458 ,461 ,464 ,467 ,469 ,473 ,475 ,479 ,481 ,484 ,487 ,490 ,492 ,496 ,498 ,502 ,504 ,507 ,511 ,513 ,517 ,519 ,523 ,526 ,528 ,532 ,534}; //warning: if camera position is changed linenumber & roadwidth have to be recalibrated. Use MeasureRoadDistance for that

	int *Table_LineNumber_read;
	int *Table_RoadWidth_read;
	
	int Table_DirToSend[TDTSSize]={0 , 10 , 20 , 30 , 40 , 45 , 50 , 60, 70 , 80 ,90};

	int Table_DirAngle[TDTSSize]={34,37,41,43,44,45,46,48,50,53,56}; //values that correspond physicaly to the direction the robot takes when given the corresponding values of Table_DirToSend
	
// 	int Table_DirToSend[TDTSSize]={0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90};
// 
// 	int Table_DirAngle[TDTSSize]={38,39,41,42,43,44,44,44,44,45,46,46,46,46,47,48,50,51,53}; //values that correspond physicaly to the direction the robot takes when given the corresponding values of Table_DirToSend

	unsigned char Table_SideSensorsComValues[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; //this is compliant with RobotSensors_v7 in PIC2 finished on the 30th of March, 2005
	
	unsigned char Table_SideSensorsDistances[16]={10,13,16,18,21,24,27,30,32,35,38,41,50,55,60,70};//this is compliant with RobotSensors_v7 in PIC2 finished on the 30th of March, 2005
	
	TypeTime Time;
	
	TypeHandle Handle;
	
	TypeMode Mode;
	
	TypeSignalsState SignalsState;
	
	TypeColor Color;
	
	SquareTest SquareRegion;
	
	SquareTest ConstSquareRegion;
	
	TypeLightsAnalisys LightsAnalisys;
	
	TypeCrossAnalisys CrossAnalisys;
	
	TypeSensorValues SensorValues;
	
	TypeTunnelAnalisys TunnelAnalisys;
	
	TypeParkAnalisys ParkAnalisys;
	
	CvFont font;
	CvFont smallfont;
	CvFont hugefont;
	
	TypeImagesParams ImagesParams;
	
	TypeOcupation Ocupation;
	
#else
	extern TypeGlobalImages GI;
	
	
	extern IplImage* RoadImg_gray;
	extern IplImage *RoadImg_rgb_hough;
	extern IplImage *RoadImg_gray_filled;
	extern IplImage *FindCross_src;
	extern IplImage *mask_Img;
	extern IplImage *mask_Img1;
	extern IplImage *cross_Img;
	extern IplImage *LightsImg_Original;
 	
	extern IplImage* SandV_Filter1;
	extern IplImage* workimg;
	extern IplImage* workimg1;
	extern IplImage* img1;
	
	//#if USEDISTANTLIGHTSREADING
		extern IplImage* HSV_Roadimg;
		extern IplImage* H_Roadimg;
		extern IplImage* S_Roadimg;
		extern IplImage* V_Roadimg;
	//#endif
	
	extern IplImage* TunnelImg;
	
	
	extern outputdefinitions outputdef;
	extern TypeRobotStatus RobotStatus;
	extern int Table_LineNumber;
	extern int Table_RoadWidth;
	extern int *Table_LineNumber_read;
	extern int *Table_RoadWidth_read;
	extern int Table_DirToSend;
	extern int Table_DirAngle;
	extern unsigned char Table_SideSensorsComValues[16];
	extern unsigned char Table_SideSensorsDistances[16];
	extern TypeTime Time;
	extern TypeHandle Handle;
	extern TypeMode Mode;
	extern TypeSignalsState SignalsState;
	extern TypeColor Color;
	extern SquareTest SquareRegion;
	extern SquareTest ConstSquareRegion;
	extern TypeLightsAnalisys LightsAnalisys;
	extern TypeCrossAnalisys CrossAnalisys;
	extern TypeSensorValues SensorValues;
	extern TypeTunnelAnalisys TunnelAnalisys;
	extern TypeParkAnalisys ParkAnalisys;
	extern CvFont font;
	extern CvFont smallfont;
	extern CvFont hugefont;
	extern TypeImagesParams ImagesParams;
	extern TypeOcupation Ocupation;
#endif


#endif 
