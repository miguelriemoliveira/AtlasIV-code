/***************************************************************/
//**TITLE: ROBOT navegation types definitions header file*******/
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
/*! @file typesdefinitions.h
* @brief All the structures uses in the program are defined here.
*/
#ifndef _TYPEDEFINITION_HEADER_
#define _TYPEDEFINITION_HEADER_

#include "mycameraclass_v1394.h"

/*! @brief This struct is responsible for the output control of the program.	
*   Any output that the whole program might have can theoreticaly be turned on or off. All the flags that regard this are within this struct.
*/
typedef struct{
		char showshellinfo; //!< Images generated in the program can be saved. This functionality is obsolete and might not work.
		char saveimages; //!< If set, the program writes several info to the shell.
		char previewwindows[5]; //!< This variable is responsible for defining which windows will be previewed: [0]=rawimage;[4]=final image
		}outputdefinitions;
 
/*! @brief This struct is used to group the variables from equation y-y0=m*(x-x0)+b.
* Besides having the variables from the equation, it also has a tag.
*/
typedef struct{
		float m; //!< m is the tangent of the line angle of inclination.
		float b; //!< b regards the value of y when x=0.
		int x1;  //!< point x1 of the line.
		int y1;	 //!< point y1 of the line.
		int x2;  //!< point x2 of the line.
		int y2;  //!< point y2 of the line.
		int tag; //!< tag to seen by some routine. I think this is used to see if this line exits or if it has been recalculated in the current iteration.
		}LineParametersStruct;

/*! @brief This struct is used to group variables that have info regarding lines.
* This is used to encapsulate variables used in the Hough lines analysis.
*/				
typedef struct{
		CvSeq *LPoints; //!< *LPoints pointer to the points of the line. Points are of CvPoint type.
		CvSeq *m; 	//!< *m is the of the line that unites each two points. Size = Size(*LPoints)-1.
		float ref_m;    //!< The Average of the m described above.
		float ref_b;	//!< The Average of the b values for all the lines.
		LineParametersStruct InterpLine; //!< Struct of type LineParametersStruct.
		}LinesStruct;

/*! @brief Used to encapsulate the LinesStruct variables.
* There can be up to four groups of lines. This struct is used for the Hough line analysis.
*/	
typedef struct{
		LinesStruct G[4];//!< G[4] array of LinesStruct.
		int NumGroups;   //!< Number of groups used in iteration.
		}GroupStruct;
		
/*! @brief Holds the line and column values for a pixel.
*
*/
typedef struct{
		int val[2]; //!< val[0] is the line value and val[1] is the column value.
		char valid; //!< Other functions need to know if this has beem calculated in the last iteration. This flags signals this situation.
		}SqPoints;

/*! @brief Used to store the lenght of the box found with the box analisys. Also has a tag that says if it has been changed.
*
*/		
typedef struct{
		int lenght; //!< lenght of the box in question
		char valid; //!< valid validation flag.
		}BoxType;
		
/*! @brief Box analisys parameters.
* Box analysys uses 4 points, the vertices of a box to navegate.
*/		
typedef struct{
		SqPoints topr; 	//!< topr Top right vertice of box.
		SqPoints topl; 	//!< topl Top left vertice of box.
		SqPoints lowr;	//!< lowr Low right vertice of box.
		SqPoints lowl;	//!< lowl Low left vertice of box.
		}SearchType;
		
/*! @brief Has all of the parameters used in the square test analysys.
* 
*/		
typedef struct{
		SearchType MSearch;		//!< MSearch uses searches that are made from the middle to the perifery of the image.
		SearchType OSearch;		//!< MSearch uses searches that are made from the borders to the middle of the image.
		BoxType Boxlow;			//!< Boxlow size of the line that unites the low pixels.
		BoxType Boxtop;			//!< Boxtop size of the line that unites the top pixels.
		CvPoint PresentPoint;		//!< PresentPoint point from wich the fill starts (seed point).
		
		int LowGoToPoint[2];		//!< LowGoToPoint line and column indexes of the pixel in question.
		int LowGoToPointWidth;
		char ValidLowGoToPoint;		//!< ValidLowGoToPoint validation flag.
		char RelocatedLowGoToPoint;
		char MaxRoadWidthExcededLowGoToPoint;
		
		int TopGoToPoint[2];		//!< TopGoToPoint line and column indexes of the pixel in question.
		int TopGoToPointWidth;
		char ValidTopGoToPoint;		//!< ValidTopGoToPoint validation flag.
		char RelocatedTopGoToPoint;
		char MaxRoadWidthExcededTopGoToPoint;
		
		int LowLeftGoToPoint[2];	//!< LowLeftGoToPoint line and column indexes of the pixel in question.
		int LowLeftGoToPointWidth;
		char ValidLowLeftGoToPoint;	//!< ValidLowLeftGoToPoint validation flag.
		char RelocatedLowLeftGoToPoint;
		
		int LowRightGoToPoint[2];	//!< LowRightGoToPoint line and column indexes of the pixel in question.
		int LowRightGoToPointWidth;
		char ValidLowRightGoToPoint;	//!< ValidLowRightGoToPoint validation flag.
		char RelocatedLowRightGoToPoint;
		
		int BothRightGoToPoint[2];	//!< BothRightGoToPoint line and column indexes of the pixel in question.
		int BothRightGoToPointWidth;
		char ValidBothRightGoToPoint;	//!< ValidBothRightGoToPoint validation flag.
		
		int BothLeftGoToPoint[2];	//!< BothLeftGoToPoint line and column indexes of the pixel in question.
		int BothLeftGoToPointWidth;
		char ValidBothLeftGoToPoint;	//!< ValidBothLeftGoToPoint validation flag.
		}SquareTest;

/*! @brief Used to store points coordenates and test if they are steping the line.
* 
*/
typedef struct{
		int LowRight[2]; 	//!< LowRight line and column indexes of the point in question.
		int TopRight[2];	//!< TopRight line and column indexes of the point in question.
		int LowLeft[2];		//!< LowLeft line and column indexes of the point in question.
		int TopLeft[2];		//!< TopLeft line and column indexes of the point in question.
		}TypeLineSteping;		

/*! @brief Drive angle (DA) calculated by each of the box analysis. Also has info regarfing linesteping.
* 
*/		
typedef struct{
		float BothLow[2]; 	//!< BothLow DA by this analysis. First the value and then the validation flag.
		float BothTop[2]; 	//!< BothTop DA by this analysis. First the value and then the validation flag.
		float LowLeft[2];	//!< LowLeft DA by this analysis. First the value and then the validation flag.
		float LowRight[2];	//!< LowRight DA by this analysis. First the value and then the validation flag.
		float BothLeft[2];	//!< BothLeft DA by this analysis. First the value and then the validation flag.
		float BothRight[2];	//!< BothRight DA by this analysis. First the value and then the validation flag.
		TypeLineSteping BothTopLineSteping; //!< BothTopLineSteping Info on the line steping.
		}TypeDriveAngleBy;		
		
/*! @brief Robot status information is stored here. 
* 
*/				
typedef struct{
		CvPoint PreferedSeed; 			//!< PreferedSeed prefered seed point to execute the fill operation. 
		CvPoint seed;				//!< seed seed point to execute the fill operation.
		CvPoint seed_a;
		CvPoint seed1;				//!< seed1 seed point to execute the fill operation.
		int PreferedHorizon;			//!< PreferedHorizon Column index in wich the horizon is preferably placed.
		int Horizon;				//!< Horizon Column index in wich the horizon is placed.
		int MinimumLineWidth;			//!< MinimumLineWidth is the least amount of columns that a line must have to be considered as such.
		TypeDriveAngleBy DA;			//!< DA TypeDriveAngleBy struct using normal analysis. 
		TypeDriveAngleBy DA_PP;			//!< DA_PP TypeDriveAngleBy struct using _PP analysis.
		float LastDA[5];			//!< LastDA stores the last 5 chosen drive angles.
		float ChosenDA;				//!< ChosenDA currently chosen DA.
		int ChosenDA_PIC;			//!< ChosenDA_PIC currently chosen DA after being converted to the pic int values.
		int ChosenAnalisys;			//!< ChosenAnalisys wich box analysis was chosen.
		int ChosenGoToPoint[2];			//!< ChosenGoToPoint line and column values of the chosen go to point.
		char ChosenSpeed;			//!< ChosenSpeed speed chosen in the current iteration.
		unsigned char LapNumber; 		//!< LapNumber how many laps have been completed.
		unsigned char PinsDetectedDown;		//!< PinsDetectedDown used in construction mode. Amount of pins that are detected in the low part of the image. Used to soften the entrance in const mode.
		unsigned char PinsDetectedAll;		//!< PinsDetectedAll used in construction mode. Amount of pins that are detected in the image.
		unsigned char BlindTurn;		//!< BlindTurn used to flag this particular situation.
		CvScalar SandV_sumDown; 		//!< SandV_sumDown used again in const mode. Sum of all the pixels that have not been filtered by the SandVFilter. This in a lower part of the image.
		CvScalar SandV_sumAll; 			//!< SandV_sumDown used again in const mode. Sum of all the pixels that have not been filtered by the SandVFilter.
		}TypeRobotStatus;


/*! @brief Contains arrays of 3 elements for some colors that are frequently used.
* This color values have three integer values corresponding to the RGB values that are going to be inserted in the cvColor() function.
*/		
typedef  struct{
		int red[3];	//!< red color definition
		int green[3];	//!< green color definition
		int blue[3];	//!< blue color definition
		int yellow[3];	//!< yellow color definition
		int cyan[3];	//!< cyan color definition
		int orange[3];	//!< orange color definition
		int magenta[3];	//!< magenta color definition
		int black[3];	//!< black color definition
		}TypeColor;		

/*! @brief Contains all int64 type variables that are used to store the amount of ticks that have passed since determined events.
* The events that refer to every variable are easily understood by analysing it's name.
*/					
typedef  struct{
		int64 ProgramStart;
		int64 LastIteration;
		int64 LastCross;
		int64 LastModeChange;
		int64 LastLapCount;
		int64 StartBlindTurn;
		int64 Now;
		int64 StopModeStart;
		int64 NormalModeStart;
		int64 TunnelModeStart;
		int64 ConstructionModeStart;
		int64 StopModeEnd;
		int64 NormalModeEnd;
		int64 TunnelModeEnd;
		int64 ConstructionModeEnd;
		int64 Tick1;
		int64 Tick2;
		int64 Tick3;
		int64 Tick4;
		int64 Tick5;
		int64 Tick6;
		int64 Tick7;
		int64 Tick8;
		}TypeTicks;		
		
/*! @brief Contains all int type variables that are used to store the amount of ticks that have passed since determined events.
* The events that refer to every variable are easily understood by analysing it's name.
*/							
typedef  struct{
		int ProgramStart;
		int LastIteration;
		int LastCross;
		int LastModeChange;
		int LastLapCount;
		int LastBlindTurn;
		int InNormalMode;
		int InStopMode;
		int InTunnelMode;
		int InConstructionMode;
		}TypeSince;				

/*! @brief Contains all TypeSince and TypeTicks and also a double that is used to convert the first's in the second's.
*
*/
typedef  struct{
		double Tickspermicrosec; //!< Tickspermicrosec the amount of ticks that ocour in a microsecond.
		TypeTicks Tick;		//!< Tick all the ticks for every event.
		TypeSince Since;	//!< Since time since every event.
		}TypeTime;		

/*! @brief Contains device handles used in the program.
*
*/				
typedef  struct{
		int ComPort_0;			//!< ComPort_0 handle to the serial port 0.
		int ComPort_1;			//!< ComPort_1 handle to the serial port 1.
		mycameraclass_v1394 *RightCam;    //!< 
		mycameraclass_v1394 *LeftCam;    //!< 
		mycameraclass_v1394 *NavCam;    //!< NavCam handle of the navegation camera.
		mycameraclass_v1394 *LightsCam; //!< LightsCam handle of the lights camera.
		}TypeHandle;		

/*! @brief Contains prefered direction status.
*
*/		
typedef enum{
		NONE=0,			//!< NONE prefered direction is none.
		TAKELEFT,		//!< LEFT prefered direction is left.
		TAKERIGHT		//!< RIGHT prefered direction is right.
		}TypeEnumDirection;

/*! @brief Enumeration of the find cross possibilities.
*
*/		
typedef enum{
		NOCROSSFOUND=0,
		CROSSFOUND
		}TypeEnumCross;

/*! @brief Flag to signal if speed mode should be normal or cross aproach.
*
*/				
typedef enum{
		NORMALSPEEDFLAG=0,
		CROSSAPROACHSPEEDFLAG
		}TypeEnumSpeed;

/*! @brief Signals the mode in wich the robot is presently.
*
*/				
typedef enum{
		DUMMYMODE=-1,
		NORMALMODE,
		STOPMODE,
		APROACHINGCROSSMODE,
		PARKINGMODE,
		TUNNELMODE,
		CONSTRUCTIONMODE
		}TypeEnumm;
		

/*! @brief Encapsulates all other enums.
*
*/						
typedef struct{
		TypeEnumDirection Direction;
		TypeEnumCross DistantCross;
		TypeEnumSpeed Speed;
		TypeEnumm Mode;
		TypeEnumm PreviousMode;
		TypeEnumm LastMode;
		}TypeMode;		

		
		
/*! @brief Shows the current state of the robot's lights.
*
*/		
typedef struct{
		unsigned char TurnLeft;
		unsigned char TurnRight;
		unsigned char HeadLights;
		unsigned char TailLights;
		}TypeSignalsState;		
						

/*! @brief Enumeration that says wich color was detected.
*
*/				
typedef enum{
		DC_DUMMY=-1,
		DC_NONE,
		DC_GREEN,
		DC_RED,
		DC_YELLOW
		}TypeEnumDetectedColor;		

/*! @brief Struct that stores the amount of pixels that were filtered by the green and red filter. 
*
*/				
typedef struct{
		unsigned int PC_RedCount;	//!< PC_RedCount is the amount of pixels filtered by the red filter.
		unsigned int PC_GreenCount;	//!< PC_GreenCount is the amount of pixels filtered by the green filter.
		unsigned int PC_YellowCount;	//!< 
		unsigned int PC_MaxCount;	//!< Holds the max of PC_RedCount and PC_GreenCount.
		unsigned int PC_MinCount;	//!< Holds the min of PC_RedCount and PC_GreenCount.
		}TypePixelCounts;				

/*! @brief Variables used in the caracterization of the minimum area rectangle (MAR).
*
*/		
typedef struct{
		float Center[2];	//!< x and y coordenates of the MAR center point.
		CvPoint boxpts[4];	//!< x and y coordenates of the MAR vertices.
		int width;		//!< MAR width.
		int height;		//!< MAR height.
		int Area;		//!< MAR area.
		}TypeMARectangle;			

/*! @brief Enumeration to define the detected shape.
*
*/		
typedef enum{
		DS_DUMMY=-1,
		DS_NONE,
		DS_OBJECTORIENTEDRIGHT,
		DS_OBJECTORIENTEDLEFT,
		DS_OBJECTORIENTEDUP,
		DS_OBJECTORIENTEDSQUARE //it happens with cross and park
		}TypeEnumDetectedShape;		

/*! @brief Enumeration to define the decision made when merging color and shape recognition.
*
*/				
typedef enum{
		DSy_DUMMY=-1,
		DSy_NONE,
		DSy_YELLOWARROWLEFT,
		DSy_YELLOWARROWRIGHT,
		DSy_GREENARROWUP,
		DSy_REDCROSS,
		DSy_REDGREENPARK,
		}TypeEnumDetectedSymbol;

/*! @brief Holds every variable that regards the lights analysis.
*
*/
typedef struct{
		//color recognition
		TypeEnumDetectedColor DetectedColor;	//!< Wich color was detected.
		TypePixelCounts PixelCounts;		//!< How many pixels were counted.
		//shape recognition
		CvSeq* pts;				//!< Sequence of points used in the shape recognition.
		CvSeq* fillpts;				//!< Sequence of points that are obtained from the fill.
		CvSeq* hull;				//!< Sequence of points used in the convex hull.
		int HullArea;				//!< Area of the convex hull.
		float OcupationPercent;			//!< Ocupation of the convex hull vs the MAR.
		TypeMARectangle MARectangle;		//!< MAR rectangle.
		CvPoint MassCenter;			//!< Center of mass.
		float MassCenterFx;			//!< Center of mass.
		float MassCenterFy;			//!< Center of mass.
		float ObjectOrientation;		//!< angle of the line that unites MAR center of mass and Convex hull center of mass.
		TypeEnumDetectedShape DetectedShape;	//!< Shape detection enum.
		TypeEnumDetectedSymbol DetectedSymbol;	//!< Symbol detection enum.
		}TypeLightsAnalisys;		

/*! @brief Holds every variable used in the cross analysis.
*
*/
typedef struct{
		int PixelNum;
		CvSeq* pts;
		TypeMARectangle MARectangle;
		int CrossYVal;			//!< Line values of the cross. (May be used for speed control).
		float MARAngle;			//!< Angle of the cross.
		float HeightWidthRatio;		//!< Ratio between the height and width of the MAR rectangle (is more or less fixed in our case)
		unsigned char AttemptedSpots;
		
		}TypeCrossAnalisys;		
		

/*! @brief Holds the raw values given by the sensors. Sensor nature is easily captioned by it's name.
*
*/	
typedef struct{
		unsigned char Cross;
		unsigned char RightFront[10];
		unsigned char RightFrontFilt;
		unsigned char RightBack[10];
		unsigned char RightBackFilt;
		unsigned char FrontRight[10];
		unsigned char FrontRightFilt;
		unsigned char FrontLeft[10];
		unsigned char FrontLeftFilt;
		unsigned char LeftFront[10];
		unsigned char LeftFrontFilt;
		unsigned char LeftBack[10];
		unsigned char LeftBackFilt;
		unsigned char Tunnel[10];
		unsigned char Tunnel_Filtered;
		
		}TypeSensorValues;		

/*! @brief Holds the  values given by the infrared distance sensors and has some statistical variables.
*
*/
typedef struct{
		unsigned char RightFront;
		unsigned char RightBack;
		unsigned char FrontRight;
		unsigned char FrontLeft;
		unsigned char LeftFront;
		unsigned char LeftBack;
		unsigned char Tunnel;
		float RightSideAvg;
		float LeftSideAvg;
		}TypeSensorDistances;

/*! @brief Holds the SensorDistances and a variable named diference ???.
*
*/
typedef struct{
		TypeSensorDistances SensorDistances;
		float Diference;
		}TypeTunnelAnalisys;

/*! @brief All parking manouvering variables are held here. Park manouvering is done in 3 phases and for each there one has to know how long it takes and which direction is to be assumed. 
*
*/		
typedef struct{
		char ManouverNumber;
		
		int ManouveringTime1;
		int ManouveringDir1;
		
		int ManouveringTime2;
		int ManouveringDir2;
		
		int ManouveringTime3;
		int ManouveringDir3;
		
		}TypeParkAnalisys;


typedef struct {IplImage *imL, *imR, *imJ;
		char *winNameL, *winNameR, *winNameJ;
		int overlapH, bin;
		void (*binOper)(const CvArr*, const CvArr*, CvArr*, const CvArr*);
                int distDataR[8]; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
		int distDataL[8]; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
		char *distname[8]; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
		int msgOn;
		int spMsgOn; 
		CvMat *mapxL, *mapyL, *mapxR, *mapyR;  /*To store distortion maps*/
		} TypeCamCalibparams;		
		
typedef struct{ CvSize NavImgSize;
		CvSize LightsImgSize;

		}TypeImagesParams;
		
//_______________________________________________________________________________________________________		
typedef struct{ CvPoint Area[4];
		double AreaOcupationPercent;
	        unsigned int AreaPixelNum;
		}TypeN;
typedef struct{
		TypeN N[9];
	       	unsigned char BiggestPercentArea;
		}TypeOcupation;

typedef struct{ IplImage* HSV;
		IplImage* H;
		IplImage* S;
		IplImage* V;
		IplImage* S_Filter; 
		IplImage* V_Filter; 
		IplImage* SandV_Filter; 
		IplImage* Orange;
		}TypeConstImg;	
		
typedef struct{ IplImage* HSV;
		IplImage* H;
		IplImage* S;
		IplImage* V;
		IplImage* S_Filter; 
		IplImage* V_Filter; 
		IplImage* SandV_Filter; 
		IplImage* Red_Filter;
		IplImage* Green_Filter;
		IplImage* Yellow_Filter;
		}TypeLightsImg;		

typedef struct{
		IplImage* RightCam;
		IplImage* LeftCam;
		
		IplImage* RightCam_1ch;
		IplImage* LeftCam_1ch;
	
		IplImage* RightCam_Undistorted;
		IplImage* LeftCam_Undistorted;
		
		IplImage* RoadImg;
		}TypeOriginalImg;		

typedef struct{
		IplImage *InnerSpotsImg;
		IplImage *CurrentSpotsImg;
		IplImage *TempImg;
		CvMat *test1;
		CvMat *test2;
		CvMat *test3;
		CvMat *CurrentSpotsMat;
		CvMat *InnerSpotsMat;
		CvMat *TempMat;
		}TypeCrossImg;

typedef struct{
		IplImage* Info;
		}TypeDataImages;		
				
typedef struct{
		TypeDataImages Data;
		TypeCrossImg Cross;
		TypeOriginalImg Orig;
		TypeLightsImg Lights;
		TypeConstImg Const;
		}TypeGlobalImages;
		
#endif 
