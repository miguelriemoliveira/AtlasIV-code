// /***************************************************************/
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

/*! @file init.cpp
* @brief All the initialization functions of the program are declared in this file. All Functions declared here start with "IN_"
*
* Besides the normal init functions there is an IN_InitEverything function that holds the proper sequence of initialization.
*/

#ifndef _INIT_
#define _INIT_

#include "header.h"



/** @brief Inits the comports.
 * 
 * @param Port0 port0 to be used
 * @param Port1 port1 to be used
 */
void IN_InitComPort(int *Port0,int *Port1)
{
	printf("Starting up communications.......");
	
	#if USEPIC1COMS 
		*Port0=initrs232(1/*pic number*/); //com port 0 initialization
		SendSpeed(STOPPEDSPEED,*Port0);	
		printf("Done\n");
	#endif
	
	#if USEPIC2COMS
		*Port1=initrs232(2/*pic number*/); //com port 1 initialization
	#endif
}	


/** @brief Inits all the ticks used in the program.
 * 
 */
void IN_TimeInit()
{
	printf("Starting Time Control.......");
	
	Time.Tick.ProgramStart=cvGetTickCount( );
	Time.Tick.LastIteration=cvGetTickCount( );
	TM_UpdateTick(&Time.Tick.LastCross);
	Time.Tick.LastLapCount=cvGetTickCount( );
	Time.Tick.StartBlindTurn=cvGetTickCount( );
	Time.Tick.LastStopMode=cvGetTickCount( );
	
	
	Time.Tickspermicrosec=cvGetTickFrequency();
	
	
	printf("Done\n");
}
/** @brief Inits data window.
 * 
 */
void IN_XWindowsData(void)
{
	printf("Creating Data windows .......");
	cvNamedWindow("Robot Data",1);
	cvResizeWindow( "Robot Data", 960, 200 );
	printf("Done\n");
}

/** @brief Inits navegation windows.
 * 
 */
void IN_XWindowsNavegation(void)
{
	printf("Creating Navegation windows .......");

	
	
	//cvNamedWindow("TunnelImg",1);
	//cvResizeWindow( "TunnelImg", 320, 240 );
	//cvMoveWindow( "TunnelImg", 0,0 );
	
	
	/*Windows declaration*/
// 	cvNamedWindow("RightImg -> Original",1);
// 	cvResizeWindow( "RightImg -> Original", 320,240);
// 	cvNamedWindow("LeftImg -> Original",1);
// 	cvResizeWindow( "LeftImg -> Original", 320,240);
	
	//cvNamedWindow("RightImg -> Undistorted",1);
	//cvResizeWindow( "RightImg -> Undistorted", ImagesParams.NavImgSize.width,ImagesParams.NavImgSize.height);
	//cvNamedWindow("LeftImg -> Undistorted",1);
	//cvResizeWindow( "LeftImg -> Undistorted", ImagesParams.NavImgSize.width,ImagesParams.NavImgSize.height);
	
// 	cvNamedWindow("STEP 1->Original_Img",1);
// 	cvResizeWindow( "STEP 1->Original_Img", ImagesParams.NavImgSize.width,ImagesParams.NavImgSize.height);
// 	
// 	
// 	cvNamedWindow("STEP 2->RoadImg_gray_filled",1);
// 	cvNamedWindow("STEP 3->mask_Img",1);
// 	cvNamedWindow("STEP 4->mask_Img1",1);
// 	cvNamedWindow("OTHER ->cross_Img",1);
// 	cvResizeWindow("OTHER ->cross_Img", 320, 240 );
// 	
// 	cvNamedWindow("STEP 5->Final Image",1);
// 	cvResizeWindow("STEP 5->Final Image", ImagesParams.NavImgSize.width,ImagesParams.NavImgSize.height);
// 	
	
	printf("Done\n");
	cvWaitKey(50);
}

/** @brief Inits lights windows.
 * 
 */
void IN_XWindowsLights(void)
{
	printf("Creating Lights windows .......");

	
	/*Windows declaration*/
	cvNamedWindow("LightsCamera",1);
	cvResizeWindow("LightsCamera", 320, 240);
	//
	
// 	cvNamedWindow("Red_Filter",1);	
// 	cvResizeWindow("Red_Filter", 320, 240);
// 	cvNamedWindow("Green_Filter",1);	
// 	cvResizeWindow("Green_Filter", 320, 240);
// 	cvNamedWindow("Yellow_Filter",1);	
// 	cvResizeWindow("Yellow_Filter", 320, 240);
	cvNamedWindow("S_Filter",1);
	cvResizeWindow("S_Filter", 320, 240);
	cvNamedWindow("V_Filter",1);
	cvResizeWindow("V_Filter", 320, 240);
	
	cvNamedWindow("SandV_Filter",1);
	cvResizeWindow("SandV_Filter", 320, 240 );

	cvNamedWindow("LightsFinal",1);
	cvResizeWindow("LightsFinal", 320, 240 );
	
	//cvNamedWindow("workimg",1);
	
	printf("Done\n");
	cvWaitKey(50);
}

/** @brief Places navegation windows in the proper place.
 * 
 */
void IN_PlaceXwindowsNavegation(void)
{
int FirstWinPos[2]={0,0};

int Winsize[2]={ImagesParams.NavImgSize.width+5 ,ImagesParams.NavImgSize.height+10};

	printf("Placing Navegation windowns.......");

	cvMoveWindow( "STEP 1->Original_Img", FirstWinPos[0], FirstWinPos[1] );
	//cvMoveWindow( "STEP 2->RoadImg_gray_filled", FirstWinPos[0]+Winsize[0], FirstWinPos[1] );
	//cvMoveWindow( "STEP 3->mask_Img", FirstWinPos[0]+2*Winsize[0], FirstWinPos[1] );
	//cvMoveWindow( "STEP 4->mask_Img1", FirstWinPos[0], FirstWinPos[1]+Winsize[1] );
	cvMoveWindow( "STEP 5->Final Image", FirstWinPos[0]+Winsize[0], FirstWinPos[1]);
	cvMoveWindow( "OTHER ->cross_Img", FirstWinPos[0]+2*Winsize[0], FirstWinPos[1]);
	
	printf("Done\n");
}

/** @brief Places lights windows in the proper place.
 * 
 */
void IN_PlaceXwindowsLights(void)
{
int FirstWinPos[2]={315,265};
int Winsize[2]={320,270};

	printf("Placing Lights windows.......");

	cvMoveWindow( "LightsCamera", FirstWinPos[0], FirstWinPos[1]);
	cvMoveWindow( "S_Filter", FirstWinPos[0]+2*Winsize[0], FirstWinPos[1] );
	cvMoveWindow( "V_Filter", FirstWinPos[0], FirstWinPos[1]+Winsize[1] );
	cvMoveWindow( "SandV_Filter", FirstWinPos[0]+Winsize[0], FirstWinPos[1]);
	//cvMoveWindow( "Red_Filter", FirstWinPos[0], FirstWinPos[1] );
	//cvMoveWindow( "Green_Filter", FirstWinPos[0]+Winsize[0], FirstWinPos[1] );
	cvMoveWindow( "LightsFinal", FirstWinPos[0]+2*Winsize[0], FirstWinPos[1]);
	
	printf("Done\n");
}

/** @brief Places data window in the proper place.
 * 
 */
void IN_PlaceXwindowsData(void)
{
int FirstWinPos[2]={315,530};


	printf("Placing Data window.......");
	cvMoveWindow( "Robot Data", FirstWinPos[0], FirstWinPos[1]);
	printf("Done\n");
}


/** @brief Initializes RobotStatus struct.
 * 
 */
void IN_RobotStatus(void)
{
	printf("Initializing Robot Status .......");
	
	RobotStatus.PreferedSeed=PREFEREDSEED;
	RobotStatus.seed=SEED;
	RobotStatus.seed1=SEED1;
	RobotStatus.PreferedHorizon=HORIZON;
	RobotStatus.Horizon=RobotStatus.PreferedHorizon;
	RobotStatus.MinimumLineWidth=MINIMUMLINEWIDTH;
	RobotStatus.LapNumber=0;
	RobotStatus.BlindTurn=0;
	RobotStatus.LastDA[0]=45;
	RobotStatus.LastDA[1]=45;
	RobotStatus.LastDA[2]=45;
	RobotStatus.LastDA[3]=45;
	RobotStatus.LastDA[4]=45;
	RobotStatus.TimesInEscapeMode=0;
		
	printf("Done\n");
}

/** @brief Initializes SquareRegion struct.
 * 
 * @param ST SquareTest typedef. 
 */
void IN_SquareRegion(SquareTest *ST)
{
	printf("Initializing Square Test.......");
	
	ST->PresentPoint=PRESENTPOINT;
	
	printf("Done\n");
}

/**@brief Initializes TypeColor struct with the proper RGB values.
 * 
 * @param p pointer to TypeColor .
 */
void IN_BuildColorTable(TypeColor *p)
{
	printf("Building Color Table .......");

	p->red[0]=255; p->red[1]=0; p->red[2]=0; //Define red
	p->green[0]=0; p->green[1]=255; p->green[2]=0; //Define green
	p->darkgreen[0]=0; p->darkgreen[1]=255; p->darkgreen[2]=0; //Define darkgreen
	p->blue[0]=0; p->blue[1]=0; p->blue[2]=255; //Define blue
	p->darkblue[0]=0; p->darkblue[1]=0; p->darkblue[2]=128; //Define blue
	p->yellow[0]=255; p->yellow[1]=255; p->yellow[2]=0; //Define yellow
	p->darkyellow[0]=128; p->darkyellow[1]=128; p->darkyellow[2]=0; //Define darkyellow
	p->cyan[0]=0; p->cyan[1]=255; p->cyan[2]=255; //Define cyan
	p->darkcyan[0]=0; p->darkcyan[1]=128; p->darkcyan[2]=128; //Define darkcyan
	p->orange[0]=255; p->orange[1]=128; p->orange[2]=0; //Define orange
	p->magenta[0]=255; p->magenta[1]=0; p->magenta[2]=255; //Define orange
	p->darkmagenta[0]=128; p->darkmagenta[1]=0; p->darkmagenta[2]=128; //Define orange
	p->black[0]=0; p->black[1]=0; p->black[2]=0; //Define black
	
	printf("Done\n");
}

/**@brief Initializes the cameras.
 * 
 */
void IN_Cameras(void)
{
	printf("Starting up cameras .......");	

	//Handle.NavCam = new mycameraclass_v1394(NAVCAM_N);
	Handle.RightCam = new mycameraclass_v1394(RIGHTCAM_N);
	Handle.LeftCam = new mycameraclass_v1394(LEFTCAM_N);
	
	Handle.LightsCam = new mycameraclass_v1394(LIGHTSCAM_N); 
	
	printf("Done\n");
}

/**@brief Initializes the ModeStruct.
 * 
 */
void IN_ModeStruct(void)
{
	printf("Initializing ModeStruct.......");
	
	Mode.DistantCross=NOCROSSFOUND;
	Mode.Speed=NORMALSPEEDFLAG;
	Mode.LastMode=DUMMYMODE;
	
	printf("Done\n");
}

/**@brief Initializes the SignalsStateStruct.
 * 
 */
void IN_SignalsStateStruct(void)
{
	printf("Initializing SignalsStateStruct....");
	
	SignalsState.TurnLeft=0;
	SignalsState.TurnRight=0;
	SignalsState.HeadLights=0;
	SignalsState.TailLights=0;
	
	printf("Done\n");
}

/**@brief Waits for a key to start program.
 * 
 */
void IN_PromptToStart(void)
{
    cvWaitKey(100);
	
    	
    
	*LightsImg_Original = Handle.LightsCam->myGetImageColor();
	cvConvertImage(LightsImg_Original,LightsImg_Original,CV_CVTIMG_SWAP_RB);
	
	*GI.Orig.LeftCam = Handle.LeftCam->myGetImageColor();
	cvConvertImage(GI.Orig.LeftCam ,GI.Orig.LeftCam ,CV_CVTIMG_SWAP_RB);
	cvCvtPixToPlane(GI.Orig.LeftCam,GI.Orig.LeftCam_1ch,NULL,NULL,0);	
	
	*GI.Orig.RightCam = Handle.RightCam->myGetImageColor();
	cvConvertImage(GI.Orig.RightCam,GI.Orig.RightCam,CV_CVTIMG_SWAP_RB);
	cvCvtPixToPlane(GI.Orig.RightCam,GI.Orig.RightCam_1ch,NULL,NULL,0);	
	
	UndistortImage(GI.Orig.RightCam, NULL,      CamCalibparams.distDataR, & CamCalibparams.mapxR, & CamCalibparams.mapyR ) ;  //initialize maps
	UndistortImage(GI.Orig.LeftCam, NULL,      CamCalibparams.distDataL, & CamCalibparams.mapxL, & CamCalibparams.mapyL ) ;  //initialize maps
	
	UndistortImage(GI.Orig.RightCam, GI.Orig.RightCam_Undistorted , CamCalibparams.distDataR, & CamCalibparams.mapxR, & CamCalibparams.mapyR ) ;  //apply maps
	UndistortImage(GI.Orig.LeftCam, GI.Orig.LeftCam_Undistorted ,  CamCalibparams.distDataL, & CamCalibparams.mapxL, & CamCalibparams.mapyL ) ;  //apply maps
	
	

	
	
	#if NPOUT
		cout << "__________Join image report_______________" << endl;
		cout << "GI.Orig.LeftCam_Undistorted =" << GI.Orig.LeftCam_Undistorted->width << "x" << GI.Orig.LeftCam_Undistorted->height << endl;
		cout << "GI.Orig.RightCam_Undistorted =" << GI.Orig.RightCam_Undistorted->width << "x" << GI.Orig.RightCam_Undistorted->height <<endl;
		cout << "RoadImg_original=" << RoadImg_original->width << "x" << RoadImg_original->height <<endl;
		cout << "CamCalibparams.overlapH =" << CamCalibparams.overlapH << endl;
		cout << "__________Join image report END___________" << endl;
	#endif
	
	if(! joinImages( GI.Orig.LeftCam_Undistorted , GI.Orig.RightCam_Undistorted , GI.Orig.RoadImg, CamCalibparams.overlapH, /*128CamCalibparams.bin*/0, CamCalibparams.binOper ))
    	{
	    printf("Could not join images. Aborting\n");
	    exit(0); 
    	}
	
	cvWaitKey(100);
	
	
	cvShowImage("LightsCamera",LightsImg_Original);

	printf("\n Init Complete. Press any key to start...\n");	
	cvWaitKey(0);
	Time.Tick.ProgramStart=cvGetTickCount( );//program starts now
}

/**@brief Initializes TunnelAnalisysStruct.
 * 
 */
void IN_TunnelAnalisysStruct(void)
{
	printf("Initializing TunnelAnalisysStruct....");
	
	
	
	
	TunnelAnalisys.SensorDistances.RightFront=99;
	TunnelAnalisys.SensorDistances.RightBack=99;
	TunnelAnalisys.SensorDistances.FrontRight=99;
	TunnelAnalisys.SensorDistances.FrontLeft=99;
	TunnelAnalisys.SensorDistances.LeftFront=99;
	TunnelAnalisys.SensorDistances.LeftBack=99;
	
	printf("Done\n");
}

/**@brief Initializes SensorValues struct.
 * 
 */
void IN_SensorValuesStruct(void)
{int i;
	printf("Initializing SensorValuesStruct....");
	
	for (i=0;i<10;i++)
	{
		SensorValues.Tunnel[i]=15;
		SensorValues.RightFront[i]=15;
		SensorValues.RightBack[i]=15;
		SensorValues.FrontRight[i]=15;
		SensorValues.FrontLeft[i]=15;
		SensorValues.LeftFront[i]=15;
		SensorValues.LeftBack[i]=15;
	}
	
		SensorValues.Tunnel_Filtered=15;
	printf("Done\n");
}

/**@brief Initializes ParkAnalisysStruct.
 * 
 */
void IN_ParkAnalisysStruct(void)
{
	printf("Initializing ParkAnalisysStruct....");
	
	ParkAnalisys.ManouverNumber=0;
	
	ParkAnalisys.ManouveringTime1=PAMT1;
	ParkAnalisys.ManouveringDir1=PAMD1;
	
	ParkAnalisys.ManouveringTime2=PAMT2;
	ParkAnalisys.ManouveringDir2=PAMD2;
	
	ParkAnalisys.ManouveringTime3=PAMT3;
	ParkAnalisys.ManouveringDir3=PAMD3;
	
	ParkAnalisys.FindObstacle = 0; 
	ParkAnalisys.PixCount = 0;
	ParkAnalisys.ObstaclePosition= 0;
	
	/*ParkAnalisys.ManouveringTime1=1200;
	ParkAnalisys.ManouveringDir1=45;
	
	ParkAnalisys.ManouveringTime2=3200;
	ParkAnalisys.ManouveringDir2=0;
	
	ParkAnalisys.ManouveringTime3=0;
	ParkAnalisys.ManouveringDir3=45;
	*/
	printf("Done\n");
}

/**@brief Read Atlas.cfg file and initialize the configuration variables.
 * 
 */
void IN_ReadConfigVars(void)
{
	switch ( IP_GetConfigurationFile(0/*read cfg file mode (1 not to read)*/) )
	{
		case -1:
		fprintf( stderr, "Could not open configuration file !\n");
		break;
	
		default:
		fprintf( stderr, "Configuration file read!\n");
	
	}
	
	IP_LoadCamCalibParameters();
	
}

/**@brief Creates memory storages for some opencv functions.
 * 
 */
void IN_CreateMemStorages(void)
{
	/*stuff needed for the fill operation*/
	storage1 = cvCreateMemStorage(0);
	storage2 = cvCreateMemStorage(0);
	storage3 = cvCreateMemStorage(0);
	storage4 = cvCreateMemStorage(0);

}

/**@brief Creates opencv sequences.
 * 
 */
void IN_CreateSequences(void)
{
	LightsAnalisys.pts = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),sizeof(CvPoint), storage3 );
	LightsAnalisys.fillpts = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2 ,  sizeof(CvContour) , sizeof(CvPoint) , storage2 );
	LightsAnalisys.origpts= cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2 ,  sizeof(CvContour) , sizeof(CvPoint) , storage4 );
	CrossAnalisys.pts = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),sizeof(CvPoint), storage1 );

}

/**@brief Creates opencv fonts for windows.
 * 
 */

void IN_CreateFont(void)
{
		cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.5/*normal horizontal size*/,0.5/*normal vertical size*/,0/*non italic font*/,1/*thickness*/,4/*line type*/ );
		cvInitFont( &smallfont, CV_FONT_HERSHEY_SIMPLEX, 0.3/*normal horizontal size*/,0.3/*normal vertical size*/,0/*non italic font*/,1/*thickness*/,4/*line type*/ );
		cvInitFont( &hugefont, CV_FONT_HERSHEY_SIMPLEX, 1/*normal horizontal size*/,1/*normal vertical size*/,0/*non italic font*/,3/*thickness*/,4/*line type*/ );
}

/**@brief Shows the Atlas project Logo. 
 * 
 * @param timetoshow how long should the logo be shown. 
 */
void IN_ShowLogo(int timetoshow)
{

	if (timetoshow) //if zero nothing to show
	{
		IplImage *PresentationImg= cvCreateImage(cvSize(1500,800),IPL_DEPTH_8U , 3);
		cvNamedWindow("PresentationDisplay",CV_WINDOW_AUTOSIZE);
		cvMoveWindow( "PresentationDisplay", 0,0);
	
		PresentationImg=cvLoadImage("images/Presentation2.jpg",1);
	
		cvShowImage("PresentationDisplay",PresentationImg);
	
		printf("\n\n\n");
		printf("____________________________________________\n");
		printf("Atlas III, um Robot Movel Orientado para Provas de Conduçao Autonoma.\n");
		printf("____________________________________________\n");
	
		cvWaitKey(timetoshow);
	
	cvReleaseImage( &PresentationImg );
	cvDestroyWindow("PresentationDisplay");
	}
}

/**@brief Initializes Windows system. Needed or else windows behaviour wont be constant.
 * 
 */
void IN_InitHighGui(void)
{
	cvInitSystem( 0, NULL );
}


/**@brief Calls all other IN_ functions in the proper order.
 * 
 * @param timetoshowlogo how long should the logo be shown.
 */
void IN_InitEverything(int timetoshowlogo)
{
	/*««««««««««Init all«««««««««««««*/
	IN_InitHighGui();
	IN_ShowLogo(timetoshowlogo);
	IN_CreateMemStorages();
	IN_CreateSequences();
	IN_CreateFont();
	IN_CamCalibparams();	
	IN_ReadConfigVars();
	
	
	IN_GlobalImages();
	cout << "olaolaolaola" << endl;

	
	IN_XWindowsData(); //initializes Xwindow data
	IN_XWindowsNavegation(); //initializes all Xwindows of navegation
	IN_XWindowsLights();	//initializes all Xwindows of lights
	IN_PlaceXwindowsData(); 	//places Xwindows Data
	IN_PlaceXwindowsNavegation(); 	//places all Xwindows of navegation
	IN_PlaceXwindowsLights();	//places all Xwindows of lights
	IN_BuildColorTable(&Color);
	
	
	
	IN_TimeInit();
	#if USEPIC1COMS or USEPIC2COMS 
		IN_InitComPort(&Handle.ComPort_0,&Handle.ComPort_1);
	#endif 
	
	IN_RobotStatus();
	IN_SquareRegion(&SquareRegion);
	IN_SquareRegion(&ConstSquareRegion);
	IN_Cameras();
	IN_ModeStruct();
	IN_SignalsStateStruct();
	IN_TunnelAnalisysStruct();
	IN_SensorValuesStruct();
	IN_ParkAnalisysStruct();
	IN_OcupationStruct();
	
	IN_ReadLineWidth();
	
	IN_CheckForIPP();
	
		
	AF_ClearBoxStruct(&SquareRegion);
	AF_ClearRobotStatusStruct();
	
	
	IN_PromptToStart();
	
	
	/*-------------------------------*/
}

/**@brief Initializes CamCalibparams in the case of the read cfg file doesn't work.
 * 
 */
void IN_CamCalibparams(void)
{
 int n;   
    
 
 
    CamCalibparams.overlapH = 30; //default is not being joint...
    CamCalibparams.bin = 0; //no thresholding
    CamCalibparams.binOper = cvOr; //Binary openCV OR
    CamCalibparams.msgOn = 1; //By default show messages on window
    CamCalibparams.spMsgOn = 0; //By default does not show a special message on window

    for(n=0;n<8; n++) CamCalibparams.distDataR[n]=50 ; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
    for(n=0;n<8; n++) CamCalibparams.distDataL[n]=50 ; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
    
    	CamCalibparams.distname[0]="fx"; /*fx, fy, cx, cy, k1, k2, k3, k4;*/
	CamCalibparams.distname[1]="fy";
	CamCalibparams.distname[2]="cx";
	CamCalibparams.distname[3]="cy";
	CamCalibparams.distname[4]="k1";
	CamCalibparams.distname[5]="k2";
	CamCalibparams.distname[6]="k3";
	CamCalibparams.distname[7]="k4";
	
	CamCalibparams.mapxL=NULL;
    	CamCalibparams.mapyL=NULL;
    	CamCalibparams.mapxR=NULL;
    	CamCalibparams.mapyR=NULL;
}


void IN_GlobalImages(void)
{

	
	GI.Orig.RightCam = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
	GI.Orig.RightCam_1ch = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);    	
	GI.Orig.RightCam_Undistorted =cvCreateImage( cvSize(GI.Orig.RightCam->width ,GI.Orig.RightCam->height) , GI.Orig.RightCam->depth, 3);
	GI.Orig.LeftCam = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);    	
	GI.Orig.LeftCam_1ch = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);    	
	GI.Orig.LeftCam_Undistorted=cvCreateImage( cvSize(GI.Orig.RightCam->width ,GI.Orig.RightCam->height), GI.Orig.RightCam->depth, 3);
	GI.Orig.pTempLeftImage_ch1 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Orig.pTempLeftImage_ch2 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Orig.pTempLeftImage_ch3 = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	
	
	ImagesParams.NavImgSize.height = GI.Orig.RightCam->height;
	ImagesParams.NavImgSize.width = 2*GI.Orig.RightCam->width-CamCalibparams.overlapH;	
	
	GI.Orig.RoadImg=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U , 3);
	
	
	
	GI.Cross.test3=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_8U);
	GI.Cross.test2=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_8U);
	GI.Cross.test1=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_8U);
	GI.Cross.CurrentSpotsMat=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_32F);
	GI.Cross.InnerSpotsMat=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_32F);
	GI.Cross.TempMat=cvCreateMat(GI.Orig.RoadImg->height,GI.Orig.RoadImg->width,CV_32F);
	GI.Cross.InnerSpotsImg=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Cross.CurrentSpotsImg=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Cross.TempImg=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	
	RoadImg_gray=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U , 1);
	RoadImg_rgb_hough = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,3);
	RoadImg_gray_filled= cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	FindCross_src = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	mask_Img = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	mask_Img1 = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	cross_Img = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	
	GI.Data.Info = cvCreateImage(cvSize(960,200),IPL_DEPTH_8U , 3);
	
	GI.Lights.HSV = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 3);
	GI.Lights.H = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.S = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.V = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.Red_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.Green_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.Yellow_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.S_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.V_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	GI.Lights.SandV_Filter = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U , 1);
	LIGHTSIMAGEROI.x = 0;
	LIGHTSIMAGEROI.y = 0;
	LIGHTSIMAGEROI.width = 319;
	LIGHTSIMAGEROI.height = 230;
	
	
	cout << "Loading Template images." << endl;
	LightsAnalisys.GAU_Template= cvLoadImage( "../imgtemplates/GAU_Template.jpg", 0 );
	LightsAnalisys.YAL_Template= cvLoadImage( "../imgtemplates/YAL_Template.jpg", 0 );
	LightsAnalisys.YAR_Template= cvLoadImage( "../imgtemplates/YAR_Template.jpg", 0 );
	LightsAnalisys.RC_Template= cvLoadImage( "../imgtemplates/RC_Template.jpg", 0 );
	cout << "Images loaded." << endl;
	cout << "olaolaolaola1111" << endl;
	LightsAnalisys.GAU_TemplateMat8U = cvCreateMat(LightsAnalisys.GAU_Template->height ,LightsAnalisys.GAU_Template->width,CV_8U);	
	cout << "olaolaolaola" << endl;
	LightsAnalisys.YAL_TemplateMat8U = cvCreateMat(LightsAnalisys.YAL_Template->height, LightsAnalisys.YAL_Template->width,CV_8U);	
	LightsAnalisys.YAR_TemplateMat8U = cvCreateMat(LightsAnalisys.YAR_Template->height, LightsAnalisys.YAR_Template->width,CV_8U);	
	LightsAnalisys.RC_TemplateMat8U = cvCreateMat(LightsAnalisys.RC_Template->height, LightsAnalisys.RC_Template->width,CV_8U);	
	
	LightsAnalisys.GAU_Results= cvCreateMat(GI.Lights.V_Filter->height -LightsAnalisys.GAU_Template->height+1,GI.Lights.V_Filter->width -LightsAnalisys.GAU_Template->width+1,CV_32F);
	LightsAnalisys.YAL_Results= cvCreateMat(GI.Lights.V_Filter->height -LightsAnalisys.YAL_Template->height+1,GI.Lights.V_Filter->width -LightsAnalisys.YAL_Template->width+1,CV_32F);
	LightsAnalisys.YAR_Results= cvCreateMat(GI.Lights.V_Filter->height -LightsAnalisys.YAR_Template->height+1,GI.Lights.V_Filter->width -LightsAnalisys.YAR_Template->width+1,CV_32F);
	LightsAnalisys.RC_Results= cvCreateMat(GI.Lights.V_Filter->height -LightsAnalisys.RC_Template->height+1,GI.Lights.V_Filter->width -LightsAnalisys.RC_Template->width+1,CV_32F);
	
	cvGetMat(LightsAnalisys.GAU_Template,LightsAnalisys.GAU_TemplateMat8U);
	
	
	#if USECONSTRUCTIONMODE
	GI.Const.HSV=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,3);
	GI.Const.H=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.S=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.V=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.S_Filter=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.V_Filter=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.SandV_Filter=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.Orange=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	GI.Const.Pins=cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	#endif
	
	
	#if USEPARKING
	ParkAnalisys.ParkImg = cvCreateImage(ImagesParams.NavImgSize,IPL_DEPTH_8U ,1);
	
	#endif
}

void IN_CheckForIPP(void)
{
const char* opencv_libraries = 0;
const char* addon_modules =0;


system("export LD_LIBRARY_PATH=/opt/intel/ipp/5.0/ia32/sharedlib:$LD_LIBRARY_PATH");
cvGetModuleInfo(0, &opencv_libraries, &addon_modules);

printf("Opencv: %s \nAdd-on Modules: %s\n", opencv_libraries, addon_modules);



}

void IN_OcupationStruct(void)
{
int step=15;
printf("Initializing OcupationStruct....");

for (int i=0;i<9;i++)
{
	Ocupation.N[i].Area[0] = cvPoint(0,ImagesParams.NavImgSize.height-1-(i*step));
	Ocupation.N[i].Area[1] = cvPoint(0,ImagesParams.NavImgSize.height-step*(i+1));
	Ocupation.N[i].Area[2] = cvPoint(ImagesParams.NavImgSize.width, ImagesParams.NavImgSize.height-step*(i+1));
	Ocupation.N[i].Area[3] = cvPoint(ImagesParams.NavImgSize.width,ImagesParams.NavImgSize.height);
}
for (int i=0;i<9;i++)
{
	Ocupation.N[i].AreaPixelNum = ( Ocupation.N[i].Area[2].x - Ocupation.N[i].Area[0].x )*(Ocupation.N[i].Area[0].y -Ocupation.N[i].Area[1].y);
}
printf("Done.\n");
}


void IN_ReadLineWidth(void)
{

int a= ImagesParams.NavImgSize.height-1 - RobotStatus.Horizon;

cout << "RobotStatus.PreferedHorizon" << RobotStatus.PreferedHorizon << endl;
cout << "ImagesParams.NavImgSize.height" << ImagesParams.NavImgSize.height << endl;
cout << "a= " << a<< endl;
cout << "TLNSize= " << TLNSize<< endl;


Table_LineNumber_read = (int*)malloc(a * sizeof(int));
Table_RoadWidth_read = (int*)malloc(a * sizeof(int));


IP_ReadRoadWidthCFG(0);

// cvWaitKey(0);




}

#endif
