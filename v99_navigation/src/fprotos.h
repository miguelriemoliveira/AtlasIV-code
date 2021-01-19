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
/*FILENAME: header.h********************************************/
/***************************************************************/
/*! @file fprotos.h
* @brief Declares the prototypes for every function used in the program. The coments should have been placed here !!!!! (next time).
*/
#ifndef _FPROTOS_HEADER_
#define _FPROTOS_HEADER_
	
/**********************************/
/*functions prototypes declarations*/
/**********************************/

/******outputfuc.c functions************/
void PrintError(char errornumber); //prints errors information in the shell


/******inputputfunc.c functions************/
void hclp(int argc,char **argv);//handle command line parameters
int IP_GetConfigurationFile(int mode); //load parameters
char IP_CheckParameter(int  argc, char **argv, char *lookFor);
FILE *IP_SkipCommentLines(  FILE *fp);
void IP_LoadCamCalibParameters(void);
int IP_ReadRoadWidthCFG(int mode);
/*******auxiliaryfunc.c********************/
void AF_DestroyImage(IplImage *Img);
void AF_DestroyMatrix(CvMat *Matrix);
void FindPixelsLFM(IplImage *src,int low_line,int top_line, SquareTest *ST);
void AF_ClearBoxStruct(SquareTest *ST);
void AF_ClearRobotStatusStruct();
void AF_ConvertDA(int *TDS, int *TDA);
void AF_LightsControl();
void AF_SpeedControl();
void AF_DirectionControl(SquareTest *ST);
void AF_DirectionControl_1();
void AF_GetLightsImage();
void AF_GetNavImage();
void AF_ModeOutput(unsigned char i);
int isodata(unsigned int *histarray);
void GetHistogram(unsigned int *histarray, IplImage *src);
void AF_ApplyTunnelSensorFilter(void);
void AF_RemoveIsolatedPix(IplImage *src,IplImage *dst,int startline);
void AF_EscapeManouver();
void AF_ClearAllStruct(void);
void IN_ReadLineWidth(void);

/*******preprocessing.c********************/
void NAVPP_NavCamPreProcessing();
void NAVPP_GetFilledArea_a(IplImage *src,IplImage *dst,CvPoint seedpoint);
void NAVPP_GetFilledArea_b(IplImage *src,IplImage *dst,CvPoint seedpoint);
void NAVPP_GetFilledArea_c(IplImage *src,IplImage *dst,CvPoint seedpoint);
void NAVPP_SelectSeed(IplImage *src,int x_in,int y_in,int *x_out,int *y_out,int Horizon);
void NAVPP_SelectSeed_a(IplImage *src,int x_in,int y_in,int *x_out,int *y_out,int Horizon);
void NAVPP_SelectHorizon(IplImage *src);
void NAVPP_SelectHorizon1(IplImage *src);
void NAVPP_SelectHorizon2(IplImage *src);
void NAVPP_SelectHorizon3(IplImage *src);
void NAVPP_SelectHorizon4(IplImage *src);
void NAVPP_SelectHorizon5(IplImage *src);
void NAVPP_SelectHorizon6(IplImage *src);
/*******preprocessing.c********************/
void NAVP_NavCamProcessing();
void NAVP_FindCross(IplImage *src,IplImage *dst,int hor);
void NAVP_FindCross1(IplImage *src,int hor);
void NAVP_FindCross2(IplImage *src,int hor);
void NAVP_FindCross3(IplImage *src,int hor);
void NAVP_FindPixelsLFO(IplImage *src,int low_line,int top_line);
void NAVP_FindPixelsLFM(IplImage *src,int low_line,int top_line);
void NAVP_GetGoToPoint(int *TLN,int *TRW);
void NAVP_GetGoToPoint_1(int *TLN,int *TRW);
void NAVP_GetDAbyBox();
void NAVP_GetDAbyPP();
void NAVP_GetDAbyPP1();
void NAVP_CheckLineSteping(IplImage *src,int *TLN,int *TRW);
void NAVP_CheckLineSteping_1(IplImage *src,int *TLN,int *TRW);
void NAVP_IsolateInnerSpots(IplImage *src, IplImage* pre_dst2);
void NAP_GetSpot(IplImage* src, IplImage* dst, int endline, int endcol);
void NAVP_GetOcupation(IplImage *src);
/*******rs232coms.c********************/
int initrs232(int pic_no);
void SendSpeed(int speed, int comport);
void SendDir(int dir,int comport);
void SendDirAndSpeed(int dir,int speed,int comport);
void closers232(int fd);
void RS_SendOrderToPIC2(int Signal,int Order,int comport);
void RS_SendAllOrdersToPIC2(int comport);
void RS_ReadSensorsFromPIC2(int comport);
/*******graphicfunc.c********************/
void GF_DrawAll(void);
void GF_DrawAll_SandV(void);
void GF_DrawCross(IplImage *src,int line,int column,int lenght, int *color);
void GF_DrawX(IplImage *src,int line,int column,int lenght, int *color);
void GF_DrawBoxPoints(IplImage *src,SquareTest *ST,int *Msearchcolor,int *Osearchcolor);
void GF_DrawGoToPoints(IplImage *src,SquareTest *ST,int *both_tl_color,int *singlecolor,int *both_lr_color);
void GF_DrawArea(IplImage *mask, IplImage *dst,int hor, int *color);
void GF_Join_GoToPoint_Present(IplImage *src,SquareTest *ST,int *Color);
void GF_WriteFlags();
void GF_ShowImages();
/*******init.c********************/
void IN_InitComPort(int *port0,int *port1);
void IN_TimeInit(void);
void IN_XWindowsData(void);
void IN_XWindowsNavegation(void);
void IN_XWindowsLights(void);
void IN_PlaceXwindowsData(void);
void IN_PlaceXwindowsNavegation(void);
void IN_PlaceXwindowsLights(void);
void IN_BuildColorTable(TypeColor *p);
void IN_RobotStatus(void);
void IN_SquareRegion(SquareTest *ST);
void IN_Cameras(void);
void IN_ModeStruct();
void IN_SignalsStateStruct(void);
void IN_TunnelAnalisysStruct(void);
void IN_SensorValuesStruct(void);
void IN_ParkAnalisysStruct(void);
void IN_PromptToStart(void);
void IN_ReadConfigVars(void);
void IN_CreateMemStorages(void);
void IN_CreateSequences(void);
void IN_CreateFont(void);
void IN_InitHighGui(void);
void IN_InitEverything(int timetoshowlogo);
void IN_ShowLogo(int timetoshow);
void IN_CamCalibparams(void);
void IN_GlobalImages(void);
void IN_CheckForIPP(void);
void IN_OcupationStruct(void);
/*******timefunc.c********************/
void TM_UpdateTick(int64 *Tick2Update);
int TM_TimeSince(int64 *Tick);
void TM_UpdateSince();
/*******readlights.c********************/
void RL_ApplyFilter(char FilterIdent);
void RL_ApplyFilter_1(char FilterIdent);
void RL_ColorRecognition();
void RL_ColorRecognition_1();
void RL_ClearLightsAnalisysStruct();
void RL_MergeSandVFilter();
void RL_ShapeRecognition();
void RL_CalculateObjectOrientation();
void RL_GetMassCenter();
void RL_BuildPtsSeqFromImg();
void RL_BuildMinimumAreaRectangle();
void RL_BuildHull();
void RL_COLORandShapeMerge();
void RL_MatchShapes(void);
/*******distantreadlights.c*************/
void DRL_ApplyFilter(char FilterIdent);
void DRL_ColorRecognition();
void DRL_ClearLightsAnalisysStruct();
void DRL_MergeSandVFilter();
void DRL_ShapeRecognition();
void DRL_CalculateObjectOrientation();
void DRL_GetMassCenter();
void DRL_BuildPtsSeqFromImg();
void DRL_BuildMinimumAreaRectangle();
void DRL_BuildHull();
void DRL_COLORandShapeMerge();
/*******tunnelnav.c********************/
void TN_TunnelNavegation(void);
void TN_GetSidesAvg(void);
void TN_GetSideSensorsDistances(void);
/*******visiontunnelnav.c********************/
void VTN_GetTunnelImage(void);
void VTN_FindPixelsLFM(IplImage *src,int low_line,int top_line);
/*******constnav.c********************/
void CN_FindPins(void);
void CN_DrawPins(void);
void CN_DetectPins(void);
void CN_FindPixelsLFO(IplImage *src,int low_line,int top_line);
void CN_GetGoToPoint(int *TLN,int *TRW);
void CN_GetDAbyPP(void);
/*******criteria.c********************/
int CR_IsTunnelUp(void);
int CR_IsTunnelLeft(void);
int CR_IsTunnelRight(void);
int CR_IsNotTunnel(void);
int CR_IsCloseCross(void);
int CR_IsCloseCross_1(void);
int CR_IsStopModeTimeout(void);
int CR_IsYellowArrowLeft(void);
int CR_IsGreenArrowUp(void);
int CR_IsYellowArrowRight(void);
/*******modemanagement.c********************/
void MM_CheckForModeChange();
void MM_CheckForModeChange_1();
/*******crossanalysis.cpp********************/
void NAVP_FindCross4(IplImage *src,int hor);
void NAVP_FindCross5(IplImage *src,int hor);
void SetCrossAnalysisStruct(CvBox2D box);


#endif 
