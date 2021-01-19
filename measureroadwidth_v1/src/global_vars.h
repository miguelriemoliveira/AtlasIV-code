/*! @file global_vars.h
* @brief Global variables are here defined. Some are Still defined in header.h .
*/

#ifdef _MAIN_FILE_

/*_______ Variables to be read in cfg file _________*/
int NORMALSPEED;	//Speed when without constraints
int CROSSAPROACHSPEED;	//Speed when cross is in view
int STOPPEDSPEED;		//Speed when stopped
int TUNNELSPEED;		//Speed when in the tunnel
int CONSTRUCTIONSPEED;		//Speed when in the Contruction
int MAXNOATTEMPTSTOSEND;	//maximum number of atempts
int TIMETOWAITFORNEXTATEMPT;	//time to wait to make a new atempt (milisecs)
int THRESHOLD_LIMIT;	//threshold limit [0 to=255]
int HORIZON;	///Horizon definition
int CONSTHORIZON;	///CONST. Horizon definition
int MINIMUMLINEWIDTH;	//Used by AF_SelectHorizon2
int COLSTOFINDABLACKPIX;	//Used by FindPixelsLFO (check before changing)
int LINETOSTARTLOWSEARCH;	//Used by FindPixelsLFO (check before changing)
int LINETOSTARTTOPSEARCH;	//Used by FindPixelsLFO. Adds Horizon automaticaly (check before changing)
float ANGLEATSTRAIGHTLEFTLINE;	//used by GetGoToPoint in auxiliaryfunc.c
float  ANGLEATSTRAIGHTRIGHTLINE;	//used by GetGoToPoint in auxiliaryfunc.c
float  ROBOTWIDTHFACTORTOROADWITH;	//Used by LineSteping
int TIMETOWAITAFTERLASTCROSS;//time to wait on CrossAproachspeed until switching to normal
int TIMETOWAITAFTERLASTTUNNEL; //time to wait on tunnel mode before changing to normal
int TIMETOWAITINSTOPMODE;	//time to wait on stop mode trying to get a lights analisys correct
unsigned int SATURATIONLIMIT;
unsigned int INTENSITYLIMIT;
unsigned int REDFILTERUPPERLIMIT;
unsigned int REDFILTERLOWERLIMIT;
unsigned int GREENFILTERUPPERLIMIT;
unsigned int GREENFILTERLOWERLIMIT;
unsigned int YELLOWFILTERUPPERLIMIT;
unsigned int YELLOWFILTERLOWERLIMIT;
float  FRACTIONLIMITTODECIDEYELLOW;
unsigned int MINIMUMCOUNTVALUE;
float MAXIMUMOCUPATIONPERCENT;
int OBJECTUPMAXANGLE;
int OBJECTUPMINANGLE;
int OBJECTLEFTMAXANGLE;
int OBJECTLEFTMINANGLE;
int OBJECTRIGHTMAXANGLE;
int OBJECTRIGHTMINANGLE;
int MINIMUMCROSSSIZE;  //Minimum cross possible size
int TIMETOSTARTLOOKINGFORCROSS;
int TIMESTOANALISESPOTFORCROSS;
int MAXDISTTOBETUNNEL;//maximum distance to be a tunnel (14 corresponds to 60 cm (excel table))
unsigned int CONSTSATLIMIT;
unsigned int PIXELSTOGOTOCONSTMODE;
int PAMT1;
int PAMD1;
int PAMT2;
int PAMD2;
int PAMT3;
int PAMD3;


//_____________________________________vs Structures_____________________________________________________		

TypeCamCalibparams CamCalibparams;
		  


/*____________________________________________________*/


CvMemStorage *storage1;
CvMemStorage *storage2;
CvMemStorage *storage3;









#else

/*_______ Parameters  to be read in cfg file _________*/
extern int NORMALSPEED;
extern int CROSSAPROACHSPEED;
extern int STOPPEDSPEED;
extern int TUNNELSPEED;
extern int CONSTRUCTIONSPEED;
extern int MAXNOATTEMPTSTOSEND;
extern int TIMETOWAITFORNEXTATEMPT;
extern int THRESHOLD_LIMIT;
extern int HORIZON;
extern int CONSTHORIZON;
extern int MINIMUMLINEWIDTH;
extern int COLSTOFINDABLACKPIX;
extern int LINETOSTARTLOWSEARCH;
extern int LINETOSTARTTOPSEARCH;
extern float ANGLEATSTRAIGHTLEFTLINE;
extern float  ANGLEATSTRAIGHTRIGHTLINE;
extern float  ROBOTWIDTHFACTORTOROADWITH;
extern int TIMETOWAITAFTERLASTCROSS;
extern int TIMETOWAITAFTERLASTTUNNEL;
extern int TIMETOWAITINSTOPMODE;
extern int TIMETOWAITAFTERLASTCROSS;
extern unsigned int SATURATIONLIMIT;
extern unsigned int INTENSITYLIMIT;
extern unsigned int REDFILTERUPPERLIMIT;
extern unsigned int REDFILTERLOWERLIMIT;
extern unsigned int GREENFILTERUPPERLIMIT;
extern unsigned int GREENFILTERLOWERLIMIT;
extern unsigned int YELLOWFILTERUPPERLIMIT;
extern unsigned int YELLOWFILTERLOWERLIMIT;
extern float  FRACTIONLIMITTODECIDEYELLOW;
extern unsigned int MINIMUMCOUNTVALUE;
extern float MAXIMUMOCUPATIONPERCENT;
extern int OBJECTUPMAXANGLE;
extern int OBJECTUPMINANGLE;
extern int OBJECTLEFTMAXANGLE;
extern int OBJECTLEFTMINANGLE;
extern int OBJECTRIGHTMAXANGLE;
extern int OBJECTRIGHTMINANGLE;
extern int MINIMUMCROSSSIZE;
extern int TIMETOSTARTLOOKINGFORCROSS;
extern int TIMESTOANALISESPOTFORCROSS;
extern int MAXDISTTOBETUNNEL;
extern unsigned int CONSTSATLIMIT;
extern unsigned int PIXELSTOGOTOCONSTMODE;
extern int PAMT1;
extern int PAMD1;
extern int PAMT2;
extern int PAMD2;
extern int PAMT3;
extern int PAMD3;

/*____________________________________________________*/

extern CvMemStorage *storage1;
extern CvMemStorage *storage2;
extern CvMemStorage *storage3;


//_____________________________________vs Structures_________________

extern TypeCamCalibparams CamCalibparams;
/*____________________________________________________*/
#endif

