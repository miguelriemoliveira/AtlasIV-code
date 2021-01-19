/***************************************************************/
/**TITLE: ROBOT navegation input auxiliary functions************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: Provides auxiliary functions to handle input guiven parameters******************************************************/
/***************************************************************/
/*FILENAME: inputfunc.c*****************************************/
/***************************************************************/
/*! @file inputfunc.cpp
* @brief Used to define all the functions that are used to read parameters or files.
*/

#ifndef _INPUTFUNC_
#define _INPUTFUNC_


#include "header.h"

/** @brief Deprecated function used to handle comand line parameters. It only works with the ./startnav -h by returning help.
 * 
 * @param argc argument count.
 * @param argv pointer to the argument value.
 */
void hclp(int argc,char **argv)
{
int n,i;
if (argc>1)
{
	for(n=1; n< argc; n++)
	{
		if (!strcmp(argv[n],"-h"))
		{
		printf("**************************\nstart_nav help:\nSintax: -h [help]\n");
		exit(1);
		}
		else if (!strcmp(argv[n],"-w"))
		{
		n++;
		for (i=0;i<5;i++,n++)
		{
			outputdef.previewwindows[i]=(atoi(argv[n]));
		}
		}
	}
}
}

#define DEFSTR "default string"
#define DEFVAL	314


/*_____________________________________________________________________
*   Name:           GetConfigurationFile
*   Creation:       18 Apr 1993 
*   Last revision:  10 Mar 2005 
*   Returns:        -1 on failure ; 1 on sucess reading
*   Arguments:      int for mode (0=load data; 1=do not load data)
*   Calls:          fopen(), strcpy(), strcat(), feof(),
                    fclose(), sscanf(), fgets(), strcmp(), SkipCommentLines(),
*__________________________________________________________________________*/
/** @brief Reads the ATLAS.cfg file and atributes the values to the variables.
 * 
 * @param mode 
 * @return 
 */
int IP_GetConfigurationFile(int mode)
{
#define MAX_LINE_SIZE 512

    static char *baseFileName="AtlasIV.cfg";
    char line[MAX_LINE_SIZE];
    char var[32]; char value[64];
    int retVal = 1;
    int items=0;
    int nn;
    float nnf;
	

    FILE *fp=fopen( baseFileName, "r");

    if( !fp ) return -1;


    if( mode == 1 )  /*DO not load data. Not very useful for now!*/
    {
        fclose( fp );
        return -1;
    }

    
    do
    {
        IP_SkipCommentLines( fp ); 

        fgets( line, MAX_LINE_SIZE, fp ); 
        if( line[0]=='#' ) continue; /*It's a comment*/

        items=sscanf( line," %32s %64s ", var, value);
        if( items != 2 ) continue;  /*Less than two tokens May be an error ...*/

	
	
	
	nn = atoi(value);
	nnf = atof(value);

        if( ! strcmp( var,"NORMALSPEED") ) NORMALSPEED = nn;
        if( ! strcmp( var,"CROSSAPROACHSPEED") ) CROSSAPROACHSPEED = nn; 
	if( ! strcmp( var,"STOPPEDSPEED") ) STOPPEDSPEED= nn; 
	if( ! strcmp( var,"TUNNELSPEED") ) TUNNELSPEED= nn; 
	if( ! strcmp( var,"CONSTRUCTIONSPEED") ) CONSTRUCTIONSPEED= nn; 
	if( ! strcmp( var,"DANGERSPEED") ) DANGERSPEED= nn; 
	if( ! strcmp( var,"TURBOADDSPEED") ) TURBOADDSPEED= nn; 
	if( ! strcmp( var,"BACKWARDSPEED") ) BACKWARDSPEED= nn; 
	if( ! strcmp( var,"MAXNOATTEMPTSTOSEND") ) MAXNOATTEMPTSTOSEND= nn; 
	if( ! strcmp( var,"TIMETOWAITFORNEXTATEMPT") ) TIMETOWAITFORNEXTATEMPT = nn; 
	if( ! strcmp( var,"THRESHOLD_LIMIT") ) THRESHOLD_LIMIT = nn; 
	if( ! strcmp( var,"HORIZON") ) HORIZON= nn; 
	if( ! strcmp( var,"CONSTHORIZON") ) CONSTHORIZON= nn; 
	if( ! strcmp( var,"MINIMUMLINEWIDTH") ) MINIMUMLINEWIDTH= nn; 
	if( ! strcmp( var,"COLSTOFINDABLACKPIX") ) COLSTOFINDABLACKPIX= nn; 
	if( ! strcmp( var,"LINETOSTARTLOWSEARCH") ) LINETOSTARTLOWSEARCH= nn; 
	if( ! strcmp( var,"LINETOSTARTTOPSEARCH") ) LINETOSTARTTOPSEARCH= nn; 
	if( ! strcmp( var,"ANGLEATSTRAIGHTLEFTLINE") ) ANGLEATSTRAIGHTLEFTLINE= nnf; 
	if( ! strcmp( var,"ANGLEATSTRAIGHTRIGHTLINE") ) ANGLEATSTRAIGHTRIGHTLINE = nnf; 
	if( ! strcmp( var,"ROBOTWIDTHFACTORTOROADWITH") ) ROBOTWIDTHFACTORTOROADWITH = nnf; 
	if( ! strcmp( var,"TIMETOWAITAFTERLASTCROSS") ) TIMETOWAITAFTERLASTCROSS= nn; 
	if( ! strcmp( var,"TIMETOWAITAFTERLASTTUNNEL") ) TIMETOWAITAFTERLASTTUNNEL= nn; 
	if( ! strcmp( var,"TIMETOWAITINSTOPMODE") ) TIMETOWAITINSTOPMODE= nn; 
	if( ! strcmp( var,"SATURATIONLIMIT") ) SATURATIONLIMIT= nn; 
	if( ! strcmp( var,"INTENSITYLIMIT") ) INTENSITYLIMIT= nn; 
	if( ! strcmp( var,"REDFILTERUPPERLIMIT") ) REDFILTERUPPERLIMIT= nn; 
	if( ! strcmp( var,"REDFILTERLOWERLIMIT") ) REDFILTERLOWERLIMIT= nn; 
	if( ! strcmp( var,"GREENFILTERUPPERLIMIT") ) GREENFILTERUPPERLIMIT= nn; 
	if( ! strcmp( var,"GREENFILTERLOWERLIMIT") ) GREENFILTERLOWERLIMIT= nn; 
	if( ! strcmp( var,"YELLOWFILTERUPPERLIMIT") ) YELLOWFILTERUPPERLIMIT= nn; 
	if( ! strcmp( var,"YELLOWFILTERLOWERLIMIT") ) YELLOWFILTERLOWERLIMIT= nn; 
	if( ! strcmp( var,"FRACTIONLIMITTODECIDEYELLOW") ) FRACTIONLIMITTODECIDEYELLOW = nnf; 
	if( ! strcmp( var,"MINIMUMCOUNTVALUE") ) MINIMUMCOUNTVALUE= nn; 
	if( ! strcmp( var,"MAXIMUMOCUPATIONPERCENT") ) MAXIMUMOCUPATIONPERCENT= nnf; 
	if( ! strcmp( var,"OBJECTUPMAXANGLE") ) OBJECTUPMAXANGLE= nn; 
	if( ! strcmp( var,"OBJECTUPMINANGLE") ) OBJECTUPMINANGLE= nn; 
	if( ! strcmp( var,"OBJECTLEFTMAXANGLE") ) OBJECTLEFTMAXANGLE= nn; 
	if( ! strcmp( var,"OBJECTLEFTMINANGLE") ) OBJECTLEFTMINANGLE= nn; 
	if( ! strcmp( var,"OBJECTRIGHTMAXANGLE") ) OBJECTRIGHTMAXANGLE= nn; 
	if( ! strcmp( var,"OBJECTRIGHTMINANGLE") ) OBJECTRIGHTMINANGLE= nn; 
	if( ! strcmp( var,"MINIMUMCROSSSIZE") ) MINIMUMCROSSSIZE= nn; 	
	if( ! strcmp( var,"TIMETOSTARTLOOKINGFORCROSS") ) TIMETOSTARTLOOKINGFORCROSS= nn; 	
	if( ! strcmp( var,"TIMESTOANALISESPOTFORCROSS") ) TIMESTOANALISESPOTFORCROSS= nn; 	
	if( ! strcmp( var,"MAXDISTTOBETUNNEL") ) MAXDISTTOBETUNNEL= nn; 
	if( ! strcmp( var,"PAMT1") ) PAMT1= nn; 
	if( ! strcmp( var,"PAMD1") ) PAMD1= nn; 
	if( ! strcmp( var,"PAMT2") ) PAMT2= nn; 
	if( ! strcmp( var,"PAMD2") ) PAMD2= nn; 
	if( ! strcmp( var,"PAMT3") ) PAMT3= nn; 
	if( ! strcmp( var,"PAMD3") ) PAMD3= nn; 
	if( ! strcmp( var,"PAMT2A") ) PAMT2A= nn; 
	if( ! strcmp( var,"PAMD2A") ) PAMD2A= nn; 
	if( ! strcmp( var,"PAMT3A") ) PAMT3A= nn; 
	if( ! strcmp( var,"PAMD3A") ) PAMD3A= nn; 
	if( ! strcmp( var,"TIMETOHOLDINESCAPEMANOUVER") ) TIMETOHOLDINESCAPEMANOUVER= nn; 
	if( ! strcmp( var,"MINANGLETOENTERESCAPEMANOUVER") ) MINANGLETOENTERESCAPEMANOUVER= nn; 
	if( ! strcmp( var,"MAXANGLETOENTERESCAPEMANOUVER") ) MAXANGLETOENTERESCAPEMANOUVER= nn;
	if( ! strcmp( var,"PIXELSTOGOTOCONSTMODE") ) PIXELSTOGOTOCONSTMODE= nn; 
	if( ! strcmp( var,"PIXELSTOEXITCONSTMODE") ) PIXELSTOEXITCONSTMODE= nn; 
	if( ! strcmp( var,"CONSTSATURATIONLIMIT") ) CONSTSATURATIONLIMIT= nn; 
	if( ! strcmp( var,"CONSTINTENSITYLIMIT") ) CONSTINTENSITYLIMIT= nn; 
	if( ! strcmp( var,"CONSTLOWORANGELIMIT") ) CONSTLOWORANGELIMIT= nn; 
	if( ! strcmp( var,"CONSTTOPORANGELIMIT") ) CONSTTOPORANGELIMIT= nn; 
	
	
	
    } while( ! feof(fp) );

	
	printf("Reading Speed Variables..............\n");
	printf("NORMALSPEED = %d\n", NORMALSPEED);
	printf("CROSSAPROACHSPEED = %d\n", CROSSAPROACHSPEED);
	printf("STOPPEDSPEED = %d\n", STOPPEDSPEED);
	printf("TUNNELSPEED= %d\n", TUNNELSPEED);
	printf("CONSTRUCTIONSPEED= %d\n", CONSTRUCTIONSPEED);
	printf("BACKWARDSPEED= %d\n", BACKWARDSPEED);
	printf("DANGERSPEED= %d\n", DANGERSPEED);
	printf("TURBOADDSPEED= %d\n", TURBOADDSPEED);
	printf("done.\n");
	
	printf("Reading Comunnication Parameters.......\n");
	printf("MAXNOATTEMPTSTOSEND = %d\n", MAXNOATTEMPTSTOSEND);
	printf("TIMETOWAITFORNEXTATEMPT = %d\n", TIMETOWAITFORNEXTATEMPT);
	printf("done.\n");
	
	printf("Reading Calculus Assumptions .......\n");
	printf("THRESHOLD_LIMIT = %d\n", THRESHOLD_LIMIT);
	printf("HORIZON= %d\n", HORIZON);
	printf("CONSTHORIZON= %d\n", CONSTHORIZON);
	printf("MINIMUMLINEWIDTH= %d\n", MINIMUMLINEWIDTH);
	printf("COLSTOFINDABLACKPIX= %d\n", COLSTOFINDABLACKPIX);
	printf("LINETOSTARTLOWSEARCH= %d\n", LINETOSTARTLOWSEARCH);
	printf("LINETOSTARTTOPSEARCH= %d\n", LINETOSTARTTOPSEARCH);
	printf("done.\n");
	
	printf("Reading Calibration Variables .......\n");
	printf("ANGLEATSTRAIGHTLEFTLINE= %g\n", ANGLEATSTRAIGHTLEFTLINE);
	printf("ANGLEATSTRAIGHTRIGHTLINE = %g\n", ANGLEATSTRAIGHTRIGHTLINE );
	printf("ROBOTWIDTHFACTORTOROADWITH = %g\n", ROBOTWIDTHFACTORTOROADWITH );
	printf("done.\n");
	
	printf("Reading Timeout Variables .......\n");
	printf("TIMETOWAITAFTERLASTCROSS= %d\n", TIMETOWAITAFTERLASTCROSS);
	printf("TIMETOWAITAFTERLASTTUNNEL= %d\n", TIMETOWAITAFTERLASTTUNNEL);
	printf("TIMETOWAITINSTOPMODE= %d\n", TIMETOWAITINSTOPMODE);
	printf("done.\n");
	
	printf("Reading Lights Analisys Variables .......\n");
	printf("SATURATIONLIMIT= %d\n", SATURATIONLIMIT);
	printf("INTENSITYLIMIT= %d\n", INTENSITYLIMIT);
	printf("REDFILTERUPPERLIMIT= %d\n", REDFILTERUPPERLIMIT);
	printf("REDFILTERLOWERLIMIT= %d\n", REDFILTERLOWERLIMIT);
	printf("GREENFILTERUPPERLIMIT= %d\n", GREENFILTERUPPERLIMIT);
	printf("GREENFILTERLOWERLIMIT= %d\n", GREENFILTERLOWERLIMIT);
	printf("YELLOWFILTERUPPERLIMIT= %d\n", YELLOWFILTERUPPERLIMIT);
	printf("YELLOWFILTERLOWERLIMIT= %d\n", YELLOWFILTERLOWERLIMIT);
	printf("FRACTIONLIMITTODECIDEYELLOW = %g\n", FRACTIONLIMITTODECIDEYELLOW );
	printf("MINIMUMCOUNTVALUE= %d\n", MINIMUMCOUNTVALUE);
	printf("MAXIMUMOCUPATIONPERCENT= %g\n", MAXIMUMOCUPATIONPERCENT);
	printf("OBJECTUPMAXANGLE= %d\n", OBJECTUPMAXANGLE);
	printf("OBJECTLEFTMAXANGLE= %d\n", OBJECTLEFTMAXANGLE);
	printf("OBJECTLEFTMINANGLE= %d\n", OBJECTLEFTMINANGLE);
	printf("OBJECTRIGHTMAXANGLE= %d\n", OBJECTRIGHTMAXANGLE);
	printf("OBJECTRIGHTMINANGLE= %d\n", OBJECTRIGHTMINANGLE);
	printf("OBJECTRIGHTMINANGLE= %d\n", OBJECTRIGHTMINANGLE);
	printf("done.\n");

	printf("Reading Cross Analisys Variables .......\n");
	printf("MINIMUMCROSSSIZE= %d\n", MINIMUMCROSSSIZE);
	printf("TIMETOSTARTLOOKINGFORCROSS= %d\n", TIMETOSTARTLOOKINGFORCROSS);
	printf("TIMESTOANALISESPOTFORCROSS= %d\n", TIMESTOANALISESPOTFORCROSS);
	printf("done.\n");
	
	printf("Reading Tunnel Analisys Variables .......\n");
	printf("MAXDISTTOBETUNNEL= %d\n", MAXDISTTOBETUNNEL);
	printf("done.\n");
	
	printf("Reading BACKWARD Analisys Variables .......\n");
	printf("CONSTSATURATIONLIMIT= %d\n", CONSTSATURATIONLIMIT);
	printf("CONSTINTENSITYLIMIT= %d\n", CONSTINTENSITYLIMIT);
	printf("CONSTLOWORANGELIMIT= %d\n", CONSTLOWORANGELIMIT);
	printf("CONSTTOPORANGELIMIT= %d\n", CONSTTOPORANGELIMIT);
	printf("PIXELSTOGOTOCONSTMODE= %d\n", PIXELSTOGOTOCONSTMODE);
	printf("PIXELSTOEXITCONSTMODE= %d\n", PIXELSTOEXITCONSTMODE);
	printf("done.\n");
	
	printf("Reading Parking Variables .......\n");
	printf("PAMT1= %d\n", PAMT1);
	printf("PAMD1= %d\n", PAMD1);
	printf("PAMT2= %d\n", PAMT2);
	printf("PAMD2= %d\n", PAMD2);
	printf("PAMT3= %d\n", PAMT3);
	printf("PAMD3= %d\n", PAMD3);
	printf("PAMT2A= %d\n", PAMT2A);
	printf("PAMD2A= %d\n", PAMD2A);
	printf("PAMT3A= %d\n", PAMT3A);
	printf("PAMD3A= %d\n", PAMD3A);
	printf("done.\n");
	
	printf("Reading Escape Manouver Variables .......\n");
	printf("TIMETOHOLDINESCAPEMANOUVER= %d\n", TIMETOHOLDINESCAPEMANOUVER);
	printf("MINANGLETOENTERESCAPEMANOUVER= %d\n", MINANGLETOENTERESCAPEMANOUVER);
	printf("MAXANGLETOENTERESCAPEMANOUVER= %d\n", MAXANGLETOENTERESCAPEMANOUVER);
	printf("done.\n");
	
    fclose( fp );
    return retVal;
}

/*_____________________________________________________________________
*   Name:           CheckParameter
*   Creation:       12 Feb 1993
*   Last revision:  12 Feb 1993
*   Returns:        Boolean expressing the existence of passed parameter
*   Arguments:      The main arguments plus the argument to locate.
*   Notes:
*   Calls:          strlen(), strncmp()
*__________________________________________________________________________*/
/** @brief Used by the IP_GetConfigurationFile.
 * 
 * @param argc 
 * @param argv 
 * @param lookFor 
 * @return 
 */
char IP_CheckParameter(int  argc, char **argv, char *lookFor)
{
    char retVal=0;
    int count = 1,
        length= strlen( lookFor );
 
    while( count < argc )
    {
        if( strncmp( lookFor, argv[count], length) == 0 )
        {
            retVal = 1;
            break;
        }
        count ++;
    }
    return retVal;
}

/*_____________________________________________________________________
*   Name:           SkipCommentLines
*   Creation:       01 Mar 1993
*   Last revision:  24 Feb 1994
*   Returns:        file pointer duly positioned
*   Arguments:      file pointer
*   Notes:          positions file pointer on next first non-comment line of file
*                   Files with lines longer than 1024 chars may not work ....
*                   Comments are lines starting with a '#' ...
*   Calls:          ftell(), fgets(), feof(), [fscanf()], fseek(),
*__________________________________________________________________________*/
/** @brief Used to skip the commented lines.
 * 
 * @param fp 
 * @return 
 */
FILE *IP_SkipCommentLines(  FILE *fp)
{
    char dummyLine[1024];
    long pos;
 
    pos = ftell( fp );
    fgets( dummyLine, 1023, fp ); /*Read line until \n or EOF */
 
    while ( dummyLine[0] == '#' )
    {
        if ( feof(fp) ) break;  /*Interrupt cycle */
 
        pos = ftell( fp ); /*save file pointer before next line read ...*/
        fgets( dummyLine, 1023, fp ); /*Read line until \n or EOF */
    }
 
    fseek( fp, pos, 0 /*SEEK_SET*/ );
    return fp;
 
}



/*!
 * @brief Loads Camera distortion parameters from a fixed file
 * @brief Uses global variables :-( Not recommended
 */
void IP_LoadCamCalibParameters(void)
{
	FILE *fp = fopen("camParams.cfg", "r");
	int n;
	static char str[255];
	if( !fp) {cout << "WARNING: Could not read camParams.cfg" << endl; return;};
	
	for(n=0; n < 5; n++) fgets(str, 127, fp); //Read comment lines
	
	for(n=0; n < 8; n++)
		fscanf(fp, "%s	%d\n", str, &CamCalibparams.distDataR[n]);
	for(n=0; n < 8; n++)
		fscanf(fp, "%s	%d\n", str, &CamCalibparams.distDataL[n]);

	fscanf(fp, "%s	%d\n", str, &CamCalibparams.overlapH);
	fscanf(fp, "%s	%d\n", str, &CamCalibparams.bin);

	
	#if POUT
		cout << "__________Reading camParams.cfg__________" << endl;
		for(n=0; n < 8; n++)
		{
			fscanf(fp, "%s	%d\n", str, &CamCalibparams.distDataR[n]);
			cout << "reading right" <<  CamCalibparams.distname[n] << "=" << CamCalibparams.distDataR[n] << endl;
		}
		
		
		for(n=0; n < 8; n++)
		{
			fscanf(fp, "%s	%d\n", str, &CamCalibparams.distDataL[n]);
			cout << "reading left" <<  CamCalibparams.distname[n] << "=" << CamCalibparams.distDataL[n] << endl;
		}
		
		cout << "reading overlapH = " << CamCalibparams.overlapH << endl;
		cout << "reading bin= " << CamCalibparams.bin << endl;
		cout << "________Reading camParams.cfg  DONE_______" << endl;
	#endif

	fclose(fp);

}

int IP_ReadRoadWidthCFG(int mode)
{
#define MAX_LINE_SIZE 512

    static char *baseFileName="RoadWidth.cfg";
    char line[MAX_LINE_SIZE];
    char var[32]; char value[64];
    int retVal = 1;
    int items=0;
    int nn;
    float nnf;
	

    FILE *fp=fopen( baseFileName, "r");

    if( !fp ) return -1;


    if( mode == 1 )  /*DO not load data. Not very useful for now!*/
    {
        fclose( fp );
        return -1;
    }

    int count = RobotStatus.Horizon+1;
    char tmpstr[255];
    printf("Reading Line width..............\n");
    do
    {
        IP_SkipCommentLines( fp ); 

        fgets( line, MAX_LINE_SIZE, fp ); 
        if( line[0]=='#' ) continue; /*It's a comment*/

        items=sscanf( line," %32s %64s ", var, value);
        if( items != 2 ) continue;  /*Less than two tokens May be an error ...*/

	nn = atoi(value);
	nnf = atof(value);
	
	
	
	sprintf(tmpstr, "line%d",count);
        if( ! strcmp( var,tmpstr) ) 
	{
	Table_RoadWidth_read[count-(RobotStatus.Horizon+1)] = nn;
        Table_LineNumber_read[count-(RobotStatus.Horizon+1)] = count;
	
	
	printf("var=%s  %d  %d \n",tmpstr,Table_LineNumber_read[count-(RobotStatus.Horizon+1)], Table_RoadWidth_read[count-(RobotStatus.Horizon+1)]);
	}
	
	count++;
	
    } while( ! feof(fp) );

	printf("done.\n");
	
	
	
    fclose( fp );
    return retVal;
}


#endif
