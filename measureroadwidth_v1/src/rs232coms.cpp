/***************************************************************/
/**TITLE: ROBOT navegation comunications file*******************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: To control ROBOT navegation in the road designed for 
the Robotica 2005 Festival*************************************/
/***************************************************************/
/*FILENAME: rs232coms.c********************************************/
/***************************************************************/
/*! @file rs232coms.cpp
* @brief All the functions used for serial port comunication are defined here.
* 
* Two pics are used to control the robot. Pic nº1 is used to control the motors and Pic nº2 is used to aquire sensors values and to control the lights.
* The lights activation protocol is the following:
* @image html "../images/ProtocoloActivacaoLuzes.jpg" "Robot's Lights Control Protocol"
* As for the sensors indexation, it is the following:
* @image html "../images/TabelaValoresSensores.jpg" "Robot's Sensor's Indexation"
*/

#ifndef _RS232COMS_
#define _RS232COMS_

#include "header.h"


/** @brief This function initializes a comunnication port. It returns the handle to that object
 * 
 * @param pic_no Needs the pic number so it knows which com port's handle it is suposed to return.
 * @return handle to the comport.
 */
int initrs232(int pic_no)
{
struct termios ComParams, newParams;
int ret,fd;
char *device=0;

if (pic_no==1)
	device = COM_DEVICE_PIC1;
else if (pic_no==2)
	device = COM_DEVICE_PIC2;
	
	
bzero( &newParams, sizeof(newParams)); //seting all structure values to zero

	fd = open( device, O_RDWR | O_NONBLOCK ); //open com device. read only and non blocking modes
	if(fd == -1) //if could not open
	{
		perror("Failed on opening Port");
		exit(1);
	}
	ret=tcgetattr(fd, &ComParams); // get serial communuication parameters
	if( ret < 0 ) perror("tcgetattr failed:");

	newParams.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        newParams.c_iflag = IGNPAR;
        newParams.c_oflag = 0;

	ret=tcsetattr( fd, TCSANOW, &newParams ); // set serial communuication parameters
	if( ret < 0 ) perror("tcsetattr failed:");
	return(fd);
}


/**@brief Sends the speed order to the pic. This function is deprecated because now we use SendDirAndSpeed.
 * 
 * @param speed speed value to send.
 * @param comport comport to which the speed should be sent.
 */
void SendSpeed(int speed, int comport)
{
unsigned char msg;
unsigned char Identificador,Quant;
int n,i;


	Quant=(char) speed;
	Identificador=1;
	Identificador=Identificador<<7;
	
	msg=(Identificador | Quant);
		
	for(i=0;i<10;i++)
	{
		n = write( comport, &msg , sizeof(msg) );
		if( n < 0)
		{
			#if POUT
				perror("Speed to PIC1: Failed\n");
			#endif
			cvWaitKey(TIMETOWAITFORNEXTATEMPT);
		}
		else
		{
			#if POUT
				printf("Speed to PIC1: Sent\n");
			#endif
			break;
		}	}
}


/**@brief Sends the dir order to the pic. This function is deprecated because now we use SendDirAndSpeed.
 * 
 * @param dir dir value to send.
 * @param comport comport to which the speed should be sent.
 */
void SendDir(int dir,int comport)
{

unsigned char msg;
unsigned char Identificador,Quant=16;
int n;
//esta tabela deve estar em conformidade com o programa c do pic
int table[32]={35,40,45,50,55,60,66,70,76,78,80,82,84,86,88,90,90,92,94,96,98,100,102,104,110,115,120,125,130,135,140,145}; 
int diff=999;
int i;	
	for (i=0;i<32;i++)
	{
		if (diff>=abs(table[i]-dir)) 
		{
			Quant=i;
			diff=abs(table[i]-dir);
		}
		else break;
	}
	
	
	Identificador=0 ; 

	msg=(Identificador | dir);
	
	for(i=0;i<10;i++)
	{
		n = write( comport, &msg , sizeof(msg) );
		if( n < 0)
		{
			#if POUT
				perror("Direction to PIC1: Failed\n");
			#endif
			cvWaitKey(TIMETOWAITFORNEXTATEMPT);
		}
		else
		{
			#if POUT
				printf("Direction to PIC1: Sent\n");
			#endif
			break;
		}
	}
			

}

/** @brief Sends both the speed and the dir order to the pic.
 * @param dir dir value to send.
 * @param speed speed value to send.
 * @param comport comport to which the speed and dir should be sent.
 */
void SendDirAndSpeed(int dir,int speed,int comport)
{
unsigned char msg[2];
unsigned char Identificador,Quant=16;
int n;
unsigned char msgsent=0;
int table[32]={35,40,45,50,55,60,66,70,76,78,80,82,84,86,88,90,90,92,94,96,98,100,102,104,110,115,120,125,130,135,140,145}; //esta tabela deve estar em conformidade com o programa c do pic
int diff=999;
int i;	
	for (i=0;i<32;i++)
	{
		if (diff>=abs(table[i]-dir)) 
		{
			Quant=i;
			diff=abs(table[i]-dir);
		}
		else break;
	}
	
	
	Identificador=0 ; 

	msg[0]=(Identificador | dir);
	
	
	
	Quant=(char) speed;
	Identificador=1;
	Identificador=Identificador<<7;
	
	msg[1]=(Identificador | Quant);
	
	
	
	for(i=0;i<MAXNOATTEMPTSTOSEND;i++) //now to send both values
	{
		n = write( comport, &msg , sizeof(msg) );
		if( n < 0)
		{
			#if NPOUT
				perror("Direction and Speed to PIC1: Failed\n");
			#endif
			cvWaitKey(TIMETOWAITFORNEXTATEMPT);
		}
		else
		{
			#if NPOUT
				printf("Direction and Speed to PIC1: Sent\n");
			#endif
			msgsent=1;
		}
		
		#if IPAINT
			cvLine(GI.Data.Info,cvPoint(648,15),cvPoint(648,200),CV_RGB(0,180,255),1,8);
			cvPutText(GI.Data.Info, "___Coms___", cvPoint(650,15),&font, CV_RGB(0,180,255));
			if (n>0)
				cvPutText(GI.Data.Info, "PIC1: Sent", cvPoint(650,30),&font, CV_RGB(0,255,0));
			else if(i==MAXNOATTEMPTSTOSEND-1)
				cvPutText(GI.Data.Info, "PIC1: Error", cvPoint(650,30),&font, CV_RGB(255,0,0));
		#endif
		
		if (msgsent)
			break;
	}
			

}


/** @brief Closes the comport of handle fd.
 * 
 * @param fd handle to the comport to be closed.
 */
void closers232(int fd)
 {close(fd);
 } 
 
/** @brief Sends an order to the sensors pic number 2.
 * @param Signal 
 * @param Order 
 * @param comport 
 */
void RS_SendOrderToPIC2(int Signal,int Order,int comport)
{

unsigned char msg;
unsigned char Identificador;
int n,i;
	
/* Protocolo de comunicaçao PC->PIC2
|------------------|---------------|------------|
|Item              | Identificador | OFF/ON     |
|------------------|---------------|------------|
|pisca esquerdo    | 0b00000000  0 | 0b00000000 |
|                  |               | 0b00000001 |
|------------------|---------------|------------|
|pisca direito     | 0b00010000  16| 0b00010000 |
|                  |               | 0b00010001 |
|------------------|---------------|------------|
|luz frontal       | 0b00100000  32| 0b00100000 |
|                  |               | 0b00100001 |
|------------------|---------------|------------|
|luz traseira      | 0b00110000  48| 0b00110000 |
|                  |               | 0b00110001 |
|------------------|---------------|------------|
|envio de sensores | 0b01000000  64| -          |
|------------------|---------------|------------|*/

switch (Signal)
{
	case (ORDERLEFTTURN):
	{
		Identificador=0; 
		msg=(Identificador | Order);
	break;
	}
	case (ORDERRIGHTTURN):
	{
		Identificador=16; 
		msg=(Identificador | Order);
	break;
	}
	case (ORDERHEADLIGHTS):
	{
		Identificador=32; 
		msg=(Identificador | Order);
	break;
	}
	case (ORDERTAILLIGHTS):
	{
		Identificador=48; 
		msg=(Identificador | Order);
	break;
	}
	case (ORDERSENDSENSORS):
	{
		Identificador=64; 
		msg=(Identificador);
	break;
	}
		
}

	for(i=0;i<MAXNOATTEMPTSTOSEND;i++)
	{
		n = write( comport, &msg , sizeof(msg) );
		if( n < 0)
		{
			#if POUT
				perror("Order to PIC2: Failed\n");
			#endif
			cvWaitKey(TIMETOWAITFORNEXTATEMPT);
		}
		else
		{
			#if POUT
				printf("Order to PIC2: Sent\n");
			#endif
			break;
		}
	}
} 

/** @brief Sends lights activation orders to pic number 2.
 * 
 * @param comport 
 */
void RS_SendAllOrdersToPIC2(int comport)
{

unsigned char msg[5];
unsigned char Identificador;
int n=0,i;

	
/* Protocolo de comunicaçao PC->PIC2
|------------------|---------------|------------|
|Item              | Identificador | OFF/ON     |
|------------------|---------------|------------|
|pisca esquerdo    | 0b00000000  0 | 0b00000000 |
|                  |               | 0b00000001 |
|------------------|---------------|------------|
|pisca direito     | 0b00010000  16| 0b00010000 |
|                  |               | 0b00010001 |
|------------------|---------------|------------|
|luz frontal       | 0b00100000  32| 0b00100000 |
|                  |               | 0b00100001 |
|------------------|---------------|------------|
|luz traseira      | 0b00110000  48| 0b00110000 |
|                  |               | 0b00110001 |
|------------------|---------------|------------|
|envio de sensores | 0b01000000  64| -          |
|------------------|---------------|------------|*/



		Identificador=0; 
		msg[0]=(Identificador | SignalsState.TurnLeft);

	
		Identificador=16; 
		msg[1]=(Identificador | SignalsState.TurnRight);
	
		Identificador=32; 
		msg[2]=(Identificador | SignalsState.HeadLights);
	
		Identificador=48; 
		msg[3]=(Identificador | SignalsState.TailLights);
	
		Identificador=64; 
		msg[4]=(Identificador);
	
		


	for(i=0;i<MAXNOATTEMPTSTOSEND;i++)
	{
		n = write( comport, &msg , sizeof(msg) );
		if( n < 0)
		{
			#if NPOUT
				perror("All Orders to PIC2: Failed\n");
			#endif
			cvWaitKey(TIMETOWAITFORNEXTATEMPT);
		}
		else
		{
			#if NPOUT
				printf("All Orders to PIC2: Sent\n");
			#endif
			break;
		}
	}
	
#if IPAINT
char str[255];

	cvLine(GI.Data.Info,cvPoint(348,15),cvPoint(348,200),CV_RGB(0,200,0),1,8);
	cvPutText(GI.Data.Info, "__LIGTHS__", cvPoint(350,15),&font, CV_RGB(0,200,0));
	sprintf(str,"Head-> %d",SignalsState.HeadLights);
	cvPutText(GI.Data.Info, str, cvPoint(350,30),&font, CV_RGB(0,200,0));
	sprintf(str,"Tail-> %d",SignalsState.TailLights);
	cvPutText(GI.Data.Info, str, cvPoint(350,45),&font, CV_RGB(0,200,0));
	sprintf(str,"Turn R-> %d",SignalsState.TurnRight);
	cvPutText(GI.Data.Info, str, cvPoint(350,60),&font, CV_RGB(0,200,0));
	sprintf(str,"Turn L-> %d",SignalsState.TurnLeft);
	cvPutText(GI.Data.Info, str, cvPoint(350,75),&font, CV_RGB(0,200,0));
	
	if (n>0)
		cvPutText(GI.Data.Info, "PIC2: Sent", cvPoint(650,45),&font, CV_RGB(0,255,0));
	else if(i==MAXNOATTEMPTSTOSEND)
		cvPutText(GI.Data.Info, "PIC2: Error", cvPoint(650,45),&font, CV_RGB(255,0,0));
	
#endif

}

/** @brief Reads the sensos's values from pic 2.
 * 
 * @param comport 
 */
void RS_ReadSensorsFromPIC2(int comport)
{
int n,i,t;
unsigned char byteread[20];
unsigned char ident,cont;


/*__________________________________
   IDENTIFIER	|      SENSOR
	0	|	Cross
	1	|	Right Front
	2	|	Right Back
	3	|	Front Right
	4	|	Front Left
	5	|	Left Front
	6	|	Left Back
	7	|	Tunnel	
____________________________________*/

n = read(comport, &byteread,20);
		
	if( n == -1 ) 
	{
		perror("Failed reading from device--------------------------------------------------");
	}
	

	

	for (i=0;i<n;i++)
	{
		
		
		
		ident=(byteread[i] >>4);
		cont=(byteread[i] & 15 /*=0b00001111*/);
		/*printf("n=%d\n",i);	
		printf("byteread=%d\n",byteread[i]);	
		printf("ident=%d\n",ident);	
		printf("cont=%d\n",cont);*/
		
		switch(ident)
		{
			case 0: // cross sensor
			{
				SensorValues.Cross=cont;	
			break;
			}
			case 1: // Right front sensor
			{	
				for (t=9;t>0;t--)
					SensorValues.RightFront[t]=SensorValues.RightFront[t-1];
					
				SensorValues.RightFront[0]=cont;	
			break;
			}
			case 2: // Right back sensor
			{	
				for (t=9;t>0;t--)
					SensorValues.RightBack[t]=SensorValues.RightBack[t-1];
				
				SensorValues.RightBack[0]=cont;	
			break;
			}
			case 3: // Front Right sensor
			{	
				for (t=9;t>0;t--)
					SensorValues.FrontRight[t]=SensorValues.FrontRight[t-1];
				
				SensorValues.FrontRight[0]=cont;	
			break;
			}
			case 4: // Front Left sensor
			{
				for (t=9;t>0;t--)
					SensorValues.FrontLeft[t]=SensorValues.FrontLeft[t-1];
				
				SensorValues.FrontLeft[0]=cont;
			break;
			}
			case 5: // Left Front sensor
			{	
				for (t=9;t>0;t--)
					SensorValues.LeftFront[t]=SensorValues.LeftFront[t-1];
				
				SensorValues.LeftFront[0]=cont;	
			break;
			}
			case 6: // Left Back sensor
			{
				for (t=9;t>0;t--)
					SensorValues.LeftBack[t]=SensorValues.LeftBack[t-1];	
					
					SensorValues.LeftBack[0]=cont;	
			break;
			}
			case 7: // Left Front sensor
			{
				SensorValues.Tunnel[9]=SensorValues.Tunnel[8];
				SensorValues.Tunnel[8]=SensorValues.Tunnel[7];
				SensorValues.Tunnel[7]=SensorValues.Tunnel[6];
				SensorValues.Tunnel[6]=SensorValues.Tunnel[5];
				SensorValues.Tunnel[5]=SensorValues.Tunnel[4];
				SensorValues.Tunnel[4]=SensorValues.Tunnel[3];
				SensorValues.Tunnel[3]=SensorValues.Tunnel[2];
				SensorValues.Tunnel[2]=SensorValues.Tunnel[1];
				SensorValues.Tunnel[2]=SensorValues.Tunnel[1];
				SensorValues.Tunnel[1]=SensorValues.Tunnel[0];
				SensorValues.Tunnel[0]=cont;	
			break;
			}
		}
	}


}


#endif 
