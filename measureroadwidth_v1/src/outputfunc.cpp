/***************************************************************/
/**TITLE: ROBOT navegation output auxiliary functions************/
/***************************************************************/
/*AUTHORS: Miguel Oliveira, Miguel Neta, Rui Cancela ***********/
/***************************************************************/
/*PROJECT LEADER: Prof. Vitor Santos****************************/
/***************************************************************/
/***************************************************************/
/*PURPOSE: Provides auxiliary functions to handle outputs ******/
/***************************************************************/
/*FILENAME: outputfunc.c****************************************/
/***************************************************************/
/*! @file outputfunc.cpp
* @brief All the functions used to execute some kind of shell output. 
*
*The idea was to create a errornumber database and the an error function that would receive the errornumber and output some text that would be stored in that database to the shell.
*/

#ifndef _OUTPUTFUNC_
#define _OUTPUTFUNC_

#include "header.h"

/** 
 * @brief Function to exit and print error message.
 * @param errornumber number of the error to print out.
 */
void PrintError(char errornumber)
{
exit (errornumber);		
}

#endif
