// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2008, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    API-debug.C
// Author:  Mickle.ding
// Created: 5/18/2007
//
// Description:  
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <stdio.h>
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Platform.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
//#include "API-Timer.h"        //��ʱע�͵�

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
//#include "API-UART0.h"       //��ʱע�͵�

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
//#include "API-W5100.h"        //��ʱע�͵�
   

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
//void debugthread( char * pThreadName, unsigned char iStatus )    //��ʱע�͵�
//{
//   char buffer[0X20];
//   sprintf( buffer, "#%05u %s=%02BX\r\n", GetTimer16(), pThreadName, iStatus );
//   
//   // The timer information
//   PutString( buffer );   
//}