/*
 * FreeModbus Libary: AVR Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- AVR includes -------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define F_CPU 16000000UL // Arduino Nano quartz resonator Hz.

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4

#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 16

//#define REG_DISCRETE_START 1
//#define REG_DISCRETE_NREGS 24
//#define REG_DISCRETE_BYTES ((REG_DISCRETE_NREGS % 8)?((REG_DISCRETE_NREGS / 8 + 1)):(REG_DISCRETE_NREGS / 8))

//#define REG_COILS_START 1
//#define REG_COILS_NREGS 5
//#define REG_COILS_BYTES ((REG_COILS_NREGS % 8)?((REG_COILS_NREGS / 8 + 1)):(REG_COILS_NREGS / 8))

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];

static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

//static USHORT   usRegDiscreteStart = REG_DISCRETE_START;
//static UCHAR    usRegDiscreteBuf[REG_DISCRETE_BYTES];

//static USHORT   usRegCoilsStart = REG_COILS_START;
//static UCHAR    usRegCoilsBuf[REG_COILS_BYTES];

/* ----------------------- Start implementation -----------------------------*/
int
main( void )
{
    const UCHAR     ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
    eMBErrorCode    eStatus;

    // MODBUS RTU SLAVE ADDRESS 10, USART0 38400 8E1.
    //eStatus = eMBInit( MB_RTU, 0x0A, 0, 38400, MB_PAR_EVEN );

    // MODBUS ASCII SLAVE ADDRESS 1, USART0 57600 8N1.
    //eStatus = eMBInit( MB_ASCII, 1, 0, 57600, MB_PAR_NONE );

    // MODBUS RTU SLAVE ADDRESS 1, USART0 9600 8N1.
    eStatus = eMBInit( MB_RTU, 1, 0, 9600, MB_PAR_NONE );

    eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
    sei(  );

    /* Enable the Modbus Protocol Stack. */
    eStatus = eMBEnable(  );

    usRegHoldingBuf[1] = 1; // HOLDING REGISTER 1 int16
    usRegHoldingBuf[2] = -2; // HOLDING REGISTER 2 int16

    for( ;; )
    {
        ( void )eMBPoll(  );

        /* Here we simply count the number of poll cycles. */
        usRegInputBuf[0]++   ; // INPUT REGISTER  999 int16
        usRegInputBuf[1] =  2; // INPUT REGISTER 1000 int16
        usRegInputBuf[2] =  3; // INPUT REGISTER 1001 int16
        usRegInputBuf[3] = -5; // INPUT REGISTER 1002 int16

        usRegHoldingBuf[0] = usRegHoldingBuf[0] + 1; // HOLDING REGISTER 0 int16
        usRegHoldingBuf[3] = usRegHoldingBuf[2] + usRegHoldingBuf[1]; // HOLDING REGISTER 3 int16
    }
}


eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
#ifdef REG_INPUT_START
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
#else
    return MB_ENOREG;
#endif
}


eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
#ifdef REG_HOLDING_START
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
            ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
            iRegIndex = ( int )( usAddress - usRegHoldingStart );
            switch ( eMode )
        {
        case MB_REG_READ:
                    while( usNRegs > 0 )
            {
                            *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                            *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                            iRegIndex++;
                            usNRegs--;
            }
                    break;
        case MB_REG_WRITE:
                    while( usNRegs > 0 )
            {
                            usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                            usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                            iRegIndex++;
                            usNRegs--;
            }
                    break;
        }
    }
    else
    {
            eStatus = MB_ENOREG;
    }

    return eStatus;
#else
    return MB_ENOREG;
#endif
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
#ifdef REG_COILS_START
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_NREGS)){
        iRegIndex = ( int ) ( usAddress - usRegCoilsStart );
        switch ( eMode ){
        case MB_REG_READ:
            while ( usNCoils > 0 ) {
                UCHAR ucResult = xMBUtilGetBits( usRegCoilsBuf, iRegIndex, 1 );
                xMBUtilSetBits( pucRegBuffer, iRegIndex - ( usAddress - usRegCoilsStart ), 1, ucResult );
                iRegIndex++;
                usNCoils--;
            }
            break;
        case MB_REG_WRITE:
            while ( usNCoils > 0 ) {
                UCHAR ucResult = xMBUtilGetBits( pucRegBuffer, iRegIndex - ( usAddress - usRegCoilsStart ), 1 );
                xMBUtilSetBits( usRegCoilsBuf, iRegIndex, 1, ucResult );
                iRegIndex++;
                usNCoils--;
            }
            break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
#else
    return MB_ENOREG;
#endif
}


eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
#ifdef REG_DISCRETE_START
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_DISCRETE_START) && (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_NREGS)){
        iRegIndex = ( int ) ( usAddress - usRegDiscreteStart );
        while (usNDiscrete > 0) {
            UCHAR ucResult = xMBUtilGetBits(usRegDiscreteBuf, iRegIndex, 1);
            xMBUtilSetBits(pucRegBuffer, iRegIndex - (usAddress - usRegDiscreteStart), 1, ucResult);
            iRegIndex++;
            usNDiscrete--;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
#else
    return MB_ENOREG;
#endif
}
