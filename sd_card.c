//*****************************************************************************
//
// sd_card.c - Example program for reading files from an SD card.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
#include "cirbuf.h"
#include "format.h"
#include "dac.h"

// CMSIS
#include "arm_math.h"
#include "fir_filter.h"

#define _CAT

//*****************************************************************************
//
// Section for ADC, Timer, uDMA added
//
//*****************************************************************************
#define BUFFER_SIZE       512

#define BUF_CHECK         0
#define BUF_INC           1
#define BUF_DEC           2

//*****************************************************************************
//
// Circular buffer impelementation
//
//*****************************************************************************
#ifndef BUFFER_LENGTH
#define BUFFER_LENGTH 1024
#endif

#define BUF_SIZE     6
#define ELEMENT_SIZE        512

//*****************************************************************************
//
// Circular buffers for uDMA ping-pong operation
//
//*****************************************************************************
volatile bufT adcBuf;
volatile bufT * gpBuf;
volatile elementT * pingPtr;
volatile elementT * pongPtr;
volatile bool bufferOverflow = false;
volatile bool stop;
volatile uint8_t testCount = 0;

//*****************************************************************************
//
// Control table used by the DMA controller. Table must be aligned to 1024 byte
// boundary.
//
//*****************************************************************************
#pragma DATA_ALIGN(controlTable, 1024)
uint8_t controlTable[1024];

//*****************************************************************************
//
// uDMA errors counter.
//
//*****************************************************************************
volatile uint32_t uDMAErrorCounter = 0;
volatile uint8_t sysTickTest = 0;
// TEST VARIABLES
volatile uint32_t doneTimes = 0;

// DAC output variables
volatile uint16_t dacIndex = 0;
volatile elementT * dacBuf;

void uDMAErrorHandler(void)
{
  uint32_t status;

  status = uDMAErrorStatusGet();

  if (status) {
    uDMAErrorCounter++;
    uDMAErrorStatusClear();
  }
}

//*****************************************************************************
//
// This function config all peripherals for data acquisition
//
//*****************************************************************************

void acqConfig(void) {

    // ADC0 configuration
    // ADC Sequencer 3, channel internal temperature
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    // Prevent hardfault because of clock instability.
    SysCtlDelay(100000);

    // Enable ADC channel
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // CH0

    // Configure the sequencer
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                            ADC_CTL_END);

    // Allow DMA channel request upon ADC completion.
    ADCSequenceDMAEnable(ADC0_BASE, 3);



    // uDMA configuration
    // Clocking for uDMA module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Allow uDMA module to run while CPU is sleeping.
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    // Enable uDMA before configuring it
    uDMAEnable();

    // Setup control table for uDMA
    uDMAControlBaseSet(controlTable);

    // Make sure default parameters are set.
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3, UDMA_ATTR_USEBURST |
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    // Config option for uDMA channels
    uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                          UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                          UDMA_ARB_1);

    // Transfer setting for first pair of transfer.
    pingPtr = bufGetFree(gpBuf);
    pongPtr = bufGetFree(gpBuf);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                          UDMA_MODE_PINGPONG,
                          (void *) (ADC0_BASE + ADC_O_SSFIFO3),
                          (void *) pingPtr->data,
                          ELEMENT_SIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
                          UDMA_MODE_PINGPONG,
                          (void *) (ADC0_BASE + ADC_O_SSFIFO3),
                          (void *) pongPtr->data,
                          ELEMENT_SIZE);

    // TIMER 0 configuration
    // Clock the TIMER 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Use timer 0 as a full-width timer.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set timer to trigger ADC at rate 8000Hz
    TimerLoadSet(TIMER0_BASE, TIMER_A, SYS_CLK/32000); // 32ksps

    // Trigger ADC using timer
    TimerControlTrigger(TIMER0_BASE, TIMER_A, 1);

    // Enable interrupt
    IntEnable(INT_ADC0SS3);
    IntEnable(INT_UDMAERR); // uDMA error

    // Enable ADC & uDMA
    ADCSequenceEnable(ADC0_BASE, 3);
    uDMAChannelEnable(UDMA_CHANNEL_ADC3); // Enable uDMA channel for operation
}

//*****************************************************************************
//
// Interurpt handler for data acquisition
//
//*****************************************************************************

void adcInterruptHandler(void)
{
  uint32_t mode;

  ADCIntClear(ADC0_BASE, 3);

  // Check if the PING buffer is full
  mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT);

  // Data was received complete into PING buffer. So the controller is transfer
  // using PONG buffer.
  if (mode == UDMA_MODE_STOP) {
    // Set the buffer elment status to FREE
    bufItemSetFree(gpBuf, pingPtr->index);

    // Get new free location
    pingPtr = bufGetFree(gpBuf);
    if (pingPtr) {
      // Setup new transfer
      uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
                              UDMA_MODE_PINGPONG,
                              (void *) (ADC0_BASE + ADC_O_SSFIFO3),
                              (void *) pingPtr->data,
                              BUFFER_SIZE);
    }
    else {
      bufferOverflow = true;
      while(1);
    }


    // Toggle the Pin
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,
    //    (uint8_t) (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)) ^ ((1 << 2)));
  }

  // Check if PONG transfer is completed
  mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT);

  // Data was received complete into PONG buffer.
  if (mode == UDMA_MODE_STOP) {
    bufItemSetFree(gpBuf, pongPtr->index);

    pongPtr = bufGetFree(gpBuf);

    if (pongPtr) {
      uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
                              UDMA_MODE_PINGPONG,
                              (void *) (ADC0_BASE + ADC_O_SSFIFO3),
                              (void *) pongPtr->data,
                              BUFFER_SIZE);
    }
    else {
      bufferOverflow = true;
      while(1);
    }

    // Toggle the Pin
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,
    //             (uint8_t) (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) ^ ((1 << 1)));
  }

  doneTimes++;

}

//****************************************************************************
//
// END SECTION
//
//****************************************************************************


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>SD card using FAT file system (sd_card)</h1>
//!
//! This example application demonstrates reading a file system from an SD
//! card.  It makes use of FatFs, a FAT file system driver.  It provides a
//! simple command console via a serial port for issuing commands to view and
//! navigate the file system on the SD card.
//!
//! The first UART, which is connected to the USB debug virtual serial port on
//! the evaluation board, is configured for 115,200 bits per second, and 8-N-1
//! mode.  When the program is started a message will be printed to the
//! terminal.  Type ``help'' for command help.
//!
//! For additional details about FatFs, see the following site:
//! http://elm-chan.org/fsw/ff/00index_e.html
//
//*****************************************************************************

//*****************************************************************************
//
// Defines the size of the buffers that hold the path, or temporary data from
// the SD card.  There are two buffers allocated of this size.  The buffer size
// must be large enough to hold the longest expected full path name, including
// the file name, and a trailing null character.
//
//*****************************************************************************
#define PATH_BUF_SIZE           80

//*****************************************************************************
//
// Defines the size of the buffer that holds the command line.
//
//*****************************************************************************
#define CMD_BUF_SIZE            64

//*****************************************************************************
//
// This buffer holds the full path to the current working directory.  Initially
// it is root ("/").
//
//*****************************************************************************
static char g_pcCwdBuf[PATH_BUF_SIZE] = "/";

//*****************************************************************************
//
// A temporary data buffer used when manipulating file paths, or reading data
// from the SD card.
//
//*****************************************************************************
static char g_pcTmpBuf[PATH_BUF_SIZE];

//*****************************************************************************
//
// The buffer that holds the command line.
//
//*****************************************************************************
static char g_pcCmdBuf[CMD_BUF_SIZE];

//*****************************************************************************
//
// The following are data structures used by FatFs.
//
//*****************************************************************************
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;

//*****************************************************************************
//
// A structure that holds a mapping between an FRESULT numerical code, and a
// string representation.  FRESULT codes are returned from the FatFs FAT file
// system driver.
//
//*****************************************************************************
typedef struct
{
    FRESULT iFResult;
    char *pcResultStr;
}
tFResultString;

//*****************************************************************************
//
// A macro to make it easy to add result codes to the table.
//
//*****************************************************************************
#define FRESULT_ENTRY(f)        { (f), (#f) }

//*****************************************************************************
//
// A table that holds a mapping between the numerical FRESULT code and it's
// name as a string.  This is used for looking up error codes for printing to
// the console.
//
//*****************************************************************************
tFResultString g_psFResultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_DISK_ERR),
    FRESULT_ENTRY(FR_INT_ERR),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_MKFS_ABORTED),
    FRESULT_ENTRY(FR_TIMEOUT),
    FRESULT_ENTRY(FR_LOCKED),
    FRESULT_ENTRY(FR_NOT_ENOUGH_CORE),
    FRESULT_ENTRY(FR_TOO_MANY_OPEN_FILES),
    FRESULT_ENTRY(FR_INVALID_PARAMETER),
};

//*****************************************************************************
//
// A macro that holds the number of result codes.
//
//*****************************************************************************
#define NUM_FRESULT_CODES       (sizeof(g_psFResultStrings) /                 \
                                 sizeof(tFResultString))

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
#ifdef _USE_GRLIB
tContext g_sContext;
#endif

//*****************************************************************************
//
// This function returns a string representation of an error code that was
// returned from a function call to FatFs.  It can be used for printing human
// readable error messages.
//
//*****************************************************************************
const char *
StringFromFResult(FRESULT iFResult)
{
    uint_fast8_t ui8Idx;

    //
    // Enter a loop to search the error code table for a matching error code.
    //
    for(ui8Idx = 0; ui8Idx < NUM_FRESULT_CODES; ui8Idx++)
    {
        //
        // If a match is found, then return the string name of the error code.
        //
        if(g_psFResultStrings[ui8Idx].iFResult == iFResult)
        {
            return(g_psFResultStrings[ui8Idx].pcResultStr);
        }
    }

    //
    // At this point no matching code was found, so return a string indicating
    // an unknown error.
    //
    return("UNKNOWN ERROR CODE");
}

//*****************************************************************************
//
// This is the handler for this SysTick interrupt.  FatFs requires a timer tick
// every 10 ms for internal timing purposes.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    //
    // Call the FatFs tick timer.
    //
    disk_timerproc();
    sysTickTest++;
    if (sysTickTest == 2 ) {
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) ^ GPIO_PIN_2));

      sysTickTest = 0;
    }
}

//*****************************************************************************
//
// This function implements the "ls" command.  It opens the current directory
// and enumerates through the contents, and prints a line for each item it
// finds.  It shows details such as file attributes, time and date, and the
// file size, along with the name.  It shows a summary of file sizes at the end
// along with free space.
//
//*****************************************************************************
int
Cmd_ls(int argc, char *argv[])
{
    uint32_t ui32TotalSize;
    uint32_t ui32FileCount;
    uint32_t ui32DirCount;
    FRESULT iFResult;
    FATFS *psFatFs;
    char *pcFileName;
#if _USE_LFN
    char pucLfn[_MAX_LFN + 1];
    g_sFileInfo.lfname = pucLfn;
    g_sFileInfo.lfsize = sizeof(pucLfn);
#endif


    //
    // Open the current directory for access.
    //
    iFResult = f_opendir(&g_sDirObject, g_pcCwdBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(iFResult != FR_OK)
    {
        return((int)iFResult);
    }

    ui32TotalSize = 0;
    ui32FileCount = 0;
    ui32DirCount = 0;

    //
    // Give an extra blank line before the listing.
    //
    UARTprintf("\n");

    //
    // Enter loop to enumerate through all directory entries.
    //
    for(;;)
    {
        //
        // Read an entry from the directory.
        //
        iFResult = f_readdir(&g_sDirObject, &g_sFileInfo);

        //
        // Check for error and return if there is a problem.
        //
        if(iFResult != FR_OK)
        {
            return((int)iFResult);
        }

        //
        // If the file name is blank, then this is the end of the listing.
        //
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        //
        // If the attribue is directory, then increment the directory count.
        //
        if(g_sFileInfo.fattrib & AM_DIR)
        {
            ui32DirCount++;
        }

        //
        // Otherwise, it is a file.  Increment the file count, and add in the
        // file size to the total.
        //
        else
        {
            ui32FileCount++;
            ui32TotalSize += g_sFileInfo.fsize;
        }

#if _USE_LFN
        pcFileName = ((*g_sFileInfo.lfname)?g_sFileInfo.lfname:g_sFileInfo.fname);
#else
        pcFileName = g_sFileInfo.fname;
#endif
        //
        // Print the entry information on a single line with formatting to show
        // the attributes, date, time, size, and name.
        //
        UARTprintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n",
                   (g_sFileInfo.fattrib & AM_DIR) ? 'D' : '-',
                   (g_sFileInfo.fattrib & AM_RDO) ? 'R' : '-',
                   (g_sFileInfo.fattrib & AM_HID) ? 'H' : '-',
                   (g_sFileInfo.fattrib & AM_SYS) ? 'S' : '-',
                   (g_sFileInfo.fattrib & AM_ARC) ? 'A' : '-',
                   (g_sFileInfo.fdate >> 9) + 1980,
                   (g_sFileInfo.fdate >> 5) & 15,
                   g_sFileInfo.fdate & 31,
                   (g_sFileInfo.ftime >> 11),
                   (g_sFileInfo.ftime >> 5) & 63,
                   g_sFileInfo.fsize,
                   pcFileName);
    }

    //
    // Print summary lines showing the file, dir, and size totals.
    //
    UARTprintf("\n%4u File(s),%10u bytes total\n%4u Dir(s)",
                ui32FileCount, ui32TotalSize, ui32DirCount);

    //
    // Get the free space.
    //
    iFResult = f_getfree("/", (DWORD *)&ui32TotalSize, &psFatFs);

    //
    // Check for error and return if there is a problem.
    //
    if(iFResult != FR_OK)
    {
        return((int)iFResult);
    }

    //
    // Display the amount of free space that was calculated.
    //
    UARTprintf(", %10uK bytes free\n", (ui32TotalSize *
                                        psFatFs->free_clust / 2));

    //
    // Made it to here, return with no errors.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "cd" command.  It takes an argument that
// specifies the directory to make the current working directory.  Path
// separators must use a forward slash "/".  The argument to cd can be one of
// the following:
//
// * root ("/")
// * a fully specified path ("/my/path/to/mydir")
// * a single directory name that is in the current directory ("mydir")
// * parent directory ("..")
//
// It does not understand relative paths, so dont try something like this:
// ("../my/new/path")
//
// Once the new directory is specified, it attempts to open the directory to
// make sure it exists.  If the new path is opened successfully, then the
// current working directory (cwd) is changed to the new path.
//
//*****************************************************************************
int
Cmd_cd(int argc, char *argv[])
{
    uint_fast8_t ui8Idx;
    FRESULT iFResult;

    //
    // Copy the current working path into a temporary buffer so it can be
    // manipulated.
    //
    strcpy(g_pcTmpBuf, g_pcCwdBuf);

    //
    // If the first character is /, then this is a fully specified path, and it
    // should just be used as-is.
    //
    if(argv[1][0] == '/')
    {
        //
        // Make sure the new path is not bigger than the cwd buffer.
        //
        if(strlen(argv[1]) + 1 > sizeof(g_pcCwdBuf))
        {
            UARTprintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // If the new path name (in argv[1])  is not too long, then copy it
        // into the temporary buffer so it can be checked.
        //
        else
        {
            strncpy(g_pcTmpBuf, argv[1], sizeof(g_pcTmpBuf));
        }
    }

    //
    // If the argument is .. then attempt to remove the lowest level on the
    // CWD.
    //
    else if(!strcmp(argv[1], ".."))
    {
        //
        // Get the index to the last character in the current path.
        //
        ui8Idx = strlen(g_pcTmpBuf) - 1;

        //
        // Back up from the end of the path name until a separator (/) is
        // found, or until we bump up to the start of the path.
        //
        while((g_pcTmpBuf[ui8Idx] != '/') && (ui8Idx > 1))
        {
            //
            // Back up one character.
            //
            ui8Idx--;
        }

        //
        // Now we are either at the lowest level separator in the current path,
        // or at the beginning of the string (root).  So set the new end of
        // string here, effectively removing that last part of the path.
        //
        g_pcTmpBuf[ui8Idx] = 0;
    }

    //
    // Otherwise this is just a normal path name from the current directory,
    // and it needs to be appended to the current path.
    //
    else
    {
        //
        // Test to make sure that when the new additional path is added on to
        // the current path, there is room in the buffer for the full new path.
        // It needs to include a new separator, and a trailing null character.
        //
        if(strlen(g_pcTmpBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_pcCwdBuf))
        {
            UARTprintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // The new path is okay, so add the separator and then append the new
        // directory to the path.
        //
        else
        {
            //
            // If not already at the root level, then append a /
            //
            if(strcmp(g_pcTmpBuf, "/"))
            {
                strcat(g_pcTmpBuf, "/");
            }

            //
            // Append the new directory to the path.
            //
            strcat(g_pcTmpBuf, argv[1]);
        }
    }

    //
    // At this point, a candidate new directory path is in chTmpBuf.  Try to
    // open it to make sure it is valid.
    //
    iFResult = f_opendir(&g_sDirObject, g_pcTmpBuf);

    //
    // If it can't be opened, then it is a bad path.  Inform the user and
    // return.
    //
    if(iFResult != FR_OK)
    {
        UARTprintf("cd: %s\n", g_pcTmpBuf);
        return((int)iFResult);
    }

    //
    // Otherwise, it is a valid new path, so copy it into the CWD.
    //
    else
    {
        strncpy(g_pcCwdBuf, g_pcTmpBuf, sizeof(g_pcCwdBuf));
    }

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "pwd" command.  It simply prints the current
// working directory.
//
//*****************************************************************************
int
Cmd_pwd(int argc, char *argv[])
{
    //
    // Print the CWD to the console.
    //
    UARTprintf("%s\n", g_pcCwdBuf);

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "cat" command.  It reads the contents of a file
// and prints it to the console.  This should only be used on text files.  If
// it is used on a binary file, then a bunch of garbage is likely to printed on
// the console.
//
// Mar 17, 2014. Modified for "nano" like command
// Data buffer is filled with useless data
//*****************************************************************************
int
Cmd_cat(int argc, char *argv[])
{
    FRESULT iFResult;
    uint32_t ui32BytesRead;

    uint32_t numOfSamples;
    uint32_t subChunk2Size; // Wave subchunksize data
    uint32_t chunkSize;
    uint32_t count = 0; // Number of elements to acquire
    uint32_t filesize = 0;
    uint32_t numOfByteToRead;

    uint8_t fmtHeader[80];

    elementT * bufData;
    uint8_t i;

    stop = false;

    bufInit(gpBuf);

    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_pcCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_pcTmpBuf))
    {
        UARTprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_pcTmpBuf, g_pcCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_pcCwdBuf))
    {
        strcat(g_pcTmpBuf, "/");
    }

    //
    // Now finally, append the file name to result in a fully specified file.
    //
    strcat(g_pcTmpBuf, argv[1]);

    //
    // Open the file for reading.
    //
    iFResult = f_open(&g_sFileObject, g_pcTmpBuf, FA_READ);

    //
    // If there was some problem opening the file, then return an error.
    //
    if(iFResult != FR_OK)
    {
        return((int)iFResult);
    }


    // 0. Read the file size
    filesize = f_size(&g_sFileObject);

    // 1. Read the WAV file header
    iFResult = f_read(&g_sFileObject, fmtHeader, 44, (UINT *)&ui32BytesRead);
    if (iFResult != FR_OK) {
      return ((int) iFResult);
    }

    subChunk2Size = ((uint32_t) fmtHeader[43] << 24) |
                    ((uint32_t) fmtHeader[42] << 16) |
                    ((uint32_t) fmtHeader[41] << 8) |
                    (uint32_t) fmtHeader[40];

    filesize -= 44; // Removed header

    // Seek to the location of the first sample data
    iFResult = f_lseek(&g_sFileObject, 44);
    if (iFResult != FR_OK) {
      return ((int) iFResult);
    }

    // Get a free buffer and preload first sector
    bufData = bufGetFree(gpBuf);
    iFResult = f_read(&g_sFileObject, bufData->data, 1024, (UINT *)&ui32BytesRead);
    if (iFResult != FR_OK) {
      return ((int) iFResult);
    }

    // Remember to set the buffer element FREE after using it
    bufItemSetFree(gpBuf, bufData->index);

    // Preload data
    dacBuf = bufGet(gpBuf);

    // Enable DAC
    dacEnable();

    // Loop to read data from the file
    while (!stop) {
      // Get a free buffer
      bufData = bufGetFree(gpBuf);

      // If there is free room for new data
      if (bufData) {
        if (filesize > 1024) {
          filesize -= 1024;

          // Read data to the buffer
          iFResult = f_read(&g_sFileObject, bufData->data, 1024, (UINT *)&ui32BytesRead);
          if (iFResult != FR_OK) {
            return ((int) iFResult);
          }

          bufItemSetFree(gpBuf, bufData->index);
        }
        else {
          iFResult = f_read(&g_sFileObject, bufData->data, filesize, (UINT *)&ui32BytesRead);
          stop = true; // End of file, ok to stop
        }
      }


    }

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
// Mar 17, 2014. Modified "cat" for "nano" like command
// Data buffer is filled with useless data
//*****************************************************************************
int
Cmd_nano(int argc, char *argv[])
{
  FRESULT iFResult;
  UINT bw;
  uint32_t numOfSamples;
  uint32_t subChunk2Size; // Wave subchunksize data
  uint32_t chunkSize;
  uint32_t count; // Number of elements to acquire

  static uint8_t fmtHeader[80];

  static elementT * bufData;
  uint16_t i;
  uint8_t t;

  // Data filtering helper arrays
  static float32_t inputf32[LENGTH]; // Filter inputs
  static float32_t outputf32[LENGTH]; // Filter output
  static float32_t firBufferf32[BLOCK_SIZE + TAPS - 1]; // Buffer state // todo fix
  static int16_t bufferOutputi16[2048];
  static int16_t outputi16[512];

  // Filter kernel structure
  static arm_fir_instance_f32 s;
  static uint32_t blocksize = BLOCK_SIZE;
  uint8_t timesDone = 0;
  uint8_t numOfBlocks = LENGTH/blocksize;

  static float32_t *input, *output;
  input = inputf32;
  output = outputf32;

  //
  // FIR filter initialization
  //
  arm_fir_init_f32(&s, TAPS, (float32_t *) &firCoeffsf32[0], &firBufferf32[0], blocksize);

  // Stop action
  stop = false;
  count = 0;

  // Copy header from format.h
  for (i = 0; i < 44; i++) {
    fmtHeader[i] = wavHeader[i];
  };

  // Init the buffer
  bufInit(gpBuf);
  acqConfig();

  // First, check to make sure that the current path (CWD), plus the file
  // name, plus a separator and trailing null, will all fit in the temporary
  // buffer that will be used to hold the file name.  The file name must be
  // fully specified, with path, to FatFs.
  if(strlen(g_pcCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_pcTmpBuf))
  {
      UARTprintf("Resulting path name is too long\n");
      return(0);
  }

  //
  // Copy the current path to the temporary buffer so it can be manipulated.
  //
  strcpy(g_pcTmpBuf, g_pcCwdBuf);

  //
  // If not already at the root level, then append a separator.
  //
  if(strcmp("/", g_pcCwdBuf))
  {
      strcat(g_pcTmpBuf, "/");
  }

  //
  // Now finally, append the file name to result in a fully specified file.
  //
  strcat(g_pcTmpBuf, argv[1]);
  //
  // Open the file for writing.
  //
  iFResult = f_open(&g_sFileObject, g_pcTmpBuf, FA_WRITE | FA_OPEN_ALWAYS);
  //
  // If there was some problem opening the file, then return an error.
  //
  if(iFResult != FR_OK)
  {
      return((int)iFResult);
  }

  // Allocate space for WAV header
  do {
    iFResult = f_write(&g_sFileObject, fmtHeader, 44,
                       (UINT *)&bw);

    if(iFResult != FR_OK)
    {
        UARTprintf("not ok\n");
        return((int)iFResult);
    }
  }
  while (bw < 44);

  // Enable timer for data acquisition
  TimerEnable(TIMER0_BASE, TIMER_A);

  // Check the buffer and write data to the disk
  while (!stop) {
    t = 0;
    while (t < 4) {
      bufData = bufGet(gpBuf);

      // If data available at the buffer, process it
      if (bufData) {
        // WAVE file format compatibility
        for (i = 0; i < 512; i++) {
          bufData->data[i] -= 2048;

        // Convert from int16 to f32 and copy to new array
        inputf32[i] = ((float32_t) (bufData->data[i]) / INT16_MAX);
        }

        // Update status of the element used
        bufData->status = FREE;

        //UARTprintf("Begin filter!\n");

        // Filter it and save to the temporary buffer
        for (i = 0; i < numOfBlocks; i++) {
          arm_fir_f32(&s, input + (i * blocksize), output + (i * blocksize),
                     (uint32_t) blocksize);
        }
        timesDone++;
        //UARTprintf("%d\n", timesDone);
        //UARTprintf("DONE!\n");
        //while(1);

        // Convert and copy the filtered output to the buffer array
        for (i = 0; i < 512; i++) {
          bufferOutputi16[t*512 + i] = (outputf32[i] * INT16_MAX);
        }

        t++;
      }
    }

    // Get only 1 per 4 samples (downsampling)
    for (i = 0; i < 512; i++) {
      outputi16[i] = bufferOutputi16[i * 4];
    }

    // Write data to the disk
    iFResult = f_write(&g_sFileObject, outputi16, elementSize*2, (UINT *)&bw);

    if (iFResult != FR_OK) {
      return ((int) iFResult);
    }

    count++;

    if (count >= 2000) { // Stop token
      stop = true;
    }
  }

  // Disable timer
  TimerDisable(TIMER0_BASE, TIMER_A);

  // Calulate chunksizes
  numOfSamples = count; // TODO This is temporary
  subChunk2Size = numOfSamples * 2 * 512;
  chunkSize  = subChunk2Size + 36;

  // Modify header Chunk Size
  fmtHeader[4] = (uint8_t) chunkSize;
  fmtHeader[5] = (uint8_t) (chunkSize >> 8);
  fmtHeader[6] = (uint8_t) (chunkSize >> 16);
  fmtHeader[7] = (uint8_t) (chunkSize >> 24);

  // Modify header Sub Chunk 2 Size
  fmtHeader[40] = (uint8_t) subChunk2Size;
  fmtHeader[41] = (uint8_t) (subChunk2Size >> 8);
  fmtHeader[42] = (uint8_t) (subChunk2Size >> 16);
  fmtHeader[43] = (uint8_t) (subChunk2Size >> 24);

  // Seek to the first location of the file and write the wav header
  iFResult = f_lseek(&g_sFileObject, 0);
  if(iFResult != FR_OK)
  {
      return((int)iFResult);
  }

  iFResult = f_write(&g_sFileObject, fmtHeader, 44, (UINT *)&bw);

  if (iFResult != FR_OK) {
    return ((int) iFResult);
  }

  f_close(&g_sFileObject);

  //
  // Return success.
  //
  return(0);
}

//*****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of the
// available commands with a brief description.
//
//*****************************************************************************
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *psEntry;

    //
    // Print some header text.
    //
    UARTprintf("\nAvailable commands\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    psEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(psEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%6s: %s\n", psEntry->pcCmd, psEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        psEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This is the table that holds the command names, implementing functions, and
// brief description.
//
//*****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    { "help",   Cmd_help,   "Display list of commands" },
    { "h",      Cmd_help,   "alias for help" },
    { "?",      Cmd_help,   "alias for help" },
    { "ls",     Cmd_ls,     "Display list of files" },
    { "chdir",  Cmd_cd,     "Change directory" },
    { "cd",     Cmd_cd,     "alias for chdir" },
    { "pwd",    Cmd_pwd,    "Show current working directory" },
    { "cat",    Cmd_cat,    "Show contents of a text file" },
    { "nano",   Cmd_nano,   "Create a file and write dump to it."},
    { 0, 0, 0 }
};

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// The program main function.  It performs initialization, then runs a command
// processing loop to read commands from the console.
//
//*****************************************************************************
int
main(void)
{
    int nStatus;
    FRESULT iFResult;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the system clock to run at 80MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure SysTick for a 100Hz interrupt.  The FatFs driver wants a 10 ms
    // tick.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
    ROM_SysTickEnable();
    ROM_SysTickIntEnable();

    //
    // Enable Interrupts
    //
    ROM_IntMasterEnable();

    //
    // Initialize the UART as a console for text I/O.
    //
    ConfigureUART();

    //
    // Buffer initialization
    //
    gpBuf = &adcBuf;
    bufInit(gpBuf);

    // Setup data acquisition
    acqConfig();

    // Setup DAC
    dacSetup();

    // TEST SECTION
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    // END TEST SECTION

    //
    // Print hello message to user.
    //
    UARTprintf("\n\nSD Card Example Program\n");
    UARTprintf("Type \'help\' for help.\n");

    //
    // Mount the file system, using logical disk 0.
    //
    iFResult = f_mount(0, &g_sFatFs);
    if(iFResult != FR_OK)
    {
        UARTprintf("f_mount error: %s\n", StringFromFResult(iFResult));
        return(1);
    }


    //
    // Enter an infinite loop for reading and processing commands from the
    // user.
    //
    while(1)
    {
        //
        // Print a prompt to the console.  Show the CWD.
        //
        UARTprintf("\n%s> ", g_pcCwdBuf);

        //
        // Get a line of text from the user.
        //
        UARTgets(g_pcCmdBuf, sizeof(g_pcCmdBuf));

        //
        // Pass the line from the user to the command processor.  It will be
        // parsed and valid commands executed.
        //
        nStatus = CmdLineProcess(g_pcCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            UARTprintf("Bad command!\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            UARTprintf("Too many arguments for command processor!\n");
        }

        //
        // Otherwise the command was executed.  Print the error code if one was
        // returned.
        //
        else if(nStatus != 0)
        {
            UARTprintf("Command returned error code %s\n",
                        StringFromFResult((FRESULT)nStatus));
        }
    }
}
