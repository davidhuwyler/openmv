/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    board.c
 * @brief   Board initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#ifndef   __RESTRICT
  #define __RESTRICT                             __restrict
#endif

#include <stdint.h>

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"

//int _sbrk() {return 0;}
extern uint32_t _heap_end;
void *_sbrk ( uint32_t delta )
{
extern char _end; /* Defined by the linker */
static char *heap_end;
char *prev_heap_end;

  if (heap_end == 0) {
    heap_end = &_heap_end;
  }

  prev_heap_end = heap_end;
  //if (prev_heap_end+delta > get_stack_pointer()) {
  //       return (void *) -1L;
  //}
  heap_end += delta;
  return (void *) prev_heap_end;
}

/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
/* Get debug console frequency. */
uint32_t BOARD_DebugConsoleSrcFreq(void)
{
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
       from application is use PLL3 source or OSC source */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
    {
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }
    else
    {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
}
/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq = BOARD_DebugConsoleSrcFreq();

    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

/* MPU configuration. */
void BOARD_ConfigMPU(void)
{
#if 1
	uint32_t rgn = 0;
    /* Disable I cache and D cache */
    SCB_DisableICache();
    SCB_DisableDCache();
    /* Disable MPU */
    ARM_MPU_Disable();

    //AN12042 Using the i.MXRT L1 Cache has some infos on page 3

    /**
    * MPU Region Attribute and Size Register Value
    *
    * \param TEX DisableExec       Instruction access disable bit, 1= disable instruction fetches.
    * \param AP  AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
    * \param     TypeExtField      Type extension field, allows you to configure memory access type, for example strongly ordered, peripheral.
    * \param S   IsShareable       Region is shareable between multiple bus masters.
    * \param C   IsCacheable       Region is cacheable, i.e. its value may be kept in cache.
    * \param B   IsBufferable      Region is bufferable, i.e. using write-back caching. Cacheable but non-bufferable regions use write-through policy.
    * \param     SubRegionDisable  Sub-region disable field.
    * \param     Size              Region size of the region to be configured, for example 4K, 8K.
    */

    //MPU->RASR=ARM_MPU_RASR(TEX, AP           ,  , S, C, B, 0, Size);
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);	// itcm, max 512kB
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_128KB);


    MPU->RBAR = ARM_MPU_RBAR(1, 0x20000000U);	// dtcm, max 512kB
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_256KB);

    MPU->RBAR = ARM_MPU_RBAR(2, 0x20200000U);	// ocram
	// rocky: Must NOT set to device or strong ordered types, otherwise, unaligned access leads to fault	// ocram
	// better to disable bufferable ---- write back, so CPU always write through, avoid DMA and CPU write same line, error prone
	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_128KB);

    MPU->RBAR = ARM_MPU_RBAR(3, 0x60000000U);	//ext Flash
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_8MB);

    //From RT1050_BriefOberview_v201.pdf page 12
	#if defined(SDRAM_MPU_INIT)
	/* Region 7 setting */
	MPU->RBAR = ARM_MPU_RBAR(4, 0x80000000U);
	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0,	ARM_MPU_REGION_SIZE_32MB);
	/* Region 8 setting */
	MPU->RBAR = ARM_MPU_RBAR(4, 0x81E00000U);
	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0,	ARM_MPU_REGION_SIZE_2MB);
	#endif

    /* Enable MPU, enable background region for priviliged access */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable I cache and D cache */
    SCB_EnableDCache();
    SCB_EnableICache();
#endif
}

