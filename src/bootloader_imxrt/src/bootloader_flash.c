/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "bootloader_flash.h"
#include "flash_QuadSpi.h"

extern void __fatal_error();

void flash_init(void)
{
    flexspi_nor_init();
}


/* 
    STM32 (OpenMV) sectors are translated to i.MX RT QSPI Blocks:

    OpenMV sectors: 
        Numbers:    0...15
        SectorSize: 128kB

    i.MX RT QSPI Blocks:
        Numbers:    0...31
        Blocksize:  256kB
*/
void flash_erase(uint32_t block)
{
    uint32_t block_sector_number = (block*QSPI_FLASH_NOF_SECTORS_PER_BLOCK);

    uint32_t primask;
    primask = DisableGlobalIRQ();

    uint32_t status;

    for(int i = 0 ; i < QSPI_FLASH_NOF_SECTORS_PER_BLOCK ; i++)
    {
        status = flexspi_nor_flash_erase_sector(FLEXSPI, (block_sector_number+i)*QSPI_FLASH_SECTOR_SIZE_BYTE);
        if (status != kStatus_Success)
        {
            return;
        }
    }

    EnableGlobalIRQ(primask);
}

void flash_write(const uint32_t *src, uint32_t dst, uint32_t size)
{
    // Program the flash 256 bytes at a time.
    uint32_t primask;
    primask = DisableGlobalIRQ();

    for (int i=0; i<size/256; i++) {
        if (flexspi_nor_flash_page_program(FLEXSPI, dst, src) != kStatus_Success) {
            // error occurred during flash write
            __fatal_error();
        }
        src += 64;
        dst += 256;
    }

    EnableGlobalIRQ(primask);
}
