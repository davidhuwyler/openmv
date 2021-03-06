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

#ifndef __BOOTLOADER_FLASH_H_
#define __BOOTLOADER_FLASH_H_


#include <stddef.h>
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "hal_wrapper.h"

#define QSPI_FLASH_START_ADDRESS  0x60000000
#define QSPI_FLASH_NOF_SECTORS_PER_BLOCK 64
#define QSPI_FLASH_SECTOR_SIZE_BYTE 4*1024

#define QSPI_FLASH_LAST_BLOCK 31
#define QSPI_FLASH_MAIN_BLOCK 4//4
#define QSPI_FLASH_MAIN_SECTOR (QSPI_FLASH_MAIN_BLOCK*QSPI_FLASH_NOF_SECTORS_PER_BLOCK)
#define QSPI_FLASH_APP_START_OFFSET (QSPI_FLASH_MAIN_SECTOR*QSPI_FLASH_SECTOR_SIZE_BYTE)

#define MAIN_APP_ADDR           (QSPI_FLASH_START_ADDRESS+(QSPI_FLASH_MAIN_SECTOR*QSPI_FLASH_SECTOR_SIZE_BYTE))


void flash_init(void);
void flash_erase(uint32_t sector);
void flash_write(const uint32_t *src, uint32_t dst, uint32_t size);

#endif //__BOOTLOADER_FLASH_H_