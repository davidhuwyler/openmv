/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Board configuration and pin definitions.
 *
 */
#ifndef __OMV_BOARDCONFIG_H__
#define __OMV_BOARDCONFIG_H__
#include "pin.h"
// Architecture info
#define OMV_ARCH_STR            "OpenMV i.MX RT1050/60 port" // 33 chars max
#define OMV_BOARD_TYPE          "M7"
#define OMV_UNIQUE_ID_ADDR      0x1FF0F420

// Sensor external clock frequency. (PixelCLock)
#define OMV_XCLK_FREQUENCY      (15000000)

// Sensor PLL register value.
#define OMV_OV7725_PLL_CONFIG   (0x41)  // x4

// Sensor Banding Filter Value
#define OMV_OV7725_BANDING      (0x7F)

//Enables the LEPTON CAM
#define OMV_ENABLE_LEPTON

//Enables the Global shutter CAM
//The Global Shutter Camera dosnt work with
//the i.MX RT CSI Peripherie, because needed
//PixelClock (26.66MHz) is not possible to generate
//#define OMV_ENABLE_MT9V034

// Have built-in RGB->LAB table.
#define OMV_HAVE_LAB_TABLE

// Enable remove_shadows()
#define OMV_ENABLE_REMOVE_SHADOWS

// Enable linpolar()
#define OMV_ENABLE_LINPOLAR

// Enable logpolar()
#define OMV_ENABLE_LOGPOLAR

// Enable chrominvar()
#define OMV_ENABLE_CHROMINVAR

// Enable illuminvar()
#define OMV_ENABLE_ILLUMINVAR

// Enable rotation_corr()
#define OMV_ENABLE_ROTATION_CORR

// Enable get_similarity()
#define OMV_ENABLE_GET_SIMILARITY

// Enable find_lines()
#define OMV_ENABLE_FIND_LINES

// Enable find_line_segments()
#define OMV_ENABLE_FIND_LINE_SEGMENTS

// Enable find_circles()
#define OMV_ENABLE_FIND_CIRCLES

// Enable find_rects()
#define OMV_ENABLE_FIND_RECTS

// Enable find_qrcodes() (14 KB)
#define OMV_ENABLE_QRCODES

// Enable find_apriltags() (64 KB)
#define OMV_ENABLE_APRILTAGS

// Enable find_datamatrices() (26 KB)
#define OMV_ENABLE_DATAMATRICES

// Enable find_barcodes() (42 KB)
#define OMV_ENABLE_BARCODES

// Enable find_displacement()
#ifdef OMV_ENABLE_ROTATION_CORR
#define OMV_ENABLE_FIND_DISPLACEMENT
#endif

// Enable LENET (200+ KB).
#define OMV_ENABLE_LENET

// Bootloader LED GPIO port/pin
#define OMV_BOOTLDR_LED_PIN     (GPIO_PIN_1)
#define OMV_BOOTLDR_LED_PORT    (GPIOC)


// If buffer size is bigger than this threshold, the quality is reduced.
// This is only used for JPEG images sent to the IDE not normal compression.
#define JPEG_QUALITY_THRESH     (160*120*2)
// Low and high JPEG QS.
#define JPEG_QUALITY_LOW        50
#define JPEG_QUALITY_HIGH       90

// Linker script constants (see the linker script template stm32fxxx.ld.S).
// Note: fb_alloc is a stack-based, dynamically allocated memory on FB.
// The maximum available fb_alloc memory = FB_ALLOC_SIZE + FB_SIZE - (w*h*bpp).
#define OMV_FB_MEMORY       RAM4   // Framebuffer, fb_alloc
#define OMV_MAIN_MEMORY     CCM     // data, bss, stack and heap
#define OMV_DMA_MEMORY      CCM     // Misc DMA buffers
#define OMV_STACK_SIZE      (8 * 1024)

#define OMV_FB_SIZE         (1000 * 1024)  // FB memory: header + VGA/GS image
#define OMV_FB_ALLOC_SIZE   (100 * 1024)   // minimum fb alloc size
#define OMV_JPEG_BUF_SIZE   (32 * 1024) // IDE JPEG buffer (header + data).

// RAW buffer size
#define OMV_RAW_BUF_SIZE    ((uint32_t)1024000)

#ifndef MCU_SERIES_RT105
#define OMV_LINE_BUF_SIZE   (3K)    // Image line buffer round(640 * 2BPP * 2 buffers).
#define OMV_MSC_BUF_SIZE    (2K)    // USB MSC bot data
#define OMV_VFS_BUF_SIZE    (1K)    // VFS sturct + FATFS file buffer (624 bytes)
#define OMV_FFS_BUF_SIZE    (32K)   // Flash filesystem cache

#define OMV_BOOT_ORIGIN     0x08000000
#define OMV_BOOT_LENGTH     32K
#define OMV_TEXT_ORIGIN     0x08020000
#define OMV_TEXT_LENGTH     1920K
#define OMV_CCM_ORIGIN      0x20000000
#define OMV_CCM_LENGTH      128K    // Note DTCM/ITCM memory is not cacheable on M7
#define OMV_SRAM1_ORIGIN    0x20020000
#define OMV_SRAM1_LENGTH    368K
#define OMV_SRAM2_ORIGIN    0x2007C000
#define OMV_SRAM2_LENGTH    16K

/* SCCB/I2C (Camera Communication) */
extern const pin_obj_t 			pin_AD_B1_00;
extern const pin_obj_t 			pin_AD_B1_01;
#define SCCB_SCL_PINOBJ			pin_AD_B1_00
#define SCCB_SDA_PINOBJ			pin_AD_B1_01
#define SCCB_I2C                (LPI2C1)
#define SCCB_PORT               (SCCB_SCL_PINOBJ.gpio)
#define SCCB_SCL_PIN            (SCCB_SCL_PINOBJ.pin)
#define SCCB_SDA_PIN            (SCCB_SDA_PINOBJ.pin)

/* DCMI */
extern const pin_obj_t 			pin_AD_B1_02;
#define DCMI_RESET_PINOBJ		pin_AD_B1_02
#define DCMI_RESET_PIN          (DCMI_RESET_PINOBJ.pin)
#define DCMI_RESET_PORT         (DCMI_RESET_PINOBJ.gpio)

extern const pin_obj_t 			pin_AD_B1_03;
#define DCMI_PWDN_PINOBJ		pin_AD_B1_03
#define DCMI_PWDN_PIN          	(DCMI_PWDN_PINOBJ.pin)
#define DCMI_PWDN_PORT	        (DCMI_PWDN_PINOBJ.gpio)

#define DCMI_RESET_LOW()        GPIO_PinWrite(DCMI_RESET_PORT, DCMI_RESET_PIN, 0);
#define DCMI_RESET_HIGH()       GPIO_PinWrite(DCMI_RESET_PORT, DCMI_RESET_PIN, 1);

#define DCMI_PWDN_LOW()         GPIO_PinWrite(DCMI_PWDN_PORT, DCMI_PWDN_PIN, 0);
#define DCMI_PWDN_HIGH()        GPIO_PinWrite(DCMI_PWDN_PORT, DCMI_PWDN_PIN, 1);

/* DebugPin ENET_MDIO */
extern const pin_obj_t 			pin_EMC_41;
#define DEBUG_PIN_PINOBJ		pin_EMC_41
#define DEBUG_PIN	            (DEBUG_PIN_PINOBJ.pin)
#define DEBUG_PIN_PORT          (DEBUG_PIN_PINOBJ.gpio)
#define DEBUG_PIN_LOW()         GPIO_PinWrite(DEBUG_PIN_PORT, DEBUG_PIN, 0);
#define DEBUG_PIN_HIGH()        GPIO_PinWrite(DEBUG_PIN_PORT, DEBUG_PIN, 1);

#endif

#endif //__OMV_BOARDCONFIG_H__
