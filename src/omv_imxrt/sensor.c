/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Sensor abstraction layer.
 *
 */

#include <stdlib.h>
#include <string.h>
#include "mp.h"
#include "irq.h"
#include "cambus.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov7725.h"
#include "ov7725_regs.h"
#include "mt9v034.h"
#include "lepton.h"
#include "sensor.h"
#include "systick.h"
#include "framebuffer.h"
#include "fsl_clock.h"
#include "fsl_csi.h"
#include "fsl_debug_console.h"
#include "fsl_camera_receiver.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"
#include "fsl_ov7725.h"
#include "fsl_elcdif.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "omv_boardconfig.h"
#include "fsl_cache.h"

//----- TODO Dave Defines only for testing -------
#define USE_LineBuffer
//-----

#ifndef RAM_CODE
#define RAM_CODE __attribute__((section(".ramfunc.$SRAM_ITC")))
//#define RAM_CODE
#endif

#define OV_CHIP_ID      (0x0A)
#define ON_CHIP_ID      (0x00)
#define MAX_XFER_SIZE (0xFFFC)

#define NO_LCD_MONITOR
//#define OV7725_I2C LPI2C1

/* LCD definition. */
#define APP_ELCDIF LCDIF

#define APP_LCD_HEIGHT 272
#define APP_LCD_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_LCD_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 2
/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN 31

#define APP_BPP 2
/* Camera definition. */
#define APP_CAMERA_HEIGHT 240
#define APP_CAMERA_WIDTH 320
#define APP_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)
#define APP_FRAME_BUFFER_COUNT 4
#define FRAME_BUFFER_ALIGN 64
sensor_t s_sensor;

/*static*/ volatile uint8_t s_isOmvSensorSnapshotReady;

/*******************************************************************************
 * Variables
 ******************************************************************************/
const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
};
/* OV7725 connect to CSI. */
static csi_resource_t csiResource = {
    .csiBase = CSI,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t cameraReceiver = {
    .resource = &csiResource, .ops = &csi_ops, .privateData = &csiPrivateData,
};

#if (OMV_XCLK_SOURCE == OMV_XCLK_TIM)

static void  setPixelClock(int framerate)
{
		CCM->CSCDR3 = framerate & (0x1F<<9);
}

static int extclk_config(int frequency)
{
	//IMXRT1050RM.pdf p.1036
	switch(frequency)
	{
	case 12000000:
		/* Set PixelClock. */
		//0<<9 = Derive Clock from OSC_CLK:24Mhz
		//(2-1)<<11 = divide by 2
		setPixelClock(0x80000000 | (0<<9|(2-1)<<11)); //12Mhz PixClock
		break;
	case 15000000:
		/* Set PixelClock. */
		//2<<9 = Derive Clock from USBPLL:480Mhz/4=120Mhz
		//(8-1)<<11 = divide by 8
		setPixelClock(0x80000000 | (2<<9|(8-1)<<11)); //15Mhz PixClock
		break;
	case 24000000:
		/* Set PixelClock. */
		//0<<9 = Derive Clock from OSC_CLK:24Mhz
		//(0)<<11 = divide by 1
		setPixelClock(0x80000000 | (0<<9|0<<11)); //24Mhz PixClock
		break;
	default://Not Supported PixelClockFreq! Using 12MHz
		setPixelClock(0x80000000 | (0<<9|(2-1)<<11)); //12Mhz PixClock
		break;
	}
    return 0;
}
#endif // (OMV_XCLK_SOURCE == OMV_XCLK_TIM)

void sensor_gpio_init(void)
{
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };
    GPIO_PinInit(DCMI_RESET_PORT, DCMI_RESET_PIN, &config);
	GPIO_PinInit(DCMI_PWDN_PORT, DCMI_PWDN_PIN, &config);

	//DEBUG PIN EMC_41 (Name:ENET_MDIO)
	GPIO_PinInit(DEBUG_PIN_PORT, DEBUG_PIN, &config);
	DEBUG_PIN_LOW();
}

void sensor_init0()
{
    // Init FB mutex
    mutex_init(&JPEG_FB()->lock);

    // Save fb_enabled flag state
    int fb_enabled = JPEG_FB()->enabled;

    // Clear framebuffers
    memset(MAIN_FB(), 0, sizeof(*MAIN_FB()));
    memset(JPEG_FB(), 0, sizeof(*JPEG_FB()));

    // Set default quality
    JPEG_FB()->quality = ((JPEG_QUALITY_HIGH - JPEG_QUALITY_LOW) / 2) + JPEG_QUALITY_LOW;
    //JPEG_FB()->quality = 35;

    // Set fb_enabled
    JPEG_FB()->enabled = fb_enabled;
}
uint32_t activeFrameAddr;
uint32_t inactiveFrameAddr;

CSI_Type *s_pCSI = CSI;

//				 			8bit | PixRisEdge | gatedClk  | SyncClrFifo| HSyncActHigh|SofOnVsyncRis|ExtVSync
#define CSICR1_INIT_VAL 	0<<0 | 1<<1	      | 1<<4	  | 1<<8	   | 1<<11		 | 1<<17	   |1<<30   


//TODO Dave: Bug in Software if buffer is enlarged... (Frame has some cutts every #Fragmet Lines
//Dave: This buffer is in DTC Memo--> Fast
uint64_t s_dmaFragBufs[2][640 * 2 / 8];	// max supported line length = 1280Byte = 640pixel in RGB

typedef struct _CSIIrq_t
{
	uint8_t isStarted;
	uint8_t isGray;
	uint32_t base0;
	uint32_t linePerFrag;
	uint32_t cnt;
	uint32_t dmaBytePerLine;
	uint32_t dmaBytePerFrag;
	uint32_t dmaFragNdx;

	uint32_t datBytePerLine;
	uint32_t datBytePerFrag;
	uint32_t datFragNdx;

	uint32_t datCurBase;

	uint32_t fragCnt;
	// in color mode, dmaFragNdx should == datLineNdx
	// in gray mode, to save memory, move backword nextDmaBulk every 4 lines
	
}CSIIrq_t;
volatile CSIIrq_t s_irq;

typedef union {
	uint8_t u8Ary[4][2];
	struct {
		uint8_t y0, u0, y1, v0, y2, u2, y3, v2;
	};
	
}YUV64bit_t;

__attribute__((naked))
RAM_CODE uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	__asm volatile (
		"	push	{r1-r7, ip, lr}  \n "
		"10:  \n "
		"	ldmia	r0!, {r3-r6}  \n "
			// schedule code carefully to allow dual-issue on Cortex-M7
		"	bfi		r7, r3, #0, #8  \n "	// Y0
		"	bfi		ip, r5, #0, #8  \n "	// Y4
		"	lsr		r3,	r3,	#16  \n "
		"	lsr		r5,	r5,	#16  \n "
		"	bfi		r7, r3, #8, #8  \n "	// Y1
		"	bfi		ip, r5, #8, #8  \n "  // Y5
		"	bfi		r7, r4, #16, #8  \n " // Y2
		"	bfi		ip, r6, #16, #8  \n " // Y6
		"	lsr		r4,	r4,	#16  \n "
		"	lsr		r6,	r6,	#16  \n "
		"	bfi		r7, r4, #24, #8  \n " // Y3
		"	bfi		ip, r6, #24, #8  \n "	// Y7
		"	stmia	r1!, {r7, ip}  \n "	
		"	subs	r2,	#1  \n "
		"	bne		10b  \n "
		"	mov		r0,	r1  \n "
		"	pop		{r1-r7, ip, pc}  \n "		
	);
}

RAM_CODE void CSI_IRQHandler(void) {
    uint32_t csisr = s_pCSI->CSISR;
    /* Clear the error flags. */
    s_pCSI->CSISR = csisr;

	if (csisr & (1<<16)) //Start Of Frame Interrupt
	{
		// VSync
		//               SOF    | FB1    | FB2    irqEn
		s_pCSI->CSICR1 = 1U<<16 | 1U<<19 | 1U<<20 | CSICR1_INIT_VAL;
		//				 16 doubleWords| RxFifoDmaReqEn| ReflashRFF|ResetFrmCnt
		s_pCSI->CSICR3 = 2<<4          | 1<<12         | 1<<14     |1<<15;
	}
	else if (csisr & (3<<19)) //DMA Transfer Done Interrupt,  either in FB1 or FB2
	{
		uint32_t dmaBase, lineNdx = s_irq.dmaFragNdx * s_irq.linePerFrag;

		//Switch between the two LineBuffers
		if (s_irq.dmaFragNdx & 1)
			dmaBase = s_pCSI->CSIDMASA_FB2;
		else
			dmaBase = s_pCSI->CSIDMASA_FB1;

		if (dmaBase >= 0x20200000) //LineBuffer not in Unchached region?
			DCACHE_CleanInvalidateByRange(dmaBase, s_irq.dmaBytePerFrag);

		if (s_irq.isGray || 
			(s_sensor.isWindowing &&  lineNdx >= s_sensor.wndY && lineNdx - s_sensor.wndY <= s_sensor.wndH) )
		{
			dmaBase += s_sensor.wndX * 2 * s_irq.linePerFrag;	// apply line window offset
			if (s_irq.isGray) {
				s_irq.datCurBase = ExtractYFromYuv(dmaBase, s_irq.datCurBase, (s_sensor.wndW * s_irq.linePerFrag) >> 3);
			}
			else {
				uint32_t byteToCopy = (s_sensor.wndW * s_irq.linePerFrag) << 1;
				memcpy((void*)s_irq.datCurBase, (void*)dmaBase, byteToCopy);
				s_irq.datCurBase += byteToCopy;
			}
		}
#ifdef USE_LineBuffer
		else if (!s_sensor.isWindowing)
		{
			uint32_t byteToCopy = (s_irq.dmaBytePerFrag);
			memcpy((void*)s_irq.datCurBase, (void*)dmaBase, byteToCopy);
			s_irq.datCurBase += byteToCopy;
		}
#endif //USE_LineBuffer

		//Last Fragment in Frame or Both DMA FBs are Full:
		if (++s_irq.dmaFragNdx == s_irq.fragCnt || (csisr & (3<<19)) == 3<<19 )
		{
			CSI_Stop(CSI);
			//				 16 doubleWords| ReflashRFF
			s_pCSI->CSICR3 = 2<<4		   | 1<<14;
			NVIC_DisableIRQ(CSI_IRQn);
			s_isOmvSensorSnapshotReady = 1;	
			goto Cleanup;
		}
		

		if (csisr & (1<<19) ) {
#ifndef USE_LineBuffer
			if (!s_irq.isGray && !s_sensor.isWindowing)
				s_pCSI->CSIDMASA_FB1 += 2 * s_irq.dmaBytePerFrag;
#endif //#ifdef USE_LineBuffer
		} else {
#ifndef USE_LineBuffer
			if (!s_irq.isGray && !s_sensor.isWindowing)
				s_pCSI->CSIDMASA_FB2 += 2 * s_irq.dmaBytePerFrag;
#endif //USE_LineBuffer
			s_pCSI->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;	// reload DMA
		}
	}
Cleanup:
	return;
}

void CsiFragModeInit(void) {

	CLOCK_EnableClock(kCLOCK_Csi);
	CSI_Reset(CSI);
	
	s_pCSI->CSICR1 = CSICR1_INIT_VAL;
	s_pCSI->CSICR2 = 3U << 30;	// INCR16 for RxFIFO DMA
	s_pCSI->CSICR3 = 2U << 4;	// 16 double words to trigger DMA request
	s_pCSI->CSIFBUF_PARA = 0;	// no stride


	s_pCSI->CSICR18 = 13<<12 | 1<<18;	// HProt AHB bus protocol, write to memory when CSI_ENABLE is 1

	NVIC_SetPriority(CSI_IRQn, IRQ_PRI_CSI);
}

//Calculates how many lines are read by one DMA Transfer
void CsiFragModeCalc(void) {
	s_irq.datBytePerLine = s_irq.dmaBytePerLine = s_sensor.fb_w * 2;
	if (s_sensor.pixformat == PIXFORMAT_GRAYSCALE) {
		s_irq.datBytePerLine /= 2;	// only contain Y
		s_irq.isGray = 1;
		s_sensor.gs_bpp = 1;
	} else {
		s_irq.isGray = 0;
		s_sensor.gs_bpp = 2;
	}
	if (s_sensor.fb_w == 0 || s_sensor.fb_h == 0)
		return;

	// calculate max bytes per DMA frag (How many Lines are read by one DMA Tranfer)
	uint32_t dmaBytePerFrag, byteStep, dmaByteTotal;
	uint32_t maxBytePerLine = sizeof(s_dmaFragBufs) / ARRAY_SIZE(s_dmaFragBufs);
	dmaByteTotal = s_sensor.fb_w * s_sensor.fb_h * 2;	
	if (s_sensor.wndX == 0 && s_sensor.wndY == 0) // No Windowing mode
	{
		dmaBytePerFrag = s_irq.dmaBytePerLine;  // set a minimal default value
		for (byteStep = s_irq.dmaBytePerLine; byteStep < maxBytePerLine; byteStep += s_irq.dmaBytePerLine) {
			if (0 == byteStep % 32 )
			{
				// find maximum allowed bytes per frag
				dmaBytePerFrag = (maxBytePerLine / byteStep) * byteStep;
				for (; dmaBytePerFrag >= byteStep; dmaBytePerFrag -= byteStep) {
					if (dmaByteTotal % dmaBytePerFrag == 0)
						break;
				}
				if (dmaBytePerFrag < byteStep) {
					dmaBytePerFrag = byteStep;
					while (1) {}
				}
				break;
			}
		}
	} 
	else {
		// for window mode, we only accept 1 line per frag
		dmaBytePerFrag = s_irq.dmaBytePerLine;
	}
	s_irq.linePerFrag = dmaBytePerFrag / s_irq.dmaBytePerLine;
	s_irq.dmaBytePerFrag = dmaBytePerFrag;
	s_irq.datBytePerLine = s_irq.isGray ? dmaBytePerFrag / 2 : dmaBytePerFrag;

	// >>> calculate how many lines per fragment (DMA xfer unit)
	uint32_t burstBytes;
	if (!(s_irq.dmaBytePerLine % (8 * 16)))
	{
		burstBytes = 128;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else if (!(s_irq.dmaBytePerLine % (8 * 8)))
	{
		burstBytes = 64;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(2U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((1U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else
	{
		burstBytes = 32;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(1U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((0U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	s_irq.fragCnt = s_sensor.fb_h / s_irq.linePerFrag;
	// <<<
}

void CsiFragModeStartNewFrame(void) {
	CsiFragModeCalc();
	s_irq.dmaFragNdx = 0;
	s_irq.cnt++;
	// DMA also writes to this cache line, to avoid being invalidated, clean MAIN_FB header.
#ifndef USE_LineBuffer
	//DCACHE_CleanByRange((uint32_t)MAIN_FB(), 32);
#endif //USE_LineBuffer

#ifdef USE_LineBuffer
	s_pCSI->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
	s_pCSI->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
#else
	if (s_irq.isGray || s_sensor.isWindowing) {
		s_pCSI->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
		s_pCSI->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
	} else {
		s_pCSI->CSIDMASA_FB1 = s_irq.base0;
		s_pCSI->CSIDMASA_FB2 = s_irq.base0 + s_irq.dmaBytePerFrag;
	}
#endif //USE_LineBuffer
	s_irq.datCurBase = s_irq.base0; // + s_sensor.wndY * s_irq.datBytePerLine + s_sensor.wndX * s_sensor.gs_bpp;
	s_pCSI->CSICR1 = CSICR1_INIT_VAL | 1<<16;	// enable SOF iRQ
	if (s_irq.dmaBytePerFrag & 0xFFFF0000) {
		
		uint32_t l16 = s_irq.linePerFrag , h16 = s_irq.dmaBytePerLine << 16;
		s_pCSI->CSIIMAG_PARA = l16 | h16;
	} else {
		s_pCSI->CSIIMAG_PARA = 1U | s_irq.dmaBytePerFrag << 16;	// CSI_IRQ every <dmaBytePerFrag>
	}
	__set_PRIMASK(1);
	s_pCSI->CSISR = s_pCSI->CSISR;
	s_pCSI->CSICR18 |= 1U<<31;	// start CSI
	NVIC_EnableIRQ(CSI_IRQn);
	__set_PRIMASK(0);	
}
#define CAMERA_TAKE_SNAPSHOT() CsiFragModeStartNewFrame()

#define CAMERA_WAIT_FOR_SNAPSHOT() do { \
	while (0 == s_isOmvSensorSnapshotReady) {} \
	s_isOmvSensorSnapshotReady = 0; \
	}while(0)

int sensor_init()
{   
	int init_ret = 0;

    /* Do a power cycle */
	sensor_gpio_init();
    DCMI_PWDN_HIGH();
    systick_sleep(10);
    DCMI_PWDN_LOW();
    systick_sleep(10);

	// Initialize the camera bus.
    cambus_init();

    /* Reset the sesnor state */
	memset(&s_sensor, 0, sizeof(s_sensor));
	s_irq.base0 = (uint32_t)(MAIN_FB()->pixels);


    /* Some sensors have different reset polarities, and we can't know which sensor
       is connected before initializing cambus and probing the sensor, which in turn
       requires pulling the sensor out of the reset state. So we try to probe the
       sensor with both polarities to determine line state. */
    s_sensor.pwdn_pol = ACTIVE_HIGH;
    s_sensor.reset_pol = ACTIVE_HIGH;

    /* Reset the sensor */
    DCMI_RESET_HIGH();
    systick_sleep(10);

    DCMI_RESET_LOW();
    systick_sleep(10);

    /* Probe the sensor */
    s_sensor.slv_addr = cambus_scan();
    if (s_sensor.slv_addr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active low */
    	s_sensor.reset_pol = ACTIVE_LOW;

        /* Pull the sensor out of the reset state */
        DCMI_RESET_HIGH();
        systick_sleep(10);

        /* Probe again to set the slave addr */
        s_sensor.slv_addr = cambus_scan();
        if (s_sensor.slv_addr == 0) {
        	s_sensor.pwdn_pol = ACTIVE_LOW;

            DCMI_PWDN_HIGH();
            systick_sleep(10);

            s_sensor.slv_addr = cambus_scan();
            if (s_sensor.slv_addr == 0) {
            	s_sensor.reset_pol = ACTIVE_HIGH;

                DCMI_RESET_LOW();
                systick_sleep(10);

                s_sensor.slv_addr = cambus_scan();
                if (s_sensor.slv_addr == 0) {
                    return -2;
                }
            }
        }
    }

    // Clear sensor chip ID.
    s_sensor.chip_id = 0;

    // Set default snapshot function.
    s_sensor.snapshot = sensor_snapshot;

    switch (s_sensor.slv_addr) {
    case OV7725_SLV_ADDR:
        cambus_readb(s_sensor.slv_addr, OV_CHIP_ID, &s_sensor.chip_id);
        break;
    case OV2640_SLV_ADDR:
        cambus_readb(s_sensor.slv_addr, OV_CHIP_ID, &s_sensor.chip_id);
        break;
    case MT9V034_SLV_ADDR:
        cambus_readb(s_sensor.slv_addr, ON_CHIP_ID, &s_sensor.chip_id);
        break;
    case LEPTON_SLV_ADDR:
    	s_sensor.chip_id = LEPTON_ID;
        break;
    case OV5640_SLV_ADDR:
        cambus_readb2(s_sensor.slv_addr, OV5640_CHIP_ID, &s_sensor.chip_id);
        break;
    default:
        return -3;
        break;
    }

    /* Default Pixeclock */
    extclk_config(OMV_XCLK_FREQUENCY);

    systick_sleep(10);
    switch (s_sensor.chip_id)
    {
    case OV7725_ID:
        init_ret = ov7725_init(&s_sensor);
        break;
    case MT9V034_ID:
        if (extclk_config(MT9V034_XCLK_FREQ) != 0) {
            return -3;
        }
        init_ret = mt9v034_init(&s_sensor);
        break;
    case LEPTON_ID:
        if (extclk_config(LEPTON_XCLK_FREQ) != 0) {
            return -3;
        }
        init_ret = lepton_init(&s_sensor);
        break;
    case OV5640_ID:
        init_ret = ov5640_init(&s_sensor);
        break;
    case OV2640_ID:
        init_ret = ov2640_init(&s_sensor);
        break;
    case OV9650_ID:
        init_ret = ov9650_init(&s_sensor);
        break;
    default:
        return -3;
        break;
    }

    if (init_ret != 0 ) {
        // Sensor init failed.
        return -4;
    }

	CsiFragModeInit();

	s_sensor.isWindowing = 0;
	s_sensor.wndH = s_sensor.fb_h;
	s_sensor.wndW = s_sensor.fb_w;
	s_sensor.wndX = s_sensor.wndY = 0;

    // Clear fb_enabled flag
    // This is executed only once to initialize the FB enabled flag.
	JPEG_FB()->enabled = 0;

    // Set default color palette.
	s_sensor.color_palette = rainbow_table;

    return 0;
}

int sensor_reset()
{
    // Reset the sesnor state
	s_sensor.sde         = 0;
	s_sensor.pixformat   = 0;
	s_sensor.framesize   = 0;
	s_sensor.framerate   = 0;
	s_sensor.gainceiling = 0;
	//s_sensor.vsync_gpio  = NULL;

    // Reset default color palette.
	s_sensor.color_palette = rainbow_table;

    // Restore shutdown state on reset.
    sensor_shutdown(false);

    // Call sensor-specific reset function
    if (s_sensor.reset(&s_sensor) != 0) {
        return -1;
    }

    return 0;
}

int sensor_get_id()
{
    return s_sensor.chip_id;
}

int sensor_set_vsync_output(GPIO_Type *gpio, uint32_t pin)
{
    // s_sensor.vsync_pin  = pin;
    // s_sensor.vsync_gpio = gpio;
    return 0;
}

int sensor_sleep(int enable)
{
    if (s_sensor.sleep == NULL
        || s_sensor.sleep(&s_sensor, enable) != 0) {
        // Operation not supported
        return -1;
    }
    return 0;
}

int sensor_shutdown(int enable)
{
    if (enable) {
        DCMI_PWDN_HIGH();
    } else {
        DCMI_PWDN_LOW();
    }

    systick_sleep(10);
    return 0;
}

int sensor_read_reg(uint8_t reg_addr)
{
    if (s_sensor.read_reg == NULL) {
        // Operation not supported
        return -1;
    }
    return s_sensor.read_reg(&s_sensor, reg_addr);
}

int sensor_write_reg(uint8_t reg_addr, uint16_t reg_data)
{
    if (s_sensor.write_reg == NULL) {
        // Operation not supported
        return -1;
    }
    return s_sensor.write_reg(&s_sensor, reg_addr, reg_data);
}

int sensor_set_pixformat(pixformat_t pixformat)
{

    if (s_sensor.pixformat == pixformat) {
        // No change
        return 0;
    }

    if (s_sensor.set_pixformat == NULL
        || s_sensor.set_pixformat(&s_sensor, pixformat) != 0) {
        // Operation not supported
        return -1;
    }

    // Set pixel format
    s_sensor.pixformat = pixformat;

    // JPEG images from the Camera are not supported...
    if (pixformat == PIXFORMAT_JPEG) {
        return -1;
    }

    // Skip the first frame.
    MAIN_FB()->bpp = 0;
    return 0;
}

int sensor_set_framesize(framesize_t framesize)
{

    // Call the sensor specific function
    if (s_sensor.set_framesize == NULL
        || s_sensor.set_framesize(&s_sensor, framesize) != 0) {
        // Operation not supported
        return -1;
    }

    // Set framebuffer size
    s_sensor.framesize = framesize;

    // Skip the first frame.
    MAIN_FB()->bpp = -1;
    // Set MAIN FB width and height.
    s_sensor.fb_w = MAIN_FB()->w = resolution[framesize][0];
    s_sensor.fb_h = MAIN_FB()->h = resolution[framesize][1];

    // Set MAIN FB backup width and height.
    MAIN_FB()->u = resolution[framesize][0];
    MAIN_FB()->v = resolution[framesize][1];
	s_sensor.wndX = 0; s_sensor.wndY = 0 ; s_sensor.wndW = s_sensor.fb_w ; s_sensor.wndH = s_sensor.fb_h;
	// CsiFragModeCalc();
    return 0;
}

int sensor_set_framerate(framerate_t framerate)
{
    if (s_sensor.framerate == framerate) {
       /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (s_sensor.set_framerate == NULL
        || s_sensor.set_framerate(&s_sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }

    /* set the frame rate */
    s_sensor.framerate = framerate;

    return 0;
}

//The CSI Hardware of the i.MX RT does not Support windowing.
//This implementation saves just the pixels inside the window but all
//pixels are sensed...
int sensor_set_windowing(int x, int y, int w, int h)
{
	w = (w + 7) & ~7 , x = (x + 7) & ~7;
	if (x >= s_sensor.fb_w - 8)
		x = s_sensor.fb_w - 8;
	if (y >= s_sensor.fb_h - 1)
		y = s_sensor.fb_h - 1;
	if (x + w > s_sensor.fb_w)
		w = s_sensor.fb_w - x;
	if (y + h > s_sensor.fb_h)
		h = s_sensor.fb_h - y;

	s_sensor.isWindowing = (w < s_sensor.fb_w && h < s_sensor.fb_h) ? 1 : 0;
	s_sensor.wndX = x ; s_sensor.wndY = y ; s_sensor.wndW = w ; s_sensor.wndH = h;
    MAIN_FB()->w = w;
    MAIN_FB()->h = h;
    return 0;
}

int sensor_set_contrast(int level)
{
    if (s_sensor.set_contrast != NULL) {
        return s_sensor.set_contrast(&s_sensor, level);
    }
    return -1;
}

int sensor_set_brightness(int level)
{
    if (s_sensor.set_brightness != NULL) {
        return s_sensor.set_brightness(&s_sensor, level);
    }
    return -1;
}

int sensor_set_saturation(int level)
{
    if (s_sensor.set_saturation != NULL) {
        return s_sensor.set_saturation(&s_sensor, level);
    }
    return -1;
}

int sensor_set_gainceiling(gainceiling_t gainceiling)
{
    if (s_sensor.gainceiling == gainceiling) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (s_sensor.set_gainceiling == NULL
        || s_sensor.set_gainceiling(&s_sensor, gainceiling) != 0) {
        /* operation not supported */
        return -1;
    }

    s_sensor.gainceiling = gainceiling;
    return 0;
}

int sensor_set_quality(int qs)
{
    /* call the sensor specific function */
    if (s_sensor.set_quality == NULL
        || s_sensor.set_quality(&s_sensor, qs) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_colorbar(int enable)
{
    /* call the sensor specific function */
    if (s_sensor.set_colorbar == NULL
        || s_sensor.set_colorbar(&s_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    /* call the sensor specific function */
    if (s_sensor.set_auto_gain == NULL
        || s_sensor.set_auto_gain(&s_sensor, enable, gain_db, gain_db_ceiling) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_gain_db(float *gain_db)
{
    /* call the sensor specific function */
    if (s_sensor.get_gain_db == NULL
        || s_sensor.get_gain_db(&s_sensor, gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_exposure(int enable, int exposure_us)
{
    /* call the sensor specific function */
    if (s_sensor.set_auto_exposure == NULL
        || s_sensor.set_auto_exposure(&s_sensor, enable, exposure_us) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_exposure_us(int *exposure_us)
{
    /* call the sensor specific function */
    if (s_sensor.get_exposure_us == NULL
        || s_sensor.get_exposure_us(&s_sensor, exposure_us) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    /* call the sensor specific function */
    if (s_sensor.set_auto_whitebal == NULL
        || s_sensor.set_auto_whitebal(&s_sensor, enable, r_gain_db, g_gain_db, b_gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    /* call the sensor specific function */
    if (s_sensor.get_rgb_gain_db == NULL
        || s_sensor.get_rgb_gain_db(&s_sensor, r_gain_db, g_gain_db, b_gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_hmirror(int enable)
{
    /* call the sensor specific function */
    if (s_sensor.set_hmirror == NULL
        || s_sensor.set_hmirror(&s_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_vflip(int enable)
{
    /* call the sensor specific function */
    if (s_sensor.set_vflip == NULL
        || s_sensor.set_vflip(&s_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_special_effect(sde_t sde)
{
    if (s_sensor.sde == sde) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (s_sensor.set_special_effect == NULL
        || s_sensor.set_special_effect(&s_sensor, sde) != 0) {
        /* operation not supported */
        return -1;
    }

    s_sensor.sde = sde;
    return 0;
}

int sensor_set_lens_correction(int enable, int radi, int coef)
{
    /* call the sensor specific function */
    if (s_sensor.set_lens_correction == NULL
        || s_sensor.set_lens_correction(&s_sensor, enable, radi, coef) != 0) {
        /* operation not supported */
        return -1;
    }

    return 0;
}

int sensor_ioctl(int request, ... /* arg */)
{
    int ret = -1;
    if (s_sensor.ioctl != NULL) {
        va_list ap;
        va_start(ap, request);
        /* call the sensor specific function */
        ret = s_sensor.ioctl(&s_sensor, request, ap);
        va_end(ap);
    }
    return ret;
}
int sensor_set_color_palette(const uint16_t *color_palette)
{
    s_sensor.color_palette = color_palette;
    return 0;
}

const uint16_t *sensor_get_color_palette()
{
    return s_sensor.color_palette;
}

static void sensor_check_bufsize()
{
    int bpp=0;
    switch (s_sensor.pixformat) {
        case PIXFORMAT_BAYER:
        case PIXFORMAT_GRAYSCALE:
            bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            bpp = 2;
            break;
        default:
            break;
    }

    if ((MAIN_FB()->w * MAIN_FB()->h * bpp) > (OMV_RAW_BUF_SIZE)) {
        if (s_sensor.pixformat == PIXFORMAT_GRAYSCALE) {
            // Crop higher GS resolutions to QVGA
            sensor_set_windowing(190, 120, 320, 240);
        } else if (s_sensor.pixformat == PIXFORMAT_RGB565) {
            // Switch to BAYER if the frame is too big to fit in RAM.
            sensor_set_pixformat(PIXFORMAT_BAYER);
        }
    }

}

// The JPEG offset allows JPEG compression of the framebuffer without overwriting the pixels.
// The offset size may need to be adjusted depending on the quality, otherwise JPEG data may
// overwrite image pixels before they are compressed.
int sensor_snapshot(sensor_t *sensor, image_t *pImg, streaming_cb_t streaming_cb)
{
	DEBUG_PIN_HIGH();
  	sensor = sensor , streaming_cb = streaming_cb;	// keep compatible with original openMV
    sensor_check_bufsize();

    switch (s_sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            MAIN_FB()->bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            MAIN_FB()->bpp = 2;
            break;
        case PIXFORMAT_BAYER:
            MAIN_FB()->bpp = 3;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            // MAIN_FB()->bpp = (MAX_XFER_SIZE - __HAL_DMA_GET_COUNTER(&DMAHandle))*4;
            break;
    }

	static uint8_t n;

	if (JPEG_FB()->enabled) {
		fb_update_jpeg_buffer();
	}
	//DEBUG_PIN_LOW();


	CAMERA_TAKE_SNAPSHOT();
	systick_sleep(5); //Time to Transfer the JPEG image to the IDE
	NVIC_DisableIRQ(USB_OTG1_IRQn);
	CAMERA_WAIT_FOR_SNAPSHOT();
	NVIC_EnableIRQ(USB_OTG1_IRQn);

	n++;

	if (pImg) {
		pImg->w = MAIN_FB()->w , pImg->h = MAIN_FB()->h , pImg->bpp = MAIN_FB()->bpp;
		pImg->pixels = (uint8_t*) MAIN_FB()->pixels;		
	}
	DEBUG_PIN_LOW();
    return 0;
}
