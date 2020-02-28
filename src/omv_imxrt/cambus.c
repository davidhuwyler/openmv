/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include "fsl_lpi2c.h"
#include "pin_mux.h"
#include "fsl_csi.h"
#include "fsl_gpio.h"
#include "clock_config.h"
#include <systick.h>
#include "omv_boardconfig.h"
#include "cambus.h"

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (1U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (0U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY

#define I2C_MASTER_BASE (SCCB_I2C)
#define I2C_MASTER ((LPI2C_Type *)I2C_MASTER_BASE)

#define LPI2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define LPI2C_BAUDRATE 100000U
#define LPI2C_DATA_LENGTH 2U

int cambus_init()
{
    uint32_t sourceClock;
       /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);
        /* LPI2C clock is OSC clock. */
    sourceClock = CLOCK_GetOscFreq();

    lpi2c_master_config_t masterConfig;
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;
    LPI2C_MasterInit(I2C_MASTER, &masterConfig, sourceClock);
  /* CSI MCLK select 24M. */
    /*
     * CSI clock source:
     *
     * 00 derive clock from osc_clk (24M)
     * 01 derive clock from PLL2 PFD2
     * 10 derive clock from pll3_120M
     * 11 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);
    /*
     * CSI clock divider:
     *
     * 000 divide by 1
     * 001 divide by 2
     * 010 divide by 3
     * 011 divide by 4
     * 100 divide by 5
     * 101 divide by 6
     * 110 divide by 7
     * 111 divide by 8
     */
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);

    /*
     * For RT1050, there is not dedicate clock gate for CSI MCLK, it use CSI
     * clock gate.
     */

    /* Set the pins for CSI reset and power down. */
    gpio_pin_config_t pinConfig = {
        kGPIO_DigitalOutput, 1,
    };

    GPIO_PinInit(GPIO1, 4, &pinConfig);
    return 0;
}


int cambus_scan()
{
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
        byte dummyBuffer;
        status_t reVal = kStatus_Fail;
        reVal = LPI2C_MasterStart(I2C_MASTER, addr, kLPI2C_Read);
        reVal = LPI2C_MasterReceive(I2C_MASTER, &dummyBuffer, 1);

        if(reVal!=kStatus_LPI2C_Nak)
        {
        	LPI2C_MasterStop(I2C_MASTER);
            return (addr << 1);
        }
    }
    return 0;
}

int cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)   
{
    status_t reVal = kStatus_Fail;
    while (1)
    {
        reVal = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write); 

        if (kStatus_Success != reVal)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    LPI2C_MasterSend(I2C_MASTER, &reg_addr, 1);
    LPI2C_MasterStop(I2C_MASTER);
    LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Read);
    LPI2C_MasterReceive(I2C_MASTER, reg_data, 1);
    LPI2C_MasterStop(I2C_MASTER);
    return 0; 
}

int cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)     
{
    uint8_t data[3];
    uint8_t size = 0;
    data[size++] = (uint8_t)reg_addr;
    data[size++] = reg_data;
    status_t status;
    while(1)
    {
	status = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write);

        if (kStatus_Success != status)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    LPI2C_MasterSend(I2C_MASTER, data, size);    
    LPI2C_MasterStop(I2C_MASTER);
    return 0;
}

int cambus_readw(uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    status_t reVal = kStatus_Fail;
    while (1)
    {
        reVal = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write);

        if (kStatus_Success != reVal)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    reVal = LPI2C_MasterSend(I2C_MASTER, &reg_addr, 1);
    reVal = LPI2C_MasterStop(I2C_MASTER);
    reVal = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Read);
    reVal = LPI2C_MasterReceive(I2C_MASTER, reg_data, 2);
    reVal = LPI2C_MasterStop(I2C_MASTER);
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);

    if (reVal == kStatus_Success)
    {
        return 0;
    }
    else
    {
        return -1;
    }    
}

int cambus_writew(uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
   uint8_t data[3];
    uint8_t size = 0;
    data[size++] = (uint8_t)reg_addr;
    data[size++] = (uint8_t)reg_data >> 8;
    data[size++] = (uint8_t)reg_data;
    status_t status;
    while(1)
    {
	status = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write);

        if (kStatus_Success != status)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    LPI2C_MasterSend(I2C_MASTER, data, size);    
    LPI2C_MasterStop(I2C_MASTER);
    return 0;
}


int cambus_readb2(uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data)
{
    for(;;); //not implemented jet
    return 0;
}

int cambus_writeb2(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data)
{
    for(;;); //not implemented jet
    return 0;
}

int cambus_readw2(uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
    status_t reVal = kStatus_Fail;
    byte buffer[2];
    buffer[0] = (uint8_t)(reg_addr>>8);
    buffer[1] = (uint8_t)(reg_addr);

    while (1)
    {
        reVal = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write);

        if (kStatus_Success != reVal)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    reVal = LPI2C_MasterSend(I2C_MASTER, buffer, 2);
    reVal = LPI2C_MasterStop(I2C_MASTER);
    reVal = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Read);
    reVal = LPI2C_MasterReceive(I2C_MASTER, reg_data, 2);
    reVal = LPI2C_MasterStop(I2C_MASTER);
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);

    if (reVal == kStatus_Success)
    {
        return 0;
    }
    else
    {
        return -1;
    }    
}

int cambus_writew2(uint8_t slv_addr, uint16_t reg_addr, uint16_t reg_data)
{
    uint8_t data[4];
    uint8_t size = 0;
    data[size++] = (uint8_t)(reg_addr >> 8);
    data[size++] = (uint8_t)reg_addr;
    data[size++] = (uint8_t)(reg_data >> 8);
    data[size++] = (uint8_t)reg_data;
    status_t status;
    while(1)
    {
	status = LPI2C_MasterStart(I2C_MASTER, slv_addr>>1, kLPI2C_Write);

        if (kStatus_Success != status)
        {
            LPI2C_MasterStop(I2C_MASTER);
	    return -1;
        }
        else
        {
            break;
        }
    }
    LPI2C_MasterSend(I2C_MASTER, data, size);    
    LPI2C_MasterStop(I2C_MASTER);
    return 0;
}
