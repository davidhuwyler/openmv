#include <stdint.h>
#include "flash.h"
//#include "usbdev/usbd_cdc.h"
#include "omv_boardconfig.h"
#include "usbd_cdc_interface.h"

#define APP_RX_DATA_SIZE    (2048)
#define APP_TX_DATA_SIZE    (2048)

//---------------------------------
//ST HAL DEFINITIONS:
#define USBD_OK 0
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23
//---------------------------------

// USBD_CDC_LineCodingTypeDef LineCoding =
// {
//     115200, /* baud rate*/
//     0x00,   /* stop bits-1*/
//     0x00,   /* parity - none*/
//     0x08    /* nb. of bits 8*/
// };

uint32_t BuffLength;
uint32_t UserTxBufPtrIn = 0;    /* Increment this pointer or roll it back to
                                   start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0;   /* Increment this pointer or roll it back to
                                   start address when data are sent over USB */
uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

static volatile uint8_t ide_connected = 0;
static volatile uint8_t vcp_connected = 0;

#define FLASH_BUF_SIZE  (64)
static volatile uint32_t flash_buf_idx=0;
static volatile uint8_t  flash_buf[FLASH_BUF_SIZE];

// Flash sectors for the bootloader.
// Flash FS sector, main FW sector, max sector.
// Sector (Blocks in iMX RT) 0..4 == Bootloader   0x60'000'000 .. 0x60'100'000 (1M)
// Sector (Blocks in iMX RT) 5..31 == Applicaton  0x60'100'000 .. 0x60'800'000 (8M)
static const    uint32_t flash_layout[3] = {31, 5, 31};
static const    uint32_t bootloader_version = 0xABCD0002;

/* USB handler declaration */
//extern USBD_HandleTypeDef  USBD_Device;

enum bootldr_cmd {
    BOOTLDR_START   = 0xABCD0001,
    BOOTLDR_RESET   = 0xABCD0002,
    BOOTLDR_ERASE   = 0xABCD0004,
    BOOTLDR_WRITE   = 0xABCD0008,
    BOOTLDR_FLASH   = 0xABCD0010,
};


/**
 * @brief  CDC_Itf_Control
 *         Manage the CDC class requests
 * @param  Cmd: Command code            
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
    switch (cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            /* Add your code here */
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            /* Add your code here */
            break;

        case CDC_SET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_GET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_CLEAR_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_SET_LINE_CODING:
            //UART params
            // LineCoding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) |
            //                                 (pbuf[2] << 16) | (pbuf[3] << 24));
            // LineCoding.format     = pbuf[4];
            // LineCoding.paritytype = pbuf[5];
            // LineCoding.datatype   = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            // pbuf[0] = (uint8_t)(LineCoding.bitrate);
            // pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
            // pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
            // pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
            // pbuf[4] = LineCoding.format;
            // pbuf[5] = LineCoding.paritytype;
            // pbuf[6] = LineCoding.datatype;     
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            vcp_connected = length & 1; // wValue is passed in Len (bit of a hack)
            break;

        case CDC_SEND_BREAK:
            /* Add your code here */
            break;    

        default:
            break;
    }

    return (USBD_OK);
}

// This function is called to process outgoing data.  We hook directly into the
// SOF (start of frame) callback so that it is called exactly at the time it is
// needed (reducing latency), and often enough (increasing bandwidth).
// void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
// {
//     uint32_t buffptr;
//     uint32_t buffsize;

//     if(UserTxBufPtrOut != UserTxBufPtrIn) {
//         if (UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */ {
//             buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
//         } else {
//             buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
//         }

//         buffptr = UserTxBufPtrOut;
//         USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[buffptr], buffsize);

//         if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) {
//             UserTxBufPtrOut += buffsize;
//             if (UserTxBufPtrOut == APP_RX_DATA_SIZE) {
//                 UserTxBufPtrOut = 0;
//             }
//         }
//     }
// }


void CDC_Tx(uint8_t *buf, uint32_t len)
{
    VCOM_Write(buf, len);
}

/**
 * @brief  CDC_Itf_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface 
 *         through this function.
 * @param  Buf: Buffer of data to be transmitted
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t Len)
{
    static volatile uint32_t flash_offset;

    uint32_t *cmd_buf = (uint32_t*) Buf; 
    uint32_t cmd = *cmd_buf++;

    //Debug... CDC_Tx(Buf,Len);

    switch (cmd) {
        case BOOTLDR_START://0x48730 BOOTLDR_START
            flash_buf_idx = 0;
            ide_connected = 1;
            flash_offset = MAIN_APP_ADDR;
            // Send back the bootloader version.
            CDC_Tx((uint8_t *) &bootloader_version, 4);
            break;
        case BOOTLDR_FLASH:
            // Return flash layout (bootloader v2)
            CDC_Tx((uint8_t*) flash_layout, 12);
            break;
        case BOOTLDR_RESET:
            ide_connected = 0;
            if (flash_buf_idx) {
                // Pad and flush the last packet
                for (int i=flash_buf_idx; i<FLASH_BUF_SIZE; i++) {
                    flash_buf[i] = 0xFF;
                }
                flash_write((uint32_t*)flash_buf, flash_offset, FLASH_BUF_SIZE);
            }
            break;
        case BOOTLDR_ERASE: {
            uint32_t sector = *cmd_buf; 
            flash_erase(sector);
            break; 
        }
        case BOOTLDR_WRITE: {
            uint8_t *buf =  Buf + 4;
            uint32_t len = Len - 4;
            for (int i=0; i<len; i++) {
                flash_buf[flash_buf_idx++] = buf[i];
                if (flash_buf_idx == FLASH_BUF_SIZE) {
                    flash_buf_idx = 0;
                    flash_write((uint32_t*)flash_buf, flash_offset, FLASH_BUF_SIZE);
                    flash_offset += FLASH_BUF_SIZE;
                }
            }
            break; 
        }
    }

    // Initiate next USB packet transfer
    //USBD_CDC_ReceivePacket(&USBD_Device);
    return USBD_OK;
}


uint8_t USBD_VCP_Connected(void)
{
    return vcp_connected;
}

uint8_t USBD_IDE_Connected(void)
{
    return ide_connected;
}


