#include <stdint.h>
#include "flash.h"
//#include "usbdev/usbd_cdc.h"
#include "omv_boardconfig.h"
#include "usbd_cdc_interface.h"

#define APP_RX_DATA_SIZE    (2048)
#define APP_TX_DATA_SIZE    (2048)

#define USBD_OK 0

uint32_t BuffLength;
uint32_t UserTxBufPtrIn = 0;    /* Increment this pointer or roll it back to
                                   start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0;   /* Increment this pointer or roll it back to
                                   start address when data are sent over USB */
uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

static volatile uint8_t ide_connected = 0;
static volatile uint8_t vcp_connected = 0;

#define FLASH_BUF_SIZE  (256)
static volatile uint32_t flash_buf_idx=0;
static volatile uint8_t  flash_buf[FLASH_BUF_SIZE];

// Flash sectors for the bootloader.
// Flash FS sector, main FW sector, max sector.
// Sector (Blocks in iMX RT) 0..4 == Bootloader   0x60'000'000 .. 0x60'100'000 (1M)
// Sector (Blocks in iMX RT) 5..31 == Applicaton  0x60'100'000 .. 0x60'800'000 (8M)
static const    uint32_t flash_layout[3] = {31, 5, 31};
static const    uint32_t bootloader_version = 0xABCD0002;

static uint8_t initBootloaderAnser[] = { 0x03, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00 };

/* USB handler declaration */
//extern USBD_HandleTypeDef  USBD_Device;

enum bootldr_cmd {
    BOOTLDR_START   = 0xABCD0001,
    BOOTLDR_RESET   = 0xABCD0002,
    BOOTLDR_ERASE   = 0xABCD0004,
    BOOTLDR_WRITE   = 0xABCD0008,
    BOOTLDR_FLASH   = 0xABCD0010,
};

void CDC_Tx(uint8_t *buf, uint32_t len)
{
    VCOM_Write(buf, len);
    //VCOM_OmvWriteAlways(buf,len);
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

    //CDC_Tx(Buf,Len); //Echo for debugging

    switch (cmd) {

    	case 0xc8030:
            CDC_Tx(initBootloaderAnser, sizeof(initBootloaderAnser));
    		break;

        case BOOTLDR_START://0xABCD0001
            flash_buf_idx = 0;
            ide_connected = 1;
            flash_offset = MAIN_APP_ADDR;
            // Send back the bootloader version.
            CDC_Tx((uint8_t *) &bootloader_version, 4);
            break;
        case BOOTLDR_FLASH: //0xABCD0010
            // Return flash layout (bootloader v2)
            CDC_Tx((uint8_t*) flash_layout, 12);
            break;
        case BOOTLDR_RESET: //0xABCD0002
            ide_connected = 0;
            if (flash_buf_idx) {
                // Pad and flush the last packet
                for (int i=flash_buf_idx; i<FLASH_BUF_SIZE; i++) {
                    flash_buf[i] = 0xFF;
                }
                flash_write((uint32_t*)flash_buf, flash_offset, FLASH_BUF_SIZE);
            }
            break;
        case BOOTLDR_ERASE: //0xABCD0004
        {
            uint32_t sector = *cmd_buf; 
            flash_erase(sector);
            break; 
        }
        case BOOTLDR_WRITE: //0xABCD0008
        {
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


