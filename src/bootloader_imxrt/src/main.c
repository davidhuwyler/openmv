
#include "omv_boardconfig.h"
#include "flash.h"
#include "usb_app.h"

#define IDE_TIMEOUT     (1000)
#define CONFIG_TIMEOUT  (2000)


void __flash_led()
{
    // HAL_GPIO_TogglePin(OMV_BOOTLDR_LED_PORT, OMV_BOOTLDR_LED_PIN);
    // HAL_Delay(100);
    // HAL_GPIO_TogglePin(OMV_BOOTLDR_LED_PORT, OMV_BOOTLDR_LED_PIN);
    // HAL_Delay(100);
}

void __attribute__((noreturn)) __fatal_error()
{
    while (1) {
        // __flash_led();
    }
}

#ifdef STACK_PROTECTOR
uint32_t __stack_chk_guard=0xDEADBEEF;

void __attribute__((noreturn)) __stack_chk_fail(void)
{
    __asm__ volatile ("BKPT");
    while (1) {
        __flash_led();
    }
}
#endif

uint8_t HAL_Init(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
	NVIC_SetPriorityGrouping(3);

	return 0;
}

int main()
{
    // Override main app interrupt vector offset (set in system_stm32fxxx.c)
    //SCB->VTOR = FLASH_BASE | 0x0;

    HAL_Init();

    pyb_usb_dev_init(USBD_VID, USBD_PID_CDC_MSC, USBD_MODE_CDC_MSC, NULL);
    VCOM_Open();

    /* Init Device Library */
    //USBD_Init(&USBD_Device, &VCP_Desc, 0);

    /* Add Supported Class */
    //USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

    /* Add CDC Interface Class */
    //USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

    /* Start Device Process */
    //USBD_Start(&USBD_Device);

    // Note: The SRQINT interrupt is triggered when VBUS is in the valid range, I assume it's safe
    // to use it to detect if USB is connected or not. The dev_connection_status is set in usbd_conf.c
    // It wasn't used anywhere else, so again assuming it's safe to use it for connection status.
    // if (USBD_Device.dev_connection_status) {
    //     uint32_t start = HAL_GetTick();
    //     // Wait for device to be configured
    //     while (USBD_Device.dev_state != USBD_STATE_CONFIGURED
    //             // We still have to timeout because the camera
    //             // might be connected to a power bank or charger
    //             && (HAL_GetTick() - start) < CONFIG_TIMEOUT) {
    //         __flash_led();
    //     }

    //     // If the device is configured, wait for IDE to connect or timeout
    //     if (USBD_Device.dev_state == USBD_STATE_CONFIGURED) {
    //         uint32_t start = HAL_GetTick();
    //         while (!USBD_IDE_Connected()
    //                 && (HAL_GetTick() - start) < IDE_TIMEOUT) {
    //             __flash_led();
    //         }

    //         // Wait for new firmware image if the IDE is connected
    //         while (USBD_IDE_Connected()) {
    //             __flash_led();
    //         }
    //     }
    // }

    // Deinit USB
    //USBD_DeInit(&USBD_Device);

    // Disable IRQs
    //__disable_irq(); __DSB(); __ISB();

    // Jump to main app
    ((void (*)(void))(*((uint32_t*) (MAIN_APP_ADDR+4))))();
}
