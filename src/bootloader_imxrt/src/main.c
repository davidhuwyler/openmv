#include "omv_boardconfig.h"
#include "bootloader_flash.h"
#include "usb_app.h"
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "hal_wrapper.h"
#include "systick.h"
#include "irq.h"

#define IDE_TIMEOUT     (1000)
#define CONFIG_TIMEOUT  (2000)

#define PYB_USB_FLAG_DEV_ENABLED        (0x0001)
#define PYB_USB_FLAG_USB_MODE_CALLED    (0x0002)

// Windows needs a different PID to distinguish different device configurations
// must use hex number directly to let python script that generate inf driver work normally
#define USBD_VID         (0x1209)
// nxp is (0x1FC9)
// (0x1400 | USBD_MODE_CDC | USBD_MODE_MSC)
#define USBD_PID_CDC_MSC (0xABD1)
// (0x1400 | USBD_MODE_CDC | USBD_MODE_HIDK | USBD_MODE_HIDM)
#define USBD_PID_CDC_HID (0xABD1)
// (0x1400 | USBD_MODE_CDC)
#define USBD_PID_CDC     (0xABD1)



#define LED_R_PORT GPIO1
#define LED_R_PIN 9
#define LED_G_PORT GPIO1
#define LED_G_PIN 10
#define LED_B_PORT GPIO1
#define LED_B_PIN 11

bool cdcIsConnected = false;

void __flash_led()
{
	GPIO_PortToggle(LED_R_PORT, 1u << LED_R_PIN);
    HAL_Delay(100);
    GPIO_PortToggle(LED_R_PORT, 1u << LED_R_PIN);
    HAL_Delay(100);
}

void __attribute__((noreturn)) __fatal_error()
{
    while (1) {
        __flash_led();
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


void setCDCconnect()
{
    cdcIsConnected= true;
}

//Systick Interrupt Service Routine
void SysTick_Handler(void) {
    extern uint32_t uwTick;	
    uwTick += 1;
	__DSB();
}

static void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer,uint32_t vectorTable)
{
    ARM_MPU_Disable();
    SCB_DisableDCache();
    SCB_DisableICache();
    
    __DSB(); __ISB();

    //Disable Interrupts:
    __asm volatile ("cpsid i");

    // Create the function call to the user application.
    // Static variables are needed since changed the stack pointer out from under the compiler
    // we need to ensure the values we are using are not stored on the previous stack
    static uint32_t s_stackPointer = 0;
    s_stackPointer = stackPointer;
    static void (*farewellBootloader)(void) = 0;
    farewellBootloader = (void (*)(void))applicationAddress;

    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)vectorTable;

    // Set stack pointers to the application stack pointer.
    __set_MSP(s_stackPointer);

    // Jump to the application.
    farewellBootloader();
}

int main()
{
	BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
	NVIC_SetPriorityGrouping(3);

	//Init LEDs and turn them off (Pin == High)
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    GPIO_PinInit(LED_R_PORT, LED_R_PIN, &led_config);
    GPIO_PinInit(LED_G_PORT, LED_G_PIN, &led_config);
    GPIO_PinInit(LED_B_PORT, LED_B_PIN, &led_config);
    GPIO_SetPinsOutput(LED_R_PORT, 1u << LED_R_PIN);
    GPIO_SetPinsOutput(LED_G_PORT, 1u << LED_G_PIN);
    GPIO_SetPinsOutput(LED_B_PORT, 1u << LED_B_PIN);

    //init Systick 1ms
    SysTick->CTRL &= SysTick_CTRL_ENABLE_Msk;
	HAL_InitTick(IRQ_PRI_SYSTICK);
	
    flash_init();

    //init USB CDC
    USBD_SetVIDPIDRelease(USBD_VID, USBD_PID_CDC, 0x0200, true);
    if (USBD_SelectMode(USBD_MODE_CDC, NULL) < 0) {
        __fatal_error();
    }
    USBAPP_Init();
    VCOM_Open();
    
    HAL_Delay(500);

    if (cdcIsConnected) {
        //uint32_t start = HAL_GetTick();
        // Wait for device to be configured
        // while (USBD_Device.dev_state != USBD_STATE_CONFIGURED
        //         // We still have to timeout because the camera
        //         // might be connected to a power bank or charger
        //         && (HAL_GetTick() - start) < CONFIG_TIMEOUT) {
        //     __flash_led();
        // }

        // If the device is configured, wait for IDE to connect or timeout
        //if (USBD_Device.dev_state == USBD_STATE_CONFIGURED) {
            // uint32_t start = HAL_GetTick();
            // while (!USBD_IDE_Connected()
            //         && (HAL_GetTick() - start) < IDE_TIMEOUT) {
            //     __flash_led();
            // }

         	while (!USBD_IDE_Connected()) {
         		__flash_led();
         	}

            // Wait for new firmware image if the IDE is connected
            while (USBD_IDE_Connected()) {
                __flash_led();
            }
        //}
    }

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


    USBAPP_Deinit();
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    jump_to_application
	(
    		0x6010231c|1, //ResetISR_C() Function-Address of the Application
						  //The last bit needs to be == 1 ( |1 ) to create a
						  //valid Thumb-Instruction Jump
			0x20068000,   //Address to the new StackTop
			0x60102000	  //Location of the new Vectortable
	);

}
