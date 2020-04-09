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

#define USBD_VID         (0x1209)
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

    static uint32_t s_stackPointer = 0;
    s_stackPointer = stackPointer;
    static void (*applicationEntryPoint)(void) = 0;
    applicationEntryPoint = (void (*)(void))applicationAddress;

    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)vectorTable;

    // Set stack pointers to the application stack pointer.
    __set_MSP(s_stackPointer);

    // Jump to the application.
    applicationEntryPoint();
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
    
    //Wait for USB init
    __flash_led();
    __flash_led();
    __flash_led();
    if (cdcIsConnected) {
        uint32_t start = HAL_GetTick();
        while (!USBD_IDE_Connected()
            && (HAL_GetTick() - start) < IDE_TIMEOUT) {
            __flash_led();
        }

        // Wait for new firmware image if the IDE is connected
        while (USBD_IDE_Connected()) {
            __flash_led();
        }
    }

    USBAPP_Deinit();
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    jump_to_application
	(
    		0x6010231c|1, //ResetISR_C() Function-Address of the Application
						  //The last bit needs to be == 1 ( |1 ) to create a
						  //valid Thumb-Instruction Jump
                          //--> Search ResetISR_C in MIMXRT1052xxxxB_Application.map
			0x20068000,   //Address to the new StackTop
                          //--> Search _vStackTop in MIMXRT1052xxxxB_Application.map
			0x60102000	  //Location of the new Vectortable
                          //--> Search __vectors_start__ in MIMXRT1052xxxxB_Application.map
	);
}
