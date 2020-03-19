#define MICROPY_HW_BOARD_NAME       "SeeedStudio Arch Mix"
#define MICROPY_HW_MCU_NAME         "i.MX RT1052"
#define MICROPY_HW_UART_REPL    	(repl_uart_id)	// uart ID of REPL uart, must be the same as repl_uart_id in uart.h
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (0)
#define MICROPY_HW_HAS_SDCARD       (1)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_CTMR      (0)
#define MICROPY_HW_ENABLE_SERVO     (1)
#define MICROPY_HW_ENABLE_DAC       (0)
#define MICROPY_HW_ENABLE_CAN       (0)
#define MICROPY_MW_ENABLE_SWIM		(0)

#define MICROPY_HW_HAS_FLASH_FS     (0) //FlashFileSystem
#define MICROPY_HW_HAS_QSPI_FLASH	(0)
#define MICROPY_HW_HAS_HYPER_FLASH	(1)


// RGB LED
#define MICROPY_HW_LED1             (pin_AD_B0_09) // red   GPIO_AD_B0_09
#define MICROPY_HW_LED2             (pin_AD_B0_10) // green GPIO_AD_B0_10
#define MICROPY_HW_LED3             (pin_AD_B0_11) // blue  GPIO_AD_B0_11

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_B1_12)
#define MICROPY_HW_SDCARD_DETECT_PULL       (1) // (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (2) // (GPIO_PIN_RESET)

// User Switch
#define MICROPY_HW_HAS_SWITCH		(1)
#define MICROPY_HW_USRSW_PIN        (pin_WAKEUP)
#define MICROPY_HW_USRSW_PULL       (1) //(GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (2) //(GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)

// SPI
#define MICROPY_HW_SPI4_NAME "spi4"
#define MICROPY_HW_SPI4_NSS  (pin_B1_04)
#define MICROPY_HW_SPI4_SCK  (pin_B1_07)
#define MICROPY_HW_SPI4_MISO (pin_B1_05)
#define MICROPY_HW_SPI4_MOSI (pin_B1_06)

// I2C
#define MICROPY_HW_I2C1_NAME "i2c1"
#define MICROPY_HW_I2C1_SDA (pin_AD_B1_01)
#define MICROPY_HW_I2C1_SCL (pin_AD_B1_00)

// UART
#define MICROPY_HW_UART1_NAME   "uart1"
#define MICROPY_HW_UART1_RX     (pin_AD_B0_13)
#define MICROPY_HW_UART1_TX     (pin_AD_B0_12)
#define MICROPY_HW_UART1_ALT	2


