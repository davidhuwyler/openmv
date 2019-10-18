/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * main function.
 *
 */

#define OPENMVRT_SEEED //--> Is already defined if Target in Makefile: TARGET ?= OPENMVRT_SEEED

#ifdef OPENMVRT_SEEED
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mpconfigboard.h"
#include "hal_wrapper.h"
#include "mpconfig.h"
#include "systick.h"
#include "pendsv.h"
#include "qstr.h"
#include "nlr.h"
#include "lexer.h"
#include "parse.h"
#include "compile.h"
#include "runtime.h"
#include "obj.h"
#include "objmodule.h"
#include "objstr.h"
#include "gc.h"
#include "stackctrl.h"
#include "gccollect.h"
#include "readline.h"
#include "usb_app.h"
#include "virtual_com.h"
#include "repl.h"
#include "pybthread.h"

#include "mperrno.h"
#include "lib/utils/pyexec.h"

#include "timer.h"
#include "pin.h"
#include "usb.h"
#include "rtc.h"
#include "storage.h"
#include "sdcard.h"
#include "ff.h"
//#include "modnetwork.h"
#include "modmachine.h"

#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include "lib/utils/pyexec.h"

#include "irq.h"
#include "rng.h"
#include "led.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "dac.h"
#include "can.h"
#include "extint.h"
#include "servo.h"

#include "sensor.h"
#include "usbdbg.h"
#include "sdram.h"
#include "fb_alloc.h"
#include "ff_wrapper.h"

#include "sensor.h"
#include "usbdbg.h"
//#include "wifidbg.h"
#include "sdram.h"
#include "fb_alloc.h"
#include "ff_wrapper.h"

#include "py_sensor.h"
#include "py_image.h"
#include "py_lcd.h"
#include "py_fir.h"
#include "py_tv.h"

#include "framebuffer.h"
#include "ini.h"
#include "overlay_manager.h"

#else
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include STM32_HAL_H
#include "mpconfig.h"
#include "systick.h"
#include "pendsv.h"
#include "qstr.h"
#include "nlr.h"
#include "lexer.h"
#include "parse.h"
#include "compile.h"
#include "runtime.h"
#include "obj.h"
#include "objmodule.h"
#include "objstr.h"
#include "gc.h"
#include "stackctrl.h"
#include "gccollect.h"
#include "readline.h"
#include "timer.h"
#include "pin.h"
#include "usb.h"
#include "rtc.h"
#include "storage.h"
#include "sdcard.h"
#include "ff.h"
#include "modnetwork.h"
#include "modmachine.h"

#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include "lib/utils/pyexec.h"

#include "irq.h"
#include "rng.h"
#include "led.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "dac.h"
#include "can.h"
#include "extint.h"
#include "servo.h"

#include "sensor.h"
#include "usbdbg.h"
#include "wifidbg.h"
#include "sdram.h"
#include "fb_alloc.h"
#include "ff_wrapper.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc_msc_hid.h"
#include "usbd_cdc_interface.h"
#include "usbd_msc_storage.h"

#include "py_sensor.h"
#include "py_image.h"
#include "py_lcd.h"
#include "py_fir.h"
#include "py_tv.h"

#include "framebuffer.h"

#include "ini.h"
#endif

pyb_thread_t pyb_thread_main;
static fs_user_mount_t fs_user_mount_flash;

int errno;
long long _vfs_buf[1024 / 8];
static fs_user_mount_t *vfs_fat = (fs_user_mount_t *) _vfs_buf;

void flash_error(int n) {

    led_state(LED_RED, 0);
    led_state(LED_GREEN, 0);
    led_state(LED_BLUE, 0);
    for (int i = 0; i < n; i++) {
        led_state(LED_RED, 0);
        HAL_Delay(100);
        led_state(LED_RED, 1);
        HAL_Delay(100);
    }
    led_state(LED_RED, 0);
}


void NORETURN __fatal_error(const char *msg) {
    FIL fp;
    if (f_open(&vfs_fat->fatfs, &fp, "ERROR.LOG",
               FA_WRITE|FA_CREATE_ALWAYS) == FR_OK) {
        UINT bytes;
        const char *hdr = "FATAL ERROR:\n";
        f_write(&fp, hdr, strlen(hdr), &bytes);
        f_write(&fp, msg, strlen(msg), &bytes);
    }
    f_close(&fp);
    storage_flush();

    for (uint i = 0;;) {
        led_toggle(((i++) & 3));
        for (volatile uint delay = 0; delay < 500000; delay++) {
        }
    }
}

void nlr_jump_fail(void *val) {
    printf("FATAL: uncaught exception %p\n", val);
    __fatal_error("");
}

#ifndef NDEBUG
void __attribute__((weak))
    __assert_func(const char *file, int line, const char *func, const char *expr) {
    (void)func;
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("");
}
#endif

void f_touch(const char *path)
{
    FIL fp;
    if (f_stat(&vfs_fat->fatfs, path, NULL) != FR_OK) {
        f_open(&vfs_fat->fatfs, &fp, path, FA_WRITE | FA_CREATE_ALWAYS);
        f_close(&fp);
    }
}

void make_flash_fs()
{
#ifdef OPENMVRT_SEEED

#else
    FIL fp;
    UINT n;

    led_state(LED_RED, 1);

    uint8_t working_buf[_MAX_SS];
    if (f_mkfs(&vfs_fat->fatfs, FM_FAT, 0, working_buf, sizeof(working_buf)) != FR_OK) {
        __fatal_error("Could not create LFS");
    }

    // Mark FS as OpenMV disk.
    f_touch("/.openmv_disk");

    // Create default main.py
    f_open(&vfs_fat->fatfs, &fp, "/main.py", FA_WRITE | FA_CREATE_ALWAYS);
    f_write(&fp, fresh_main_py, sizeof(fresh_main_py) - 1 /* don't count null terminator */, &n);
    f_close(&fp);

    // Create readme file
    f_open(&vfs_fat->fatfs, &fp, "/README.txt", FA_WRITE | FA_CREATE_ALWAYS);
    f_write(&fp, fresh_readme_txt, sizeof(fresh_readme_txt) - 1 /* don't count null terminator */, &n);
    f_close(&fp);

    // Create default selftest.py
    f_open(&vfs_fat->fatfs, &fp, "/selftest.py", FA_WRITE | FA_CREATE_ALWAYS);
    f_write(&fp, fresh_selftest_py, sizeof(fresh_selftest_py) - 1 /* don't count null terminator */, &n);
    f_close(&fp);

    led_state(LED_RED, 0);
#endif

}

#ifdef STACK_PROTECTOR
uint32_t __stack_chk_guard=0xDEADBEEF;

void NORETURN __stack_chk_fail(void)
{
    while (1) {
        flash_error(100);
    }
}
#endif

typedef struct openmv_config {
#ifdef OPENMVRT_SEEED

#else
    bool wifidbg;
    wifidbg_config_t wifidbg_config;
#endif

} openmv_config_t;

int ini_handler_callback(void *user, const char *section, const char *name, const char *value)
{

#ifdef OPENMVRT_SEEED

#else
    openmv_config_t *openmv_config = (openmv_config_t *) user;

    #define MATCH(s, n) ((strcmp(section, (s)) == 0) && (strcmp(name, (n)) == 0))

    if (MATCH("BoardConfig", "REPLUart")) {
        if (ini_is_true(value)) {
            mp_obj_t args[2] = {
                MP_OBJ_NEW_SMALL_INT(3), // UART Port
                MP_OBJ_NEW_SMALL_INT(115200) // Baud Rate
            };

            MP_STATE_PORT(pyb_stdio_uart) = pyb_uart_type.make_new((mp_obj_t) &pyb_uart_type, MP_ARRAY_SIZE(args), 0, args);
            uart_attach_to_repl(MP_STATE_PORT(pyb_stdio_uart), true);
        }
    } else if (MATCH("BoardConfig", "WiFiDebug")) {
        openmv_config->wifidbg = ini_is_true(value);
    } else if (MATCH("WiFiConfig", "Mode")) {
        openmv_config->wifidbg_config.mode = ini_atoi(value);
    } else if (MATCH("WiFiConfig", "ClientSSID")) {
        strncpy(openmv_config->wifidbg_config.client_ssid, value, WINC_MAX_SSID_LEN);
    } else if (MATCH("WiFiConfig", "ClientKey")) {
        strncpy(openmv_config->wifidbg_config.client_key,  value, WINC_MAX_PSK_LEN);
    } else if (MATCH("WiFiConfig", "ClientSecurity")) {
        openmv_config->wifidbg_config.client_security = ini_atoi(value);
    } else if (MATCH("WiFiConfig", "ClientChannel")) {
        openmv_config->wifidbg_config.client_channel = ini_atoi(value);
    } else if (MATCH("WiFiConfig", "AccessPointSSID")) {
        strncpy(openmv_config->wifidbg_config.access_point_ssid, value, WINC_MAX_SSID_LEN);
    } else if (MATCH("WiFiConfig", "AccessPointKey")) {
        strncpy(openmv_config->wifidbg_config.access_point_key,  value, WINC_MAX_PSK_LEN);
    } else if (MATCH("WiFiConfig", "AccessPointSecurity")) {
        openmv_config->wifidbg_config.access_point_security = ini_atoi(value);
    } else if (MATCH("WiFiConfig", "AccessPointChannel")) {
        openmv_config->wifidbg_config.access_point_channel = ini_atoi(value);
    } else if (MATCH("WiFiConfig", "BoardName")) {
        strncpy(openmv_config->wifidbg_config.board_name,  value, WINC_MAX_BOARD_NAME_LEN);
    } else {
        return 0;
    }
#endif

    return 1;

    #undef MATCH
}


FRESULT apply_settings(const char *path)
{
    nlr_buf_t nlr;
    FRESULT f_res = f_stat(&vfs_fat->fatfs, path, NULL);

    if (f_res == FR_OK) {
        if (nlr_push(&nlr) == 0) {
            ini_parse(&vfs_fat->fatfs, path, ini_handler_callback, NULL);
            nlr_pop();
        }
    }

    return f_res;
}


FRESULT exec_boot_script(const char *path, bool selftest, bool interruptible)
{

	FRESULT res = FR_OK;
	return res;


    nlr_buf_t nlr;
    bool interrupted = false;
    FRESULT f_res = f_stat(&vfs_fat->fatfs, path, NULL);

    if (f_res == FR_OK) {
        if (nlr_push(&nlr) == 0) {
            // Enable IDE interrupts if allowed.
            if (interruptible) {
                usbdbg_set_irq_enabled(true);
                usbdbg_set_script_running(true);
            }

            // Parse, compile and execute the script.
            pyexec_file(path);
            nlr_pop();
        } else {
            interrupted = true;
        }
    }

    // Disable IDE interrupts
    usbdbg_set_irq_enabled(false);
    usbdbg_set_script_running(false);

    if (interrupted) {
        if (selftest) {
            // Get the exception message. TODO: might be a hack.
            mp_obj_str_t *str = mp_obj_exception_get_value((mp_obj_t)nlr.ret_val);
            // If any of the self-tests fail log the exception message
            // and loop forever. Note: IDE exceptions will not be caught.
            __fatal_error((const char*) str->data);
        } else {
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
            if (nlr_push(&nlr) == 0) {
                flash_error(3);
                nlr_pop();
            }// If this gets interrupted again ignore it.
        }
    }

    if (selftest && f_res == FR_OK) {
        // Remove self tests script and flush cache
        f_unlink(&vfs_fat->fatfs, path);
        storage_flush();

        // Set flag for SWD debugger.
        // Note: main.py does not use the frame buffer.
        MAIN_FB()->bpp = 0xDEADBEEF;
    }

    return f_res;
}

extern uint32_t __StackTop;
extern uint32_t __StackSize;
extern uint32_t _heap_start;
extern uint32_t _heap_end;


HAL_StatusTypeDef HAL_Init(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
	PRINTF("Debug console inited!\r\n");
	/* Set Interrupt Group Priority */
	NVIC_SetPriorityGrouping(3);

	/* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
	HAL_InitTick(IRQ_PRI_SYSTICK);

	#if 0 // #ifdef XIP_EXTERNAL_FLASH
	uint32_t wait;
	for (wait = HAL_GetTick(); wait < 1501; wait = HAL_GetTick()) {
		if (wait % 250 == 0) {
			PRINTF("%d\r\n", (1501 - wait) / 250);
		}
		HAL_WFI();
	}
	#endif

	return HAL_OK;
}

#if(MICROPY_HW_HAS_HYPER_FLASH || MICROPY_HW_HAS_QSPI_FLASH) && MICROPY_HW_HAS_FLASH
static const char fresh_boot_py[] __ALIGNED(4) =
"# boot.py -- run on boot-up\r\n"
"# can run arbitrary Python, but best to keep it minimal\r\n"
"\r\n"
"import machine\r\n"
"import pyb\r\n"
"#pyb.main('main.py') # main script to run after this one\r\n"
"#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device\r\n"
"#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse\r\n"
;

static const char fresh_selftest_py[] =
"import sensor, time, pyb\n"
"\n"
"def test_int_adc():\n"
"    adc  = pyb.ADCAll(12)\n"
"    # Test VBAT\n"
"    vbat = adc.read_core_vbat()\n"
"    vbat_diff = abs(vbat-3.3)\n"
"    if (vbat_diff > 0.1):\n"
"        raise Exception('INTERNAL ADC TEST FAILED VBAT=%fv'%vbat)\n"
"\n"
"    # Test VREF\n"
"    vref = adc.read_core_vref()\n"
"    vref_diff = abs(vref-1.2)\n"
"    if (vref_diff > 0.1):\n"
"        raise Exception('INTERNAL ADC TEST FAILED VREF=%fv'%vref)\n"
"    adc = None\n"
"    print('INTERNAL ADC TEST PASSED...')\n"
"\n"
"def test_color_bars():\n"
"    sensor.reset()\n"
"    # Set sensor settings\n"
"    sensor.set_brightness(0)\n"
"    sensor.set_saturation(3)\n"
"    sensor.set_gainceiling(8)\n"
"    sensor.set_contrast(2)\n"
"\n"
"    # Set sensor pixel format\n"
"    sensor.set_framesize(sensor.QVGA)\n"
"    sensor.set_pixformat(sensor.RGB565)\n"
"\n"
"    # Enable colorbar test mode\n"
"    sensor.set_colorbar(True)\n"
"\n"
"    # Skip a few frames to allow the sensor settle down\n"
"    for i in range(0, 100):\n"
"        image = sensor.snapshot()\n"
"\n"
"    #color bars thresholds\n"
"    t = [lambda r, g, b: r < 70  and g < 70  and b < 70,   # Black\n"
"         lambda r, g, b: r < 70  and g < 70  and b > 200,  # Blue\n"
"         lambda r, g, b: r > 200 and g < 70  and b < 70,   # Red\n"
"         lambda r, g, b: r > 200 and g < 70  and b > 200,  # Purple\n"
"         lambda r, g, b: r < 70  and g > 200 and b < 70,   # Green\n"
"         lambda r, g, b: r < 70  and g > 200 and b > 200,  # Aqua\n"
"         lambda r, g, b: r > 200 and g > 200 and b < 70,   # Yellow\n"
"         lambda r, g, b: r > 200 and g > 200 and b > 200]  # White\n"
"\n"
"    # color bars are inverted for OV7725\n"
"    if (sensor.get_id() == sensor.OV7725):\n"
"        t = t[::-1]\n"
"\n"
"    #320x240 image with 8 color bars each one is approx 40 pixels.\n"
"    #we start from the center of the frame buffer, and average the\n"
"    #values of 10 sample pixels from the center of each color bar.\n"
"    for i in range(0, 8):\n"
"        avg = (0, 0, 0)\n"
"        idx = 40*i+20 #center of colorbars\n"
"        for off in range(0, 10): #avg 10 pixels\n"
"            rgb = image.get_pixel(idx+off, 120)\n"
"            avg = tuple(map(sum, zip(avg, rgb)))\n"
"\n"
"        if not t[i](avg[0]/10, avg[1]/10, avg[2]/10):\n"
"            raise Exception('COLOR BARS TEST FAILED.'\n"
"            'BAR#(%d): RGB(%d,%d,%d)'%(i+1, avg[0]/10, avg[1]/10, avg[2]/10))\n"
"\n"
"    print('COLOR BARS TEST PASSED...')\n"
"\n"
"if __name__ == '__main__':\n"
"    print('')\n"
"    test_int_adc()\n"
"    test_color_bars()\n"
"\n"
;

static const char fresh_main_py[] __ALIGNED(4) =
"# main.py -- put your code here!\n"
"import pyb, time\n"
"led = pyb.LED(1)\n"
"usb = pyb.USB_VCP()\n"
"while (usb.isconnected()==False):\n"
"   led.on()\n"
"   time.sleep(150)\n"
"   led.off()\n"
"   time.sleep(100)\n"
"   led.on()\n"
"   time.sleep(150)\n"
"   led.off()\n"
"   time.sleep(600)\n"
;

static const char fresh_pybcdc_inf[] __ALIGNED(4) =
#include "genhdr/pybcdc_inf.h"
;

static const char fresh_readme_txt[] __ALIGNED(4) =
"This is a MicroPython board\r\n"
"\r\n"
"You can get started right away by writing your Python code in 'main.py'.\r\n"
"\r\n"
"For a serial prompt:\r\n"
" - Windows: you need to go to 'Device manager', right click on the unknown device,\r\n"
"   then update the driver software, using the 'pybcdc.inf' file found on this drive.\r\n"
"   Then use a terminal program like Hyperterminal or putty.\r\n"
" - Mac OS X: use the command: screen /dev/tty.usbmodem*\r\n"
" - Linux: use the command: screen /dev/ttyACM0\r\n"
"\r\n"
"Please visit http://micropython.org/help/ for further help.\r\n"
"Thank you for supporting the OpenMV project!\r\n"
"\r\n"
"To download the IDE, please visit:\r\n"
"https://openmv.io/pages/download\r\n"
"\r\n"
"For tutorials and documentation, please visit:\r\n"
"http://docs.openmv.io/\r\n"
"\r\n"
"For technical support and projects, please visit the forums:\r\n"
"http://forums.openmv.io/\r\n"
"\r\n"
"Please use github to report bugs and issues:\r\n"
"https://github.com/openmv/openmv\r\n"
;

// avoid inlining to avoid stack usage within main()
MP_NOINLINE STATIC bool init_flash_fs(uint reset_mode) {
    // init the vfs object
    fs_user_mount_t *vfs_fat = &fs_user_mount_flash;
    vfs_fat->flags = 0;
    pyb_flash_init_vfs(vfs_fat);

    // try to mount the flash
    FRESULT res = f_mount(&vfs_fat->fatfs);

    if (reset_mode == 3 || res == FR_NO_FILESYSTEM) {
        // no filesystem, or asked to reset it, so create a fresh one

        // LED on to indicate creation of LFS
        led_state(PYB_LED_GREEN, 1);
        uint32_t start_tick = HAL_GetTick();

        uint8_t working_buf[_MAX_SS];
        res = f_mkfs(&vfs_fat->fatfs, FM_FAT, 0, working_buf, sizeof(working_buf));
        if (res == FR_OK) {
            // success creating fresh LFS
        } else {
            printf("PYB: can't create flash filesystem\n");
            return false;
        }

        // set label
        f_setlabel(&vfs_fat->fatfs, "pybflash");

        // create empty main.py
        FIL fp;
        f_open(&vfs_fat->fatfs, &fp, "/main.py", FA_WRITE | FA_CREATE_ALWAYS);
        UINT n;
        f_write(&fp, fresh_main_py, sizeof(fresh_main_py) - 1 /* don't count null terminator */, &n);
        // TODO check we could write n bytes
        f_close(&fp);

        // create .inf driver file
        f_open(&vfs_fat->fatfs, &fp, "/pybcdc.inf", FA_WRITE | FA_CREATE_ALWAYS);
        f_write(&fp, fresh_pybcdc_inf, sizeof(fresh_pybcdc_inf) - 1 /* don't count null terminator */, &n);
        f_close(&fp);

        // create readme file
        f_open(&vfs_fat->fatfs, &fp, "/README.txt", FA_WRITE | FA_CREATE_ALWAYS);
        f_write(&fp, fresh_readme_txt, sizeof(fresh_readme_txt) - 1 /* don't count null terminator */, &n);
        f_close(&fp);

	    // Create default selftest.py
	    f_open(&vfs_fat->fatfs, &fp, "/selftest.py", FA_WRITE | FA_CREATE_ALWAYS);
	    f_write(&fp, fresh_selftest_py, sizeof(fresh_selftest_py) - 1 /* don't count null terminator */, &n);
	    f_close(&fp);

        // keep LED on for at least 200ms
        sys_tick_wait_at_least(start_tick, 200);
        led_state(PYB_LED_GREEN, 0);
    } else if (res == FR_OK) {
        // mount sucessful
    } else {
    fail:
        printf("PYB: can't mount flash\n");
        return false;
    }

    // mount the flash device (there should be no other devices mounted at this point)
    // we allocate this structure on the heap because vfs->next is a root pointer
    mp_vfs_mount_t *vfs = m_new_obj_maybe(mp_vfs_mount_t);
    if (vfs == NULL) {
        goto fail;
    }
    vfs->str = "/flash";
    vfs->len = 6;
    vfs->obj = MP_OBJ_FROM_PTR(vfs_fat);
    vfs->next = NULL;
    MP_STATE_VM(vfs_mount_table) = vfs;

    // The current directory is used as the boot up directory.
    // It is set to the internal flash filesystem by default.
    MP_STATE_PORT(vfs_cur) = vfs;

    // Make sure we have a /flash/boot.py.  Create it if needed.
    FILINFO fno;
    res = f_stat(&vfs_fat->fatfs, "/boot.py", &fno);
    if (res != FR_OK) {
        // doesn't exist, create fresh file

        // LED on to indicate creation of boot.py
        led_state(PYB_LED_GREEN, 1);
        uint32_t start_tick = HAL_GetTick();

        FIL fp;
        f_open(&vfs_fat->fatfs, &fp, "/boot.py", FA_WRITE | FA_CREATE_ALWAYS);
        UINT n;
        f_write(&fp, fresh_boot_py, sizeof(fresh_boot_py) - 1 /* don't count null terminator */, &n);
        // TODO check we could write n bytes
        f_close(&fp);

        // keep LED on for at least 200ms
        sys_tick_wait_at_least(start_tick, 200);
        led_state(PYB_LED_GREEN, 0);
    }

    return true;
}
#endif //(MICROPY_HW_HAS_HYPER_FLASH || MICROPY_HW_HAS_QSPI_FLASH) && MICROPY_HW_HAS_FLASH

#if MICROPY_HW_HAS_SDCARD
STATIC bool init_sdcard_fs(bool first_soft_reset) {
    bool first_part = true;
    for (int part_num = 1; part_num <= 4; ++part_num) {
        // create vfs object
        fs_user_mount_t *vfs_fat = m_new_obj_maybe(fs_user_mount_t);
        mp_vfs_mount_t *vfs = m_new_obj_maybe(mp_vfs_mount_t);
        if (vfs == NULL || vfs_fat == NULL) {
            break;
        }
        vfs_fat->flags = FSUSER_FREE_OBJ;
        sdcard_init_vfs(vfs_fat, part_num);

        // try to mount the partition
        FRESULT res = f_mount(&vfs_fat->fatfs);

        if (res != FR_OK) {
            // couldn't mount

            m_del_obj(fs_user_mount_t, vfs_fat);
            m_del_obj(mp_vfs_mount_t, vfs);
        } else {
            // mounted via FatFs, now mount the SD partition in the VFS
            if (first_part) {
                // the first available partition is traditionally called "sd" for simplicity
                vfs->str = "/";
                vfs->len = 1;
				// for openMV's root dir, set to SD card
				MP_STATE_PORT(vfs_cur) = vfs;
            } else {
                // subsequent partitions are numbered by their index in the partition table
                if (part_num == 2) {
                    vfs->str = "/sd2";
                } else if (part_num == 2) {
                    vfs->str = "/sd3";
                } else {
                    vfs->str = "/sd4";
                }
                vfs->len = 4;
            }
            vfs->obj = MP_OBJ_FROM_PTR(vfs_fat);
            vfs->next = NULL;
            for (mp_vfs_mount_t **m = &MP_STATE_VM(vfs_mount_table);; m = &(*m)->next) {
                if (*m == NULL) {
                    *m = vfs;
                    break;
                }
            }

            if (first_soft_reset) {
                // use SD card as medium for the USB MSD
                #if defined(USE_DEVICE_MODE)
                pyb_usb_storage_medium = PYB_USB_STORAGE_MEDIUM_SDCARD;
                #endif
            }

            #if defined(USE_DEVICE_MODE)
            // only use SD card as current directory if that's what the USB medium is
            if (pyb_usb_storage_medium == PYB_USB_STORAGE_MEDIUM_SDCARD)
            #endif
            {
                if (first_part) {
                    // use SD card as current directory
                    MP_STATE_PORT(vfs_cur) = vfs;
                }
            }
            first_part = false;
        }
    }

    if (first_part) {
        printf("PYB: can't mount SD card\n");
        return false;
    } else {
        return true;
    }
}
#endif


int main(void)
{
    bool first_soft_reset = true;
    HAL_Init();

    // Basic sub-system init
    #if MICROPY_PY_THREAD
    pyb_thread_init(&pyb_thread_main);
    #endif
    pendsv_init();
    led_init();

soft_reset:
	{
		//Flash Magenta LED 3times at Startup
		HAL_Delay(200);
		for(int i = 1; i<7; i++)
		{
			led_state(LED_RED,i%2);
			led_state(LED_BLUE,i%2);
			HAL_Delay(200);
		}
	}

	//TODO Dave: Include a reset_mode update function
	uint reset_mode = 1;
    machine_init();

    // more sub-system init
#if MICROPY_HW_HAS_SDCARD
    if (first_soft_reset) {
        sdcard_init();
        HAL_Delay(200);
    }
#endif


    // Python threading init
#if MICROPY_PY_THREAD
     mp_thread_init();
#endif
     // Stack limit should be less than real stack size, so we have a
     // chance to recover from limit hit. (Limit is measured in bytes)
     mp_stack_set_top(&__StackTop);
     mp_stack_set_limit(&__StackSize);

     // GC init
#if MICROPY_ENABLE_GC
     gc_init(&_heap_start,&_heap_end);
#endif
     MP_STATE_PORT(omv_ide_irq) = 0;

     // Micro Python init
     mp_init();
     mp_obj_list_init(mp_sys_path, 0);
     mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_)); // current dir (or base dir of the script)
     mp_obj_list_init(mp_sys_argv, 0);

     switch_init0();
     readline_init0();
     pin_init0();
     spi_init0();
     uart_init0();
     usbdbg_init();
     pyb_usb_init0();
     sensor_init0();


     // Define MICROPY_HW_UART_REPL to be PYB_UART_6 and define
     // MICROPY_HW_UART_REPL_BAUD in your mpconfigboard.h file if you want a
     // REPL on a hardware UART as well as on USB VCP
 #if defined(MICROPY_HW_UART_REPL)
     {

         mp_obj_t args[2] = {
             MP_OBJ_NEW_SMALL_INT(MICROPY_HW_UART_REPL),
             MP_OBJ_NEW_SMALL_INT(115200),
         };
         MP_STATE_PORT(pyb_stdio_uart) = pyb_uart_type.make_new((mp_obj_t)&pyb_uart_type, MP_ARRAY_SIZE(args), 0, args);
     }
 #else
     MP_STATE_PORT(pyb_stdio_uart) = NULL;
 #endif


 	/*--------------------------------------------------*
 	 *  	  	Init/Mount FlashFileSystem				*
 	 *--------------------------------------------------*/
     // Create it if needed, mount in on /flash, and set it as current dir.
 	bool mounted_flash;
 	#if !defined(XIP_EXTERNAL_FLASH) && (MICROPY_HW_HAS_HYPER_FLASH || MICROPY_HW_HAS_QSPI_FLASH) && MICROPY_HW_HAS_FLASH
     mounted_flash = init_flash_fs(reset_mode);
 	#else
 	mounted_flash = 0;
 	#endif


    bool mounted_sdcard = false;
	/*--------------------------------------------------*
	 *  	  		Init/Mount SD Card						*
	 *--------------------------------------------------*/
#if MICROPY_HW_HAS_SDCARD
    // if an SD card is present then mount it on /sd/
    if (sdcard_is_present()) {
        // if there is a file in the flash called "SKIPSD", then we don't mount the SD card
        if (!mounted_flash || f_stat(&fs_user_mount_flash.fatfs, "/SKIPSD", NULL) != FR_OK) {
			int retry = 16;
			while (retry--) {
				mounted_sdcard = init_sdcard_fs(first_soft_reset);
				if (mounted_sdcard)
					break;
				else
				{
					PRINTF("can't mount SD card FS!\r\n");
					HAL_Delay(200);
				}
			}
        }
    } else {

	}
#endif

    // set sys.path based on mounted filesystems (/sd is first so it can override /flash)
    if (mounted_sdcard) {
    	sdcard_setSysPathToSD();
    }
    if (mounted_flash) {
    	flash_setSysPathToFlashFS();
    }

    // reset config variables; they should be set by boot.py
    MP_STATE_PORT(pyb_config_main) = MP_OBJ_NULL;

	/*--------------------------------------------------*
	 *  	  		Run Boot.py from SD	(of Flash)		*
	 *--------------------------------------------------*/
	// TODO perhaps have pyb.reboot([bootpy]) function to soft-reboot and execute custom boot.py
	if (reset_mode == 1 || reset_mode == 3) {
		const char *boot_py = "boot.py";
		mp_import_stat_t stat = mp_import_stat(boot_py);
		if (stat == MP_IMPORT_STAT_FILE) {
			PRINTF("Executing boot.py\r\n");
			int ret = pyexec_file(boot_py);
			if (ret & PYEXEC_FORCED_EXIT) {
				goto soft_reset_exit;
			}
			if (!ret) {
				flash_error(4);
			}
		}
	}

	#if defined(USE_DEVICE_MODE)
    // init USB device to default setting if it was not already configured
    if (first_soft_reset) {
	    if (!(pyb_usb_flags & PYB_USB_FLAG_USB_MODE_CALLED)) {
	        pyb_usb_dev_init(USBD_VID, USBD_PID_CDC_MSC, USBD_MODE_CDC_MSC, NULL);
			// NVIC_SetPriority(USB_OTG1_IRQn, 0);
	    }
    }
	#endif

     VCOM_Open();

	int ret = 0;
	PRINTF("Enter OpenMV main\r\n");
	// SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;
    file_buffer_init0();

 MainLoop:
	/*--------------------------------------------------*
	 *  	  		Run Main.py from SD					*
	 *--------------------------------------------------*/
	if (!usbdbg_script_ready()) {
		if (first_soft_reset) {
			first_soft_reset = 0;
			exec_boot_script("/selftest.py", true, false);
			apply_settings("/openmv.config");
			usbdbg_set_irq_enabled(true);
			// rocky: pyb's main uses different method to access file system from omv
			mp_import_stat_t stat = mp_import_stat("main.py");
			if (stat == MP_IMPORT_STAT_FILE) {
				nlr_buf_t nlr;
				if (nlr_push(&nlr) == 0) {
					PRINTF("executing main.py...\r\n");
					int ret = pyexec_file("main.py");
					if (ret & PYEXEC_FORCED_EXIT) {
						ret = 1;
					}
					if (!ret) {
						flash_error(3);
					}
					nlr_pop();
				}
				else {
					flash_error(3);
					goto cleanup;
				}
			}
		}
	}

	/*--------------------------------------------------*
	 *  	 Run REPL (if no OpenMV sript ready)		*
	 *--------------------------------------------------*/
    while (!usbdbg_script_ready()) {
        nlr_buf_t nlr;

        if (nlr_push(&nlr) == 0) {
            // enable IDE interrupt
            usbdbg_set_irq_enabled(true);

            // run REPL
            if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                if (pyexec_raw_repl() != 0) {
                    break;
                }
            } else {
                if (pyexec_friendly_repl() != 0) {
					ret = 1;
                    break;
                }
            }
            nlr_pop();
        }
    }

	/*--------------------------------------------------*
	 *  	  		Run OpenMV IDE Skript				*
	 *--------------------------------------------------*/
    if (usbdbg_script_ready()) {
        nlr_buf_t nlr;
		PRINTF("script ready!\r\n");
        // execute the script
        if (nlr_push(&nlr) == 0) {
			// rocky: 2019.03.27 19:00 reset fb alloc memory for new script
			#ifndef OMV_MPY_ONLY
			fb_alloc_init0();
			#endif

			usbdbg_set_irq_enabled(true);
			pyexec_str(usbdbg_get_script());

            nlr_pop();
        } else {
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
        }
    }
cleanup:
	usbdbg_set_irq_enabled(true);
    // Disable all other IRQs except Systick and Flash IRQs
    // Note: FS IRQ is disable, since we're going for a soft-reset.
    // __set_BASEPRI(IRQ_PRI_FLASH + 1);

    // soft reset
//    storage_flush();
//    timer_deinit();
//    uart_deinit();
//    can_deinit();
	ProfReset();


soft_reset_exit:

	    // soft reset

	printf("PYB: sync filesystems\n");
	storage_flush();

	printf("PYB: soft reboot\n");
	// rocky ignore: timer_deinit();

	// rocky ignore: uart_deinit();
#if MICROPY_HW_ENABLE_CAN
	// rocky ignore: can_deinit();
#endif

	#if MICROPY_PY_THREAD
	pyb_thread_deinit();
	#endif

	first_soft_reset = false;
	goto soft_reset;

    return 0;
}
