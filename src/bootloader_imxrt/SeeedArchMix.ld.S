/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for MIMXRT1052xxxxB
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v11.0.0 [Build 2516] [2019-06-05] on Oct 4, 2019 3:32:48 PM
 */

/*
INCLUDE "MIMXRT1052xxxxB_Project_Test_Debug_library.ld"

Removed:   "libcr_newlib_semihost.a"

*/

GROUP (
  "libgcc.a"
  "libc_nano.a"
  "libm.a"
  "libnosys.a"   
)

/*
INCLUDE "MIMXRT1052xxxxB_Project_Test_Debug_memory.ld"
*/
MEMORY
{
  /* Define each memory region */
  BOARD_FLASH (rx) : ORIGIN = 0x60000000, LENGTH = 0x100000 /* 1M bytes (alias Flash) */
  SRAM_ITC (rwx) : ORIGIN = 0x0, LENGTH = 0x5000 /* 20K bytes (alias RAM2) */
  SRAM_OC (rwx) : ORIGIN = 0x20200000, LENGTH = 0x10000 /* 64K bytes (alias RAM3) */
}

  /* Define a symbol for the top of each memory region */
  __base_BOARD_FLASH = 0x60000000  ; /* BOARD_FLASH */  
  __base_Flash = 0x60000000 ; /* Flash */  
  __top_BOARD_FLASH = 0x60000000 + 0x100000 ; /* 1M bytes */
  __top_Flash = 0x60000000 + 0x100000 ; /* 1M bytes */
  __base_SRAM_ITC = 0x0  ; /* SRAM_ITC */  
  __base_RAM2 = 0x0 ; /* RAM2 */  
  __top_SRAM_ITC = 0x0 + 0x5000 ; /* 20K bytes */
  __top_RAM2 = 0x0 + 0x5000 ; /* 20K bytes */
  __base_SRAM_OC = 0x20200000  ; /* SRAM_OC */  
  __base_RAM3 = 0x20200000 ; /* RAM3 */  
  __top_SRAM_OC = 0x20200000 + 0x10000 ; /* 64K bytes */
  __top_RAM3 = 0x20200000 + 0x10000 ; /* 64K bytes */

/*
Rest:
*/

ENTRY(ResetISR)

SECTIONS
{
/*---------------------------------------------------------------------------------
ExternalFlash:  (BOARD_FLASH)
---------------------------------------------------------------------------------*/
     /* Image Vector Table and Boot Data for booting from external flash */
    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;
    } >BOARD_FLASH

    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

    } > BOARD_FLASH


/*---------------------------------------------------------------------------------
Exclude flash_QuadSpi.o & fsl_flexspi.o from the Code in the BOARD_FLASH
 References:
 MCUXpresso_IDE_User_Guide.pdf: 18.15.1 Relocating code from FLASH to RAM
 https://mcuoneclipse.com/2014/10/06/putting-code-of-files-into-special-section-with-the-gnu-linker/
---------------------------------------------------------------------------------*/
    .text : ALIGN(4)
    {
       /* *(.text*)  */
       /* *(.rodata .rodata.* .constdata .constdata.*) */

	   *(EXCLUDE_FILE(*flash_QuadSpi.o *fsl_flexspi.o) .text*)
       *(EXCLUDE_FILE(*flash_QuadSpi.o *fsl_flexspi.o) .rodata)
       *(EXCLUDE_FILE(*flash_QuadSpi.o *fsl_flexspi.o) .rodata.*)
       *(EXCLUDE_FILE(*flash_QuadSpi.o *fsl_flexspi.o) .constdata)
       *(EXCLUDE_FILE(*flash_QuadSpi.o *fsl_flexspi.o) .constdata.*)

       . = ALIGN(4);
    } > BOARD_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4) 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > BOARD_FLASH

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > BOARD_FLASH
    __exidx_end = .;
 
    _etext = .;
        

/*---------------------------------------------------------------------------------
InstructionTightCoupledMemory:  (SRAM_ITC)
---------------------------------------------------------------------------------*/
    /* DATA section for SRAM_ITC */
    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;

		*flash_QuadSpi.o (.text*)
		*flash_QuadSpi.o (.rodata .rodata.* .constdata .constdata.*)
		*fsl_flexspi.o (.text*)
		*fsl_flexspi.o (.rodata .rodata.* .constdata .constdata.*)

        *(.ramfunc.$RAM2)
        *(.ramfunc.$SRAM_ITC)
        *(.data.$RAM2)
        *(.data.$SRAM_ITC)
        *(.data.$RAM2.*)
        *(.data.$SRAM_ITC.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
     } > SRAM_ITC AT>BOARD_FLASH

/*---------------------------------------------------------------------------------
OnChip Memory:  (SRAM_OC)
---------------------------------------------------------------------------------*/
    /* DATA section for SRAM_OC */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$SRAM_OC)
        *(.data.$RAM3)
        *(.data.$SRAM_OC)
        *(.data.$RAM3.*)
        *(.data.$SRAM_OC.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
     } > SRAM_OC AT>BOARD_FLASH


/*---------------------------------------------------------------------------------
External RAM:  (BOARD_SDRAM)
---------------------------------------------------------------------------------*/
    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) :
    {
        . = ALIGN(4) ;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > SRAM_ITC


/*---------------------------------------------------------------------------------
BSS (uninitialized Data) in all memo Sections
---------------------------------------------------------------------------------*/
    /* BSS section for SRAM_ITC */
    .bss_RAM2 :
    {
       . = ALIGN(4) ;
       PROVIDE(__start_bss_RAM2 = .) ;
       *(.bss.$RAM2)
       *(.bss.$SRAM_ITC)
       *(.bss.$RAM2.*)
       *(.bss.$SRAM_ITC.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
    } > SRAM_ITC

    /* BSS section for SRAM_OC */
    .bss_RAM3 :
    {
       . = ALIGN(4) ;
       PROVIDE(__start_bss_RAM3 = .) ;
       *(.bss.$RAM3)
       *(.bss.$SRAM_OC)
       *(.bss.$RAM3.*)
       *(.bss.$SRAM_OC.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
    } > SRAM_OC


    /* MAIN BSS SECTION */
    .bss :
    {
        . = ALIGN(4) ;
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > SRAM_ITC

/*---------------------------------------------------------------------------------
NOINIT in all memo Sections
---------------------------------------------------------------------------------*/
    /* NOINIT section for SRAM_ITC */
    .noinit_RAM2 (NOLOAD) :
    {
       . = ALIGN(4) ;
       *(.noinit.$RAM2)
       *(.noinit.$SRAM_ITC)
       *(.noinit.$RAM2.*)
       *(.noinit.$SRAM_ITC.*)
       . = ALIGN(4) ;
    } > SRAM_ITC

    /* NOINIT section for SRAM_OC */
    .noinit_RAM3 (NOLOAD) :
    {
       . = ALIGN(4) ;
       *(.noinit.$RAM3)
       *(.noinit.$SRAM_OC)
       *(.noinit.$RAM3.*)
       *(.noinit.$SRAM_OC.*)
       . = ALIGN(4) ;
    } > SRAM_OC



/*---------------------------------------------------------------------------------
Heap
---------------------------------------------------------------------------------*/
    _HeapSize = 2 * 1024;
    .heap :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM_ITC

/*---------------------------------------------------------------------------------
Stack
---------------------------------------------------------------------------------*/
     _StackSize = 2 * 1024;
     /* Reserve space in memory for Stack 16k */
    .heap2stackfill  :
    {
        . += _StackSize;
    } > SRAM_ITC
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM_ITC) + LENGTH(SRAM_ITC) - _StackSize - 0:  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM_ITC

	PROVIDE(_estack = _vStackTop);

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
  _heap_start  = _pvHeapStart;
  _heap_end    = _pvHeapLimit;
