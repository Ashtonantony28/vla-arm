/* STM32G474RE: 512 KB flash, 96 KB SRAM (SRAM1 80 KB + SRAM2 16 KB, contiguous at 0x20000000),
 * 32 KB CCM SRAM at 0x10000000 (also aliased at 0x20018000; not used yet). */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 508K   /* last 4 KB: calibration store */
  CALIB (r)   : ORIGIN = 0x0807F000, LENGTH = 4K     /* bank 2 pages 126-127 (flash_stm32g4.c) */
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
  CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 32K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x1000; /* 4 KB; control_step peak stack is reported by -fstack-usage */

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
  } > FLASH

  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  .stack_check (NOLOAD) :
  {
    . = ALIGN(8);
    . = . + _min_stack;
    . = ALIGN(8);
  } > RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
