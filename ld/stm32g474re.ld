/* STM32G474RE: 512 KB flash (last 4 KB reserved for config A/B), 128 KB SRAM
 * (SRAM1 80K + SRAM2 16K contiguous at 0x20000000, CCM 32K separately). */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 508K
  CONFIG (r)  : ORIGIN = 0x0807F000, LENGTH = 4K
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 96K
  CCMRAM (xrw): ORIGIN = 0x10000000, LENGTH = 32K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x1000;

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

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

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
    PROVIDE(end = .);   /* heap start for newlib's _sbrk (firmware itself never mallocs) */
    PROVIDE(_end = .);
  } > RAM

  ._stack_check (NOLOAD) : { . = ALIGN(8); . = . + _min_stack; } > RAM

}
