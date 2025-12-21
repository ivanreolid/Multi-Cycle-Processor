OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
  /* 1. El código empieza en la dirección 0x00000000 */
  . = 0x00000000;
  .text.init : { *(.text.init) }
  .text : { *(.text) }

  /* 2. Datos */
  .data : { *(.data) }

  /* 3. Definimos una dirección FIJA para 'tohost' */
  /* Esto es un truco para que tu Testbench sepa qué dirección espiar */
  . = 0x00001000; 
  .tohost : { *(.tohost) }

  .bss : { *(.bss) }
  _end = .;
}
