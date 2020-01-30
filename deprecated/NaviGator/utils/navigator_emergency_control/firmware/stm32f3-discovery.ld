/* Linker script for ST STM32F3DISCOVERY */
/* (STM32F303VC, 256K flash, 48K RAM (Of which 8kB are CCM RAM and 40KB are SRAM). */

/* Define memory regions. */
MEMORY
{
        boot_rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
        rom (rx) : ORIGIN = 0x8010000, LENGTH = 192K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f3.ld

