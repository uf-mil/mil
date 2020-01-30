/* Linker script for ST STM32F3DISCOVERY */
/* (STM32F303VC, 256K flash, 48K RAM (Of which 8kB are CCM RAM and 40KB are SRAM). */

/* Define memory regions. */
MEMORY
{
        rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
        client_rom (rx) : ORIGIN = 0x8010000, LENGTH = 192K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
}

PROVIDE(_begin_client_rom = ORIGIN(client_rom));
PROVIDE(_end_client_rom = ORIGIN(client_rom) + LENGTH(client_rom));

/* Include the common ld script. */
INCLUDE libopencm3_stm32f3.ld

