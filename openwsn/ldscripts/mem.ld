/*
 * Memory Spaces Definitions.
 *
 * Need modifying for a specific board. 
 *   FLASH.ORIGIN: starting address of flash
 *   FLASH.LENGTH: length of flash
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 *
 * The values below can be addressed in further linker scripts
 * using functions like 'ORIGIN(RAM)' or 'LENGTH(RAM)'.
 */


/* MEM_512K */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x200000, LENGTH = 0x0007FFD4
    FLASH_CCA (RX) : ORIGIN = 0x0027FFD4, LENGTH = 12
    SRAM (RWX) : ORIGIN = 0x20004000, LENGTH = 0x00004000
}



/* MEM_256K
MEMORY
{
    FLASH (rx) : ORIGIN = 0x200000, LENGTH = 0x0003FFD4
    FLASH_CCA (RX) : ORIGIN = 0x0023FFD4, LENGTH = 12
    SRAM (RWX) : ORIGIN = 0x20004000, LENGTH = 0x00004000
}
 
 */