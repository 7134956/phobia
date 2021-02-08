MEMORY
{
	ITCM_RAM1 (rwx)	: ORIGIN = 0x00000000, LENGTH = 16K
	ITCM_FLASH1 (rx): ORIGIN = 0x00200000, LENGTH = 512K
	FLASH1 (rx)		: ORIGIN = 0x08000000, LENGTH = 512K
	DTCM_RAM1 (rw)	: ORIGIN = 0x20000000, LENGTH = 64K
	RAM1 (rwx)		: ORIGIN = 0x20010000, LENGTH = 176K
	RAM2 (rwx)		: ORIGIN = 0x2003C000, LENGTH = 16K
	BKPSRAM (rw)	: ORIGIN = 0x40024000, LENGTH = 4k
}

ld_stack = ORIGIN(DTCM_RAM1) + LENGTH(DTCM_RAM1);

ENTRY(irq_Reset);

SECTIONS
{
	.text : ALIGN(4)
	{
		ld_begin_vectors = . ;

		KEEP(*(.vectors))
		. = ALIGN(4);

		*(.text)
		*(.text.*)

		*(.rodata)
		*(.rodata.*)

		. = ALIGN(4);
		ld_end_text = . ;
		ld_end_flash = . + SIZEOF(.data) ;

	} > FLASH1

	.data : AT (ld_end_text) ALIGN(4)
	{
		ld_begin_data = . ;

		*(.ramfunc)
		*(.ramfunc.*)

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		ld_end_data = . ;

	} > RAM1

	.bss (NOLOAD) : ALIGN(4)
	{
		ld_begin_bss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		ld_end_bss = . ;

	} > RAM1

	.noinit (NOLOAD) : ALIGN(4)
	{
		ld_begin_noinit = . ;

		*(.noinit)
		*(.noinit.*)

		. = ALIGN(4);
		ld_end_noinit = . ;

	} > DTCM_RAM1

	.ccmram (NOLOAD) : ALIGN(4)
	{
		ld_begin_ccm = . ;

		*(.ccmram)
		*(.ccmram.*)

		. = ALIGN(4);
		ld_end_ccm = . ;

	} > DTCM_RAM1

	/DISCARD/ : { *(.ARM.exidx*) }
}

