#ifndef _STM32_H_
#define _STM32_H_

/* modify bitfield */
#define _BMD(reg, msk, val)     (reg) = (((reg) & ~(msk)) | (val))
/* set bitfield */
#define _BST(reg, bits)         (reg) = ((reg) | (bits))
/* clear bitfield */
#define _BCL(reg, bits)         (reg) = ((reg) & ~(bits))
/* wait until bitfield set */
#define _WBS(reg, bits)         while(((reg) & (bits)) == 0)
/* wait until bitfield clear */
#define _WBC(reg, bits)         while(((reg) & (bits)) != 0)
/* wait for bitfield value */
#define _WVL(reg, msk, val)     while(((reg) & (msk)) != (val))
/* bit value */
#define _BV(bit)                (0x01 << (bit))

#if defined(STM32F7) || defined(_HW_PESC1)
	#define STM32F722xx
	#include "../hal/cmsis/stm32f7xx.h"
#else
	#define STM32F405xx
	#include "../hal/cmsis/stm32f4xx.h"
#endif

#define		USBD_SOF_DISABLED

#endif // _STM32_H_
