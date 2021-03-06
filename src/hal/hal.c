#include <stddef.h>

#include "hal.h"
#include "libc.h"

#ifdef STM32F7
#define CLOCK_CPU_TARGET_HZ		216000000UL
#define SYS_MEM_ADDR			0x1FF00000UL //On AXIM interface
//#define SYS_MEM_ADDR			0x00100000UL //On ICTM interface
#else
#define CLOCK_CPU_TARGET_HZ		168000000UL
#define SYS_MEM_ADDR			0x1FFF0000UL
#endif

unsigned long			clock_cpu_hz;

HAL_t				hal;
LOG_t				log		LD_NOINIT;
volatile int			bootload_jump	LD_NOINIT;

void irq_NMI()
{
	log_TRACE("IRQ: NMI" EOL);

	hal_system_reset();
}

void irq_HardFault()
{
	log_TRACE("IRQ: HardFault" EOL);

	hal_system_reset();
}

void irq_MemoryFault()
{
	log_TRACE("IRQ: MemoryFault" EOL);

	hal_system_reset();
}

void irq_BusFault()
{
	log_TRACE("IRQ: BusFault" EOL);

	hal_system_reset();
}

void irq_UsageFault()
{
	log_TRACE("IRQ: UsageFault" EOL);

	hal_system_reset();
}

void irq_Default()
{
	log_TRACE("IRQ: Default" EOL);

	hal_system_reset();
}

static void
base_startup()
{
	/* Enable FPU.
	 * */
	SCB->CPACR |= (3UL << 20) | (3UL << 22);

	/* Vector table offset.
	 * */
	SCB->VTOR = (u32_t) &ld_begin_vectors;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(0UL);
}

static uint32_t
get_HSE_clock(void) {
	uint32_t clock;			//������� ���������� � ����������
	uint16_t cap_value;		//�������� �������

	RCC->CFGR |= 26 << RCC_CFGR_RTCPRE_Pos; //HSE_RTC at 1 MHz max.

	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; //�������� ������������ ������� ������ �������

#ifdef STM32F7
	TIM11->OR = TIM11_OR_TI1_RMP_1;
#else
	TIM11->OR = TIM_OR_TI1_RMP_1;	//HSE_RTC clock ��� ������ �������; ��������: 26 * 8 = 208
#endif
	TIM11->CCMR1 =					//��������� ���������� IC1 -> TI1, ��������� / 8
		TIM_CCMR1_CC1S_0 |
		TIM_CCMR1_IC1PSC_1 |
		TIM_CCMR1_IC1PSC_0;
	TIM11->CCER = TIM_CCER_CC1E;	//��������� ����� �������
	TIM11->CR1 = TIM_CR1_CEN;		//������ �������

	while(!(TIM11->SR & TIM_SR_UIF));	//���� ��� �������� ������������
	TIM11->SR = 0;						//������� ����� UIF � CC1F

	while(!(TIM11->SR & TIM_SR_CC1IF) && !(TIM11->SR & TIM_SR_UIF)); //�������� �������
	if(TIM11->SR & TIM_SR_UIF) {
		clock = 0;			//��� ���������
	} else {
		cap_value = TIM11->CCR1;//������ �������� �������
		TIM11->SR = 0;			//������� ����� UIF � CC1F
		while(!(TIM11->SR & TIM_SR_CC1IF));	//�������� �������
		cap_value = TIM11->CCR1 - cap_value;//������� �������
		clock = (208 * 16 + 80) / cap_value;//��������� � 2%. ������
	}

	//�������� ��������� � ��������� ������ 11
	TIM11->CR1 = 0;
	TIM11->CCMR1 = 0;
	TIM11->CCER = 0;
	TIM11->OR = 0;
	TIM11->CNT = 0;
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;

	return clock * 1000000;
}

static void
clock_startup()
{
	u32_t		CLOCK, PLLQ, PLLP, PLLN, PLLM;
	int		HSE, N = 0;

#ifdef STM32F7
	SCB_EnableICache();	// Enable I-Cache
	SCB_EnableDCache();	// Enable D-Cache
	RCC->DCKCFGR2 = 0;	// Need if return from system bootloader
#endif

	/* Enable HSI.
	 * */
	RCC->CR |= RCC_CR_HSION;

	/* Reset RCC.
	 * */
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);
	RCC->CFGR = 0;
	RCC->CIR = 0;

	/* Enable HSE.
	 * */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready.
	 * */
	do {
		HSE = RCC->CR & RCC_CR_HSERDY;
		N++;

		__NOP();
	}
	while (HSE == 0 && N < 40000UL);

	/* Enable power interface clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Regulator voltage scale 1 mode.
	 * */
#ifdef STM32F7
	PWR->CR1 = PWR_CR1_VOS_1;
#else
	PWR->CR |= PWR_CR_VOS;
#endif

	/* Set AHB/APB1/APB2 prescalers.
	 * */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	if (HSE != 0) {

		CLOCK = get_HSE_clock();

		/* From HSE.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE;
	}
	else {
		CLOCK = 16000000UL;

		/* From HSI.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI;

		log_TRACE("HSE failed" EOL);
	}

	PLLP = 2;

	PLLM = (CLOCK + 1999999UL) / 2000000UL;
	CLOCK /= PLLM;

	PLLN = (PLLP * CLOCK_CPU_TARGET_HZ) / CLOCK;
	CLOCK *= PLLN;

	PLLQ = (CLOCK + 47999999UL) / 48000000UL;

	RCC->PLLCFGR |= (PLLQ << 24)
		| ((PLLP / 2UL - 1UL) << 16)
		| (PLLN << 6) | (PLLM << 0);

	/* Define clock frequency.
	 * */
	clock_cpu_hz = CLOCK / PLLP;

	/* Enable PLL.
	 * */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready.
	 * */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {

		__NOP();
	}

	/* Configure Flash.
	 * */
	FLASH->ACR =
#ifndef STM32F7
	FLASH_ACR_DCEN | FLASH_ACR_ICEN |
#endif
	FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	/* Select PLL.
	 * */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till PLL is used.
	 * */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {

		__NOP();
	}
}

static void
periph_startup()
{
	/* Enable Programmable voltage detector.
	 * */
#ifdef STM32F7
	PWR->CR1 |= PWR_CR1_PLS_LEV7 | PWR_CR1_PVDE;
#else
	PWR->CR |= PWR_CR_PLS_LEV7 | PWR_CR_PVDE;
#endif

	/* Enable LSI.
	 * */
	RCC->CSR |= RCC_CSR_LSION;

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
		| RCC_AHB1ENR_GPIOCEN;

	/* Check for reset reason.
	 * */
	if (RCC->CSR & RCC_CSR_WWDGRSTF) {

		log_TRACE("RESET: WD" EOL);
	}

	RCC->CSR |= RCC_CSR_RMVF;
}

void hal_bootload()
{
	if (bootload_jump == INIT_SIGNATURE) {

		bootload_jump = 0;

		/*SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;

		__DSB();
		__ISB();*/

		/* Load MSP.
		 * */
		__set_MSP(* (u32_t *) SYS_MEM_ADDR);

		/* Jump to the bootloader.
		 * */
		((void (*) (void)) (* (u32_t *) (SYS_MEM_ADDR + 4))) ();
	}
}

void hal_startup()
{
	DBGMCU_mode_stop();
	base_startup();
	clock_startup();
	periph_startup();
}

void hal_delay_ns(int ns)
{
	u32_t			xVAL, xLOAD;
	int			elapsed, tickval;

	xVAL = SysTick->VAL;
	xLOAD = SysTick->LOAD + 1UL;

	tickval = ns * (clock_cpu_hz / 1000000UL) / 1000UL;

	do {
		elapsed = (int) (xVAL - SysTick->VAL);
		elapsed += (elapsed < 0) ? xLOAD : 0;

		if (elapsed >= tickval)
			break;

		__NOP();
	}
	while (1);
}

int hal_lock_irq()
{
	int		irq;

	irq = __get_BASEPRI();
	__set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));

	return irq;
}

void hal_unlock_irq(int irq)
{
	__set_BASEPRI(irq);
}

void hal_system_reset()
{
	NVIC_SystemReset();
}

void hal_bootload_jump()
{
	bootload_jump = INIT_SIGNATURE;

	NVIC_SystemReset();
}

void hal_sleep()
{
	__WFI();
}

void hal_fence()
{
	__DMB();
}

int log_bootup()
{
	int		rc = 0;

	if (log.boot_SIGNATURE != INIT_SIGNATURE) {

		log.boot_SIGNATURE = INIT_SIGNATURE;
		log.boot_COUNT = 0;

		/* Initialize LOG at first usage.
		 * */
		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.tail = 0;
	}
	else {
		log.boot_COUNT += 1;

		/* Make sure LOG is null-terminated.
		 * */
		log.textbuf[sizeof(log.textbuf) - 1] = 0;

		rc = (log.textbuf[0] != 0) ? 1 : 0;
	}

	return rc;
}

void log_putc(int c)
{
	const int	tailmax = sizeof(log.textbuf) - 1;

	log.tail = (log.tail >= 0 && log.tail < tailmax) ? log.tail : 0;
	log.textbuf[log.tail] = (char) c;

	++log.tail;
}

void DBGMCU_mode_stop()
{
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
	DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
}

