/*
 * Copyright (c) 2017-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <libfdt.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/clk.h>
#include <drivers/delay_timer.h>
#include <drivers/regulator.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stm32mp1_pwr.h>
#include <drivers/st/stm32mp1_rcc.h>
#include <drivers/st/stpmic1.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <lib/spinlock.h>
#include <plat/common/platform.h>

#include <boot_api.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp1_context.h>
#include <stm32mp1_low_power.h>
#include <stm32mp1_power_config.h>
#include <stm32mp1_private.h>

static unsigned int gicc_pmr;
static struct stm32_rtc_calendar sleep_time;
static bool enter_cstop_done;
static unsigned long long stgen_cnt;

struct pwr_lp_config {
	uint32_t pwr_cr1;
	uint32_t pwr_mpucr;
	const char *regul_suspend_node_name;
};

#define PWRLP_TEMPO_5_HSI	5

#define PWR_CR1_MASK	(PWR_CR1_LPDS | PWR_CR1_LPCFG | PWR_CR1_LVDS)
#define PWR_MPUCR_MASK	(PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF | PWR_MPUCR_PDDS)

static const struct pwr_lp_config config_pwr[STM32_PM_MAX_SOC_MODE] = {
	[STM32_PM_CSLEEP_RUN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSSF,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_STOP] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_LP_STOP] = {
		.pwr_cr1 = PWR_CR1_LPDS,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lp-stop",
	},
	[STM32_PM_CSTOP_ALLOW_LPLV_STOP] = {
		.pwr_cr1 = PWR_CR1_LVDS | PWR_CR1_LPDS | PWR_CR1_LPCFG,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lplv-stop",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-sr",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_OFF] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-off",
	},
	[STM32_PM_SHUTDOWN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = 0U,
		.regul_suspend_node_name = "standby-ddr-off",
	},
};

#define GICC_PMR_PRIORITY_8	U(0x8)

enum {
	STATE_NONE = 0,
	STATE_AUTOSTOP_ENTRY,
	STATE_AUTOSTOP_EXIT,
};

static struct spinlock lp_lock;
static volatile int cpu0_state = STATE_NONE;
static volatile int cpu1_state = STATE_NONE;

const char *plat_get_lp_mode_name(int mode)
{
	return config_pwr[mode].regul_suspend_node_name;
}

void stm32_apply_pmic_suspend_config(uint32_t mode)
{
	const char *node_name;

	assert(mode < ARRAY_SIZE(config_pwr));

	node_name = config_pwr[mode].regul_suspend_node_name;

	if (node_name != NULL) {
		if (!initialize_pmic_i2c()) {
			panic();
		}
	}
}

/*
 * stm32_enter_cstop - Prepare CSTOP mode
 *
 * @mode - Target low power mode
 * @nsec_addr - Non secure resume entry point
 * Return 0 if succeed to suspend, non 0 else.
 */
static void enter_cstop(uint32_t mode, uint32_t nsec_addr)
{
	uint32_t zq0cr0_zdata;
	uintptr_t bkpr_core1_addr =
		tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uintptr_t bkpr_core1_magic =
		tamp_bkpr(BOOT_API_CORE1_MAGIC_NUMBER_TAMP_BCK_REG_IDX);
	uint32_t pwr_cr1 = config_pwr[mode].pwr_cr1;
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();

#ifdef STM32MP15
	if (mode == STM32_PM_CSTOP_ALLOW_LPLV_STOP2) {
		ERROR("LPLV-Stop2 mode not supported\n");
		panic();
	}
#endif

	stm32mp1_syscfg_disable_io_compensation();

	stm32_clean_context();

	if (mode == STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR) {
		/*
		 * The first 64 bytes of DDR need to be saved for DDR DQS
		 * training
		 */
		stm32_save_ddr_training_area();
	}

	if (dt_pmic_status() > 0) {
		stm32_apply_pmic_suspend_config(mode);

		if (mode == STM32_PM_CSTOP_ALLOW_LP_STOP) {
			pwr_cr1 |= PWR_CR1_LPCFG;
		}
	}

	regulator_core_suspend(mode);

	/* Clear RCC interrupt before enabling it */
	mmio_setbits_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_WKUPF);

	/* Enable RCC Wake-up */
	mmio_setbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	/* Configure low power mode */
	mmio_clrsetbits_32(pwr_base + PWR_MPUCR, PWR_MPUCR_MASK,
			   config_pwr[mode].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1, PWR_CR1_MASK,
			   pwr_cr1);

	/* Clear RCC pending interrupt flags */
	mmio_write_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_MASK);

	/* Request CSTOP mode to RCC */
	mmio_setbits_32(rcc_base + RCC_MP_SREQSETR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	stm32_iwdg_refresh();

	gicc_pmr = plat_ic_set_priority_mask(GICC_PMR_PRIORITY_8);

	zq0cr0_zdata = ddr_get_io_calibration_val();

	clk_enable(RTCAPB);

	mmio_write_32(bkpr_core1_addr, 0);
	mmio_write_32(bkpr_core1_magic, 0);

	stm32mp1_clock_stopmode_save();

	stm32_rtc_get_calendar(&sleep_time);
	stgen_cnt = stm32mp_stgen_get_counter();

	if (mode == STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR) {
		/*
		 * Save non-secure world entrypoint after standby in Backup
		 * register
		 */
		mmio_write_32(bkpr_core1_addr, nsec_addr);
		mmio_write_32(bkpr_core1_magic,
			      BOOT_API_A7_CORE0_MAGIC_NUMBER);

		if (stm32_save_context(zq0cr0_zdata, &sleep_time,
				       stgen_cnt) != 0) {
			panic();
		}

		/* set POPL to 20ms */
		mmio_clrsetbits_32(pwr_base + PWR_CR3, PWR_CR3_POPL_MASK,
				   20U << PWR_CR3_POPL_SHIFT);

		if (stm32mp1_get_retram_enabled()) {
			mmio_setbits_32(pwr_base + PWR_CR2, PWR_CR2_RREN);
			while ((mmio_read_32(pwr_base + PWR_CR2) &
				PWR_CR2_RRRDY) == 0U) {
				;
			}
		}
	}

	clk_disable(RTCAPB);

	enter_cstop_done = true;
}

bool stm32_is_cstop_done(void)
{
	return enter_cstop_done;
}

/*
 * stm32_exit_cstop - Exit from CSTOP mode
 */
void stm32_exit_cstop(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();
	unsigned long long stdby_time_in_ms;
	struct stm32_rtc_calendar current_calendar;

	if (!enter_cstop_done) {
		return;
	}

	enter_cstop_done = false;

	stm32mp1_syscfg_enable_io_compensation_start();

	plat_ic_set_priority_mask(gicc_pmr);

	/* Disable RCC Wake-up */
	mmio_clrbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	/* Disable STOP request */
	mmio_setbits_32(rcc_base + RCC_MP_SREQCLRR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	dsb();
	isb();

	/* Disable retention RAM content after stop */
	mmio_clrbits_32(pwr_base + PWR_CR2, PWR_CR2_RREN);

	/* Update STGEN counter with low power mode duration */
	stm32_rtc_get_calendar(&current_calendar);

	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &sleep_time);
	stm32mp_stgen_restore_counter(stgen_cnt, stdby_time_in_ms);

	if (stm32mp1_clock_stopmode_resume() != 0) {
		panic();
	}

	regulator_core_resume();

	stm32mp1_syscfg_enable_io_compensation_finish();
}

static int get_locked(volatile int *state)
{
	volatile int val;

	spin_lock(&lp_lock);
	val = *state;
	spin_unlock(&lp_lock);

	return val;
}

static void set_locked(volatile int *state, int val)
{
	spin_lock(&lp_lock);
	*state = val;
	spin_unlock(&lp_lock);
}

static void smp_synchro(int state, bool wake_up)
{
	/* if the other CPU is stopped, no need to synchronize */
	if (psci_is_last_on_cpu() == 1U) {
		return;
	}

	if (plat_my_core_pos() == STM32MP_PRIMARY_CPU) {
		set_locked(&cpu0_state, state);

		while (get_locked(&cpu1_state) != state) {
			if (wake_up) {
				/* wakeup secondary CPU */
				gicv2_raise_sgi(ARM_IRQ_SEC_SGI_6,
						STM32MP_SECONDARY_CPU);
				udelay(10);
			}
		};
	} else {
		while (get_locked(&cpu0_state) != state) {
			if (wake_up) {
				/* wakeup primary CPU */
				gicv2_raise_sgi(ARM_IRQ_SEC_SGI_6,
						STM32MP_PRIMARY_CPU);
				udelay(10);
			}
		};

		set_locked(&cpu1_state, state);
	}
}

static void stm32_auto_stop_cpu0(void)
{
	smp_synchro(STATE_AUTOSTOP_ENTRY, false);

	enter_cstop(STM32_PM_CSTOP_ALLOW_LP_STOP, 0);

	stm32_pwr_down_wfi(true, STM32_PM_CSTOP_ALLOW_LP_STOP);

	stm32_exit_cstop();

	smp_synchro(STATE_AUTOSTOP_EXIT, true);
}

static void stm32_auto_stop_cpu1(void)
{
	unsigned int gicc_pmr_cpu1;

	/* clear cache before the DDR is being disabled by cpu0 */
	dcsw_op_all(DC_OP_CISW);

	smp_synchro(STATE_AUTOSTOP_ENTRY, false);

	gicc_pmr_cpu1 = plat_ic_set_priority_mask(GICC_PMR_PRIORITY_8);
	wfi();
	plat_ic_set_priority_mask(gicc_pmr_cpu1);

	smp_synchro(STATE_AUTOSTOP_EXIT, true);
}

void stm32_auto_stop(void)
{
	if (plat_my_core_pos() == STM32MP_PRIMARY_CPU) {
		stm32_auto_stop_cpu0();
	} else {
		stm32_auto_stop_cpu1();
	}
}

static void enter_csleep(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();

	mmio_clrsetbits_32(pwr_base + PWR_MPUCR, PWR_MPUCR_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1, PWR_CR1_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_cr1);

	stm32_pwr_down_wfi(false, STM32_PM_CSLEEP_RUN);
}

void stm32_enter_low_power(uint32_t mode, uint32_t nsec_addr)
{
	switch (mode) {
	case STM32_PM_SHUTDOWN:
		break;

	case STM32_PM_CSLEEP_RUN:
		enter_csleep();
		break;

	default:
		enter_cstop(mode, nsec_addr);
		break;
	}
}

void stm32_init_low_power(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();

	/*
	 * Configure Standby mode available for MCU by default
	 * and allow to switch in standby SoC in all case
	 */
	mmio_setbits_32(pwr_base + PWR_MCUCR, PWR_MCUCR_PDDS);

	/* Disable STOP request */
	mmio_setbits_32(rcc_base + RCC_MP_SREQCLRR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	/* Disable retention RAM content after standby */
	mmio_clrbits_32(pwr_base + PWR_CR2, PWR_CR2_RREN);

	/* Wait 5 HSI periods before re-enabling PLLs after STOP modes */
	mmio_clrsetbits_32(rcc_base + RCC_PWRLPDLYCR,
			   RCC_PWRLPDLYCR_PWRLP_DLY_MASK,
			   PWRLP_TEMPO_5_HSI);

	/* Keep backup RAM content in standby and VBAT mode */
	mmio_setbits_32(pwr_base + PWR_CR2, PWR_CR2_BREN);
	while ((mmio_read_32(pwr_base + PWR_CR2) &
		PWR_CR2_BRRDY) == 0U) {
		;
	}
}
