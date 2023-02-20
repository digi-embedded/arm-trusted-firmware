/*
 * Copyright (c) 2017-2022, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include <arch_helpers.h>
#include <context.h>
#include <drivers/arm/tzc400.h>
#include <drivers/clk.h>
#include <drivers/regulator.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stm32mp1_ddr_regs.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <smccc_helpers.h>

#include <boot_api.h>
#include <platform_def.h>
#include <stm32mp1_context.h>
#include <stm32mp1_critic_power.h>

#define TRAINING_AREA_SIZE		64

#define BL32_CANARY_ID			U(0x424c3332)

/*
 * MAILBOX_MAGIC relates to struct backup_data_s as defined
 *
 * MAILBOX_MAGIC_V1:
 * Context provides magic, resume entry, zq0cr0 zdata and DDR training buffer.
 *
 * MAILBOX_MAGIC_V2:
 * Context provides V1 content and PLL1 dual OPP settings structure (86 bytes).
 *
 * MAILBOX_MAGIC_V3:
 * Context provides V2 content, low power entry point, BL2 code start, end and
 * BL2_END (102 bytes). And, only for STM32MP13, adds MCE master key (16 bytes).
 */
#define MAILBOX_MAGIC_V1		(0x0001 << 16)
#define MAILBOX_MAGIC_V2		(0x0002 << 16)
#define MAILBOX_MAGIC_V3		(0x0003 << 16)
#define MAILBOX_MAGIC			(MAILBOX_MAGIC_V3 | \
					 TRAINING_AREA_SIZE)

#define MAGIC_ID(magic)			((magic) & GENMASK_32(31, 16))
#define MAGIC_AREA_SIZE(magic)		((magic) & GENMASK_32(15, 0))

#if (PLAT_MAX_OPP_NB != 2) || (PLAT_MAX_PLLCFG_NB != 6)
#error MAILBOX_MAGIC_V2/_V3 does not support expected PLL1 settings
#endif

/* pll_settings structure size definitions (reference to clock driver) */
#define PLL1_SETTINGS_SIZE		(((PLAT_MAX_OPP_NB * \
					  (PLAT_MAX_PLLCFG_NB + 3)) + 1) * \
					 sizeof(uint32_t))

/* Set to 600 bytes to be a bit flexible but could be optimized if needed */
#define CLOCK_CONTEXT_SIZE		600

/* SCMI needs only 24 bits to save the state of the 24 exposed clocks */
#define SCMI_CONTEXT_SIZE		(sizeof(uint8_t) * 4)

struct backup_data_s {
	uint32_t magic;
	uint32_t core0_resume_hint;
	uint32_t zq0cr0_zdata;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
	uint8_t pll1_settings[PLL1_SETTINGS_SIZE];
	uint32_t low_power_ep;
	uint32_t bl2_code_base;
	uint32_t bl2_code_end;
	uint32_t bl2_end;
#if STM32MP13
	uint8_t mce_mkey[MCE_KEY_SIZE_IN_BYTES];
	struct stm32_mce_region_s mce_regions[MCE_IP_MAX_REGION_NB];
#endif
};

#if defined(IMAGE_BL32)
#define STM32MP1_TZC_REGION_OFFSET(num_region)		(U(0x20) * ((num_region) - U(1)))
#define STM32MP1_TZC_REGION_LOW(num_region)		(U(0x120) + \
							 STM32MP1_TZC_REGION_OFFSET(num_region))
#define STM32MP1_TZC_REGION_TOP(num_region)		(U(0x128) + \
							 STM32MP1_TZC_REGION_OFFSET(num_region))
#define STM32MP1_TZC_REGION_ATTRIBUTE(num_region)	(U(0x130) + \
							 STM32MP1_TZC_REGION_OFFSET(num_region))
#define STM32MP1_TZC_REGION_ACCESS(num_region)		(U(0x134) + \
							 STM32MP1_TZC_REGION_OFFSET(num_region))

struct tzc_regions {
	uint32_t low;
	uint32_t top;
	uint32_t attribute;
	uint32_t access;
};

struct backup_bl32_data_s {
	uint32_t canary_id;
	smc_ctx_t saved_smc_context[PLATFORM_CORE_COUNT];
	cpu_context_t saved_cpu_context[PLATFORM_CORE_COUNT];
	struct stm32_rtc_calendar rtc;
	unsigned long long stgen;
	uint8_t clock_cfg[CLOCK_CONTEXT_SIZE];
	uint8_t scmi_context[SCMI_CONTEXT_SIZE];
	uint8_t regul_context[PLAT_BACKUP_REGULATOR_SIZE];
	struct tzc_regions tzc_backup_context[STM32MP1_TZC_MAX_REGIONS];
};

static struct backup_bl32_data_s *get_bl32_backup_data(void)
{
	return (struct backup_bl32_data_s *)(STM32MP_BACKUP_RAM_BASE +
					     sizeof(struct backup_data_s));
}
#endif

uint32_t stm32_pm_get_optee_ep(void)
{
	struct backup_data_s *backup_data;
	uint32_t ep;

#if STM32MP15
	clk_enable(BKPSRAM);
#endif

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	switch (MAGIC_ID(backup_data->magic)) {
	case MAILBOX_MAGIC_V1:
	case MAILBOX_MAGIC_V2:
	case MAILBOX_MAGIC_V3:
		if (MAGIC_AREA_SIZE(backup_data->magic) != TRAINING_AREA_SIZE) {
			panic();
		}
		break;
	default:
		ERROR("PM context: bad magic\n");
		panic();
	}

	ep = backup_data->core0_resume_hint;

#if STM32MP15
	clk_disable(BKPSRAM);
#endif

	return ep;
}

void stm32_clean_context(void)
{
	clk_enable(BKPSRAM);

#if defined(IMAGE_BL2)
	zeromem((void *)STM32MP_BACKUP_RAM_BASE, sizeof(struct backup_data_s));
#elif defined(IMAGE_BL32)
	zeromem((void *)get_bl32_backup_data(), sizeof(struct backup_bl32_data_s));
#endif

	clk_disable(BKPSRAM);
}

#if defined(IMAGE_BL32)
void stm32mp1_pm_save_clock_cfg(size_t offset, uint8_t *data, size_t size)
{
	struct backup_bl32_data_s *backup_data = get_bl32_backup_data();

	if (offset + size > sizeof(backup_data->clock_cfg)) {
		panic();
	}

	clk_enable(BKPSRAM);

	memcpy(backup_data->clock_cfg + offset, data, size);

	clk_disable(BKPSRAM);
}

void stm32mp1_pm_restore_clock_cfg(size_t offset, uint8_t *data, size_t size)
{
	struct backup_bl32_data_s *backup_data = get_bl32_backup_data();

	if (offset + size > sizeof(backup_data->clock_cfg))
		panic();

	clk_enable(BKPSRAM);

	memcpy(data, backup_data->clock_cfg + offset, size);

	clk_disable(BKPSRAM);
}

static void tzc_backup_context_save(struct tzc_regions *regions)
{
	unsigned int i;

	assert(regions != NULL);

	zeromem(regions, sizeof(struct tzc_regions) * STM32MP1_TZC_MAX_REGIONS);

	tzc400_init(STM32MP1_TZC_BASE);

	for (i = 0U; i < STM32MP1_TZC_MAX_REGIONS; i++) {
		unsigned int reg_num = i + 1U;

		regions[i].low = mmio_read_32(STM32MP1_TZC_BASE +
					      STM32MP1_TZC_REGION_LOW(reg_num));
		regions[i].top = mmio_read_32(STM32MP1_TZC_BASE +
					      STM32MP1_TZC_REGION_TOP(reg_num));
		regions[i].attribute = mmio_read_32(STM32MP1_TZC_BASE +
						    STM32MP1_TZC_REGION_ATTRIBUTE(reg_num));
		regions[i].access = mmio_read_32(STM32MP1_TZC_BASE +
						 STM32MP1_TZC_REGION_ACCESS(reg_num));
	}
}

static void tzc_backup_context_restore(struct tzc_regions *regions)
{
	unsigned int i;

	assert(regions != NULL);

	tzc400_init(STM32MP1_TZC_BASE);

	for (i = 0U; i < STM32MP1_TZC_MAX_REGIONS; i++) {
		unsigned int reg_num = i + 1U;

		mmio_write_32(STM32MP1_TZC_BASE + STM32MP1_TZC_REGION_LOW(reg_num),
			      regions[i].low);
		mmio_write_32(STM32MP1_TZC_BASE + STM32MP1_TZC_REGION_TOP(reg_num),
			      regions[i].top);
		mmio_write_32(STM32MP1_TZC_BASE + STM32MP1_TZC_REGION_ATTRIBUTE(reg_num),
			      regions[i].attribute);
		mmio_write_32(STM32MP1_TZC_BASE + STM32MP1_TZC_REGION_ACCESS(reg_num),
			      regions[i].access);
	}

	/* Disable Region 0 access */
	tzc400_configure_region0(TZC_REGION_S_NONE, 0);
}

int stm32_save_context(uint32_t zq0cr0_zdata,
		       struct stm32_rtc_calendar *rtc_time,
		       unsigned long long stgen_cnt)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;
	struct backup_bl32_data_s *backup_bl32_data;

	stm32mp1_clock_suspend();

	clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	/* Save BL32 context data provided to BL2 */
	backup_data->magic = MAILBOX_MAGIC;

	backup_data->zq0cr0_zdata = zq0cr0_zdata;

	stm32mp1_clk_lp_save_opp_pll1_settings(backup_data->pll1_settings,
					       sizeof(backup_data->pll1_settings));

	/* Save the BL32 specific data */
	backup_bl32_data = get_bl32_backup_data();

	backup_bl32_data->canary_id = BL32_CANARY_ID;

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	/* Save context in Backup SRAM */
	memcpy(&backup_bl32_data->saved_smc_context[0], smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(&backup_bl32_data->saved_cpu_context[0], cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	memcpy(&backup_bl32_data->rtc, rtc_time, sizeof(struct stm32_rtc_calendar));
	backup_bl32_data->stgen = stgen_cnt;

	stm32mp1_pm_save_scmi_state(backup_bl32_data->scmi_context,
				    sizeof(backup_bl32_data->scmi_context));

	save_clock_pm_context();

	regulator_core_backup_context(backup_bl32_data->regul_context,
				      sizeof(backup_bl32_data->regul_context));

	tzc_backup_context_save(backup_bl32_data->tzc_backup_context);

	clk_disable(BKPSRAM);

	return 0;
}

int stm32_restore_context(void)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;
	struct backup_bl32_data_s *backup_bl32_data;
	struct stm32_rtc_calendar current_calendar;
	unsigned long long stdby_time_in_ms;

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	stm32mp1_clk_lp_load_opp_pll1_settings(backup_data->pll1_settings,
					       sizeof(backup_data->pll1_settings));

	backup_bl32_data = get_bl32_backup_data();

	if (backup_bl32_data->canary_id != BL32_CANARY_ID) {
		ERROR("Incorrect BL32 backup data\n");
		return -EINVAL;
	}

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	restore_clock_pm_context();

	tzc_backup_context_restore(backup_bl32_data->tzc_backup_context);

	stm32mp1_pm_restore_scmi_state(backup_bl32_data->scmi_context,
				       sizeof(backup_bl32_data->scmi_context));

	/* Restore data from Backup SRAM */
	memcpy(smc_context, backup_bl32_data->saved_smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(cpu_context, backup_bl32_data->saved_cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	/* Restore STGEN counter with standby mode length */
	stm32_rtc_get_calendar(&current_calendar);
	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &backup_bl32_data->rtc);
	stm32mp_stgen_restore_counter(backup_bl32_data->stgen, stdby_time_in_ms);

	regulator_core_restore_context(backup_bl32_data->regul_context,
				       sizeof(backup_bl32_data->regul_context));

	clk_disable(BKPSRAM);

	stm32mp1_clock_resume();

	return 0;
}

unsigned long long stm32_get_stgen_from_context(void)
{
	struct backup_bl32_data_s *backup_data;
	unsigned long long stgen_cnt;

	clk_enable(BKPSRAM);

	backup_data = get_bl32_backup_data();

	stgen_cnt = backup_data->stgen;

	clk_disable(BKPSRAM);

	return stgen_cnt;
}

void stm32_context_get_bl2_low_power_params(uintptr_t *bl2_code_base,
					    uintptr_t *bl2_code_end,
					    uintptr_t *bl2_end)
{
	struct backup_data_s *backup_data;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	if (MAGIC_ID(backup_data->magic) != MAILBOX_MAGIC_V3) {
		panic();
	}

	*bl2_code_base = (uintptr_t)backup_data->bl2_code_base;
	*bl2_code_end = (uintptr_t)backup_data->bl2_code_end;
	*bl2_end = (uintptr_t)backup_data->bl2_end;

	clk_disable(BKPSRAM);
}

#endif /* IMAGE_BL32 */

#if defined(IMAGE_BL2)
void stm32_context_save_bl2_param(void)
{
	struct backup_data_s *backup_data;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	backup_data->low_power_ep = (uint32_t)&stm32_pwr_down_wfi_wrapper;
	backup_data->bl2_code_base = BL_CODE_BASE;
	backup_data->bl2_code_end = BL_CODE_END;
	backup_data->bl2_end = BL2_END;
	backup_data->magic = MAILBOX_MAGIC_V3;
	backup_data->zq0cr0_zdata = ddr_get_io_calibration_val();

	clk_disable(BKPSRAM);
}
#endif

uint32_t stm32_get_zdata_from_context(void)
{
	struct backup_data_s *backup_data;
	uint32_t zdata;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	zdata = (backup_data->zq0cr0_zdata >> DDRPHYC_ZQ0CRN_ZDATA_SHIFT) &
		DDRPHYC_ZQ0CRN_ZDATA_MASK;

	clk_disable(BKPSRAM);

	return zdata;
}

static int pll1_settings_in_context(struct backup_data_s *backup_data)
{
	switch (MAGIC_ID(backup_data->magic)) {
	case MAILBOX_MAGIC_V1:
		return -ENOENT;
	case MAILBOX_MAGIC_V2:
	case MAILBOX_MAGIC_V3:
		assert(MAGIC_AREA_SIZE(backup_data->magic) ==
		       TRAINING_AREA_SIZE);
		return 0;
	default:
		panic();
	}
}

int stm32_get_pll1_settings_from_context(void)
{
	struct backup_data_s *backup_data;
	int ret;

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	ret = pll1_settings_in_context(backup_data);
	if (ret == 0) {
		uint8_t *data = (uint8_t *)backup_data->pll1_settings;
		size_t size = sizeof(backup_data->pll1_settings);

		stm32mp1_clk_lp_load_opp_pll1_settings(data, size);
	}

	clk_disable(BKPSRAM);

	return ret;
}

bool stm32_are_pll1_settings_valid_in_context(void)
{
	struct backup_data_s *backup_data;
	uint32_t *data;
	bool is_valid;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;
	data = (uint32_t *)backup_data->pll1_settings;

	is_valid = (data[0] == PLL1_SETTINGS_VALID_ID);

	clk_disable(BKPSRAM);

	return is_valid;
}

bool stm32_pm_context_is_valid(void)
{
	struct backup_data_s *backup_data;
	bool ret;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	switch (MAGIC_ID(backup_data->magic)) {
	case MAILBOX_MAGIC_V1:
	case MAILBOX_MAGIC_V2:
	case MAILBOX_MAGIC_V3:
		ret = true;
		break;
	default:
		ret = false;
		break;
	}

	clk_disable(BKPSRAM);

	return ret;
}

#if defined(IMAGE_BL32)
/*
 * When returning from STANDBY, the 64 first bytes of DDR will be overwritten
 * during DDR DQS training. This area must then be saved before going to
 * standby, and will be restored after
 */
void stm32_save_ddr_training_area(void)
{
	struct backup_data_s *backup_data;
	int ret __unused;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	ret = mmap_add_dynamic_region(STM32MP_DDR_BASE, STM32MP_DDR_BASE,
				      PAGE_SIZE, MT_MEMORY | MT_RW | MT_NS);
	assert(ret == 0);

	flush_dcache_range(STM32MP_DDR_BASE, TRAINING_AREA_SIZE);

	memcpy(&backup_data->ddr_training_backup,
	       (const uint32_t *)STM32MP_DDR_BASE,
	       TRAINING_AREA_SIZE);
	dsb();

	ret = mmap_remove_dynamic_region(STM32MP_DDR_BASE, PAGE_SIZE);
	assert(ret == 0);

	clk_disable(BKPSRAM);
}
#endif

void stm32_restore_ddr_training_area(void)
{
	struct backup_data_s *backup_data;

	clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	memcpy((uint32_t *)STM32MP_DDR_BASE,
	       &backup_data->ddr_training_backup,
	       TRAINING_AREA_SIZE);
	dsb();

	clk_disable(BKPSRAM);
}

#if STM32MP13
void stm32mp1_pm_save_mce_mkey_in_context(uint8_t *data)
{
	struct backup_data_s *backup_data;

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	memcpy(backup_data->mce_mkey, data, MCE_KEY_SIZE_IN_BYTES);

	clk_disable(BKPSRAM);
}

void stm32mp1_pm_get_mce_mkey_from_context(uint8_t *data)
{
	struct backup_data_s *backup_data;

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	memcpy(data, backup_data->mce_mkey, MCE_KEY_SIZE_IN_BYTES);

	clk_disable(BKPSRAM);
}

void stm32mp1_pm_save_mce_region(uint32_t index, struct stm32_mce_region_s *config)
{
	struct backup_data_s *backup_data;

	if (index >= MCE_IP_MAX_REGION_NB) {
		panic();
	}

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	memcpy(&backup_data->mce_regions[index], config, sizeof(struct stm32_mce_region_s));

	clk_disable(BKPSRAM);
}

void stm32mp1_pm_get_mce_region(uint32_t index, struct stm32_mce_region_s *config)
{
	struct backup_data_s *backup_data;

	if (index >= MCE_IP_MAX_REGION_NB) {
		panic();
	}

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	clk_enable(BKPSRAM);

	memcpy(config, &backup_data->mce_regions[index], sizeof(struct stm32_mce_region_s));

	clk_disable(BKPSRAM);
}
#endif
