/*
 * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#include <drivers/clk.h>
#include <drivers/delay_timer.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/mmc.h>
#include <drivers/st/bsec.h>
#include <drivers/st/regulator.h>
#include <drivers/st/regulator_fixed.h>
#include <drivers/st/stm32_console.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32_rifsc.h>
#include <drivers/st/stm32_rng.h>
#include <drivers/st/stm32mp_pmic2.h>
#include <drivers/st/stm32mp_reset.h>
#include <drivers/st/stm32mp_rifsc_regs.h>
#include <drivers/st/stm32mp_risab_regs.h>
#include <drivers/st/stm32mp2_ram.h>
#include <drivers/st/stm32mp2_risaf.h>
#include <lib/fconf/fconf.h>
#include <lib/fconf/fconf_dyn_cfg_getter.h>
#include <lib/mmio.h>
#include <lib/optee_utils.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <platform_def.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp2_context.h>

#if CONFIG_STM32MP25X_REVA
#define BOOT_CTX_ADDR	0x0e000000UL /* beginning of SYSRAM */
#else
#define BOOT_CTX_ADDR	0x0e000020UL
#endif

IMPORT_SYM(uintptr_t, __BSS_START__, BSS_START);
IMPORT_SYM(uintptr_t, __BSS_END__, BSS_END);
IMPORT_SYM(uintptr_t, __DATA_START__, DATA_START);
IMPORT_SYM(uintptr_t, __DATA_END__, DATA_END);

static void print_reset_reason(void)
{
	uint32_t rstsr = mmio_read_32(stm32mp_rcc_base() + RCC_C1BOOTRSTSCLRR);

	if (rstsr == 0U) {
		WARN("Reset reason unknown\n");
		return;
	}

	INFO("Reset reason (0x%x):\n", rstsr);

	if ((rstsr & RCC_C1BOOTRSTSCLRR_PADRSTF) == 0U) {
		if ((rstsr & RCC_C1BOOTRSTSCLRR_D1STBYRSTF) != 0U) {
			INFO("D1 domain exits from DStandby\n");
			return;
		}

		if ((rstsr & RCC_C1BOOTRSTSCLRR_STBYC1RSTF) != 0U) {
			INFO("System exits from A35 STANDBY\n");
			return;
		}
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_PORRSTF) != 0U) {
		INFO("  Power-on Reset (rst_por)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_BORRSTF) != 0U) {
		INFO("  Brownout Reset (rst_bor)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSSETR_SYSC2RSTF) != 0U) {
		INFO("  System reset (SYSRST) by M33\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSSETR_SYSC1RSTF) != 0U) {
		INFO("  System reset (SYSRST) by A35\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_HCSSRSTF) != 0U) {
		INFO("  Clock failure on HSE\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_IWDG1SYSRSTF) != 0U) {
		INFO("  IWDG1 system reset (rst_iwdg1)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_IWDG2SYSRSTF) != 0U) {
		INFO("  IWDG2 system reset (rst_iwdg2)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_IWDG3SYSRSTF) != 0U) {
		INFO("  IWDG3 system reset (rst_iwdg3)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_IWDG4SYSRSTF) != 0U) {
		INFO("  IWDG4 system reset (rst_iwdg4)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_IWDG5SYSRSTF) != 0U) {
		INFO("  IWDG5 system reset (rst_iwdg5)\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_C1P1RSTF) != 0U) {
		INFO("  A35 processor core 1 reset\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_PADRSTF) != 0U) {
		INFO("  Pad Reset from NRST\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_VCORERSTF) != 0U) {
		INFO("  Reset due to a failure of VDD_CORE\n");
		return;
	}

	if ((rstsr & RCC_C1BOOTRSTSCLRR_C1RSTF) != 0U) {
		INFO("  A35 processor reset\n");
		return;
	}

	ERROR("  Unidentified reset reason\n");
}

void bl2_el3_early_platform_setup(u_register_t arg0 __unused,
				  u_register_t arg1 __unused,
				  u_register_t arg2 __unused,
				  u_register_t arg3 __unused)
{
	stm32mp_setup_early_console();

	stm32mp_save_boot_ctx_address(BOOT_CTX_ADDR);

	stm32_save_header();
}

void bl2_platform_setup(void)
{
	int ret;

	ret = stm32mp2_ddr_probe();
	if (ret != 0) {
		ERROR("DDR probe: error %d\n", ret);
		panic();
	}

	if (stm32mp2_risaf_init() < 0) {
		panic();
	}

	/* Map DDR for binary load, now with cacheable attribute */
	ret = mmap_add_dynamic_region(STM32MP_DDR_BASE, STM32MP_DDR_BASE,
				      STM32MP_DDR_MAX_SIZE, MT_MEMORY | MT_RW | MT_SECURE);
	if (ret < 0) {
		ERROR("DDR mapping: error %d\n", ret);
		panic();
	}

	/* Set QOS ICN priority */
	stm32mp2_syscfg_set_icn_qos();
}

static void reset_backup_domain(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();

	/*
	 * Disable the backup domain write protection.
	 * The protection is enable at each reset by hardware
	 * and must be disabled by software.
	 */
	mmio_setbits_32(pwr_base + PWR_BDCR1, PWR_BDCR1_DBD3P);

	while ((mmio_read_32(pwr_base + PWR_BDCR1) & PWR_BDCR1_DBD3P) == 0U) {
		;
	}

	/* Reset backup domain on cold boot cases */
	if ((mmio_read_32(rcc_base + RCC_BDCR) & RCC_BDCR_RTCSRC_MASK) == 0U) {
		mmio_setbits_32(rcc_base + RCC_BDCR, RCC_BDCR_VSWRST);

		while ((mmio_read_32(rcc_base + RCC_BDCR) & RCC_BDCR_VSWRST) == 0U) {
			;
		}

		mmio_clrbits_32(rcc_base + RCC_BDCR, RCC_BDCR_VSWRST);
	}
}

void bl2_el3_plat_arch_setup(void)
{
	const char *board_model;
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();

#if CONFIG_STM32MP25X_REVA
	/* Clear ROM code generated IACs for low power mode */
	mmio_write_32(IAC_BASE + IAC_ICR(IAC_RIFSC / 32), BIT(IAC_RIFSC % 32));
	mmio_write_32(IAC_BASE + IAC_ICR(IAC_BSEC / 32), BIT(IAC_BSEC % 32));
#endif

	if (stm32_otp_probe() != 0) {
		panic();
	}

	mmap_add_region(BL_CODE_BASE, BL_CODE_BASE,
			BL_CODE_END - BL_CODE_BASE,
			MT_CODE | MT_SECURE);

	configure_mmu();

	/* Prevent corruption of preloaded Device Tree */
	mmap_add_dynamic_region(DTB_BASE, DTB_BASE,
				DTB_LIMIT - DTB_BASE,
				MT_RO_DATA | MT_SECURE);

	if (dt_open_and_check(STM32MP_DTB_BASE) < 0) {
		panic();
	}

	reset_backup_domain();

	if (stm32mp2_clk_init() < 0) {
		panic();
	}

	if (stm32_iwdg_init() < 0) {
		panic();
	}

	stm32_iwdg_refresh();

	stm32_save_boot_info(boot_context);

	if (stm32mp_uart_console_setup() != 0) {
		goto skip_console_init;
	}

	stm32mp_print_cpuinfo();

	board_model = dt_get_board_model();
	if (board_model != NULL) {
		NOTICE("Model: %s\n", board_model);
	}

	print_reset_reason();

	if (boot_context->auth_status != BOOT_API_CTX_AUTH_NO) {
		NOTICE("Bootrom authentication %s\n",
		       (boot_context->auth_status == BOOT_API_CTX_AUTH_FAILED) ?
		       "failed" : "succeeded");
	}

skip_console_init:
#if !TRUSTED_BOARD_BOOT
	if (stm32mp_check_closed_device() == STM32MP_CHIP_SEC_CLOSED) {
		/* Closed chip mandates authentication */
		ERROR("Secure chip: TRUSTED_BOARD_BOOT must be enabled\n");
		panic();
	}
#endif

	if (stm32_rifsc_semaphore_init()) {
		panic();
	}

	if (stm32_rng_init() != 0) {
		panic();
	}

	if (fixed_regulator_register() != 0) {
		panic();
	}

	if (dt_pmic_status() > 0) {
		initialize_pmic();
	}

	stm32mp2_arch_security_setup();

	fconf_populate("TB_FW", STM32MP_DTB_BASE);

#if STM32MP_DDR_FIP_IO_STORAGE || TRUSTED_BOARD_BOOT
	/*
	 * RISAB3 setup (dedicated for SRAM1)
	 *
	 * Allow secure read/writes data accesses to non-secure
	 * blocks or pages, all RISAB registers are writable.
	 * DDR firmwares are saved there before being loaded in DDRPHY memory.
	 */
	mmio_write_32(RISAB3_BASE + RISAB_CR, RISAB_CR_SRWIAD);
#endif /* STM32MP_DDR_FIP_IO_STORAGE || TRUSTED_BOARD_BOOT */
#if STM32MP_USB_PROGRAMMER
	/*
	 * Set USB3DR Peripheriphal accesses to Secure/Privilege only
	 */
	mmio_write_32(RIFSC_BASE + _RIFSC_RISC_SECCFGR(STM32MP25_RIFSC_USB3DR_ID),
		     RIFSC_USB3DR_SEC);
	mmio_write_32(RIFSC_BASE + _RIFSC_RISC_PRIVCFGR(STM32MP25_RIFSC_USB3DR_ID),
		      RIFSC_USB3DR_PRIV);

	/*
	 * Apply USB boot specific configuration to RIF master USB3DR
	 */
	mmio_write_32(RIFSC_BASE + _RIFSC_RIMC_ATTR(STM32MP25_RIMU_USB3DR),
		      RIFSC_USB_BOOT_USBDR_RIMC_CONF);
#endif /* STM32MP_USB_PROGRAMMER */

	/*
	 * RISAB5 setup (dedicated for RETRAM)
	 *
	 * Allow secure read/writes data accesses to non-secure
	 * blocks or pages, all RISAB registers are writable.
	 * DDR retention registers are saved there and restored
	 * when exiting standby low power state.
	 */
	mmio_write_32(RISAB5_BASE + RISAB_CR, RISAB_CR_SRWIAD);

	if (stm32mp2_pwr_init_io_domains() != 0) {
		panic();
	}

#if STM32MP_DDR_FIP_IO_STORAGE
	/* Skip DDR FW ID = the first image to load for standby exit */
	if (stm32mp_is_wakeup_from_standby()) {
		bl_mem_params_node_t *bl_mem_params;

		bl_mem_params = get_bl_mem_params_node(DDR_FW_ID);
		assert(bl_mem_params != NULL);
		bl_mem_params->image_info.h.attr |= IMAGE_ATTRIB_SKIP_LOADING;
	}
#endif

	stm32mp_io_setup();
}

static void prepare_encryption(void)
{
	uint8_t mkey[RISAF_KEY_SIZE_IN_BYTES];

	if (stm32mp_is_wakeup_from_standby()) {
		stm32mp_pm_get_enc_mkey_from_context(mkey);
	} else {
		/* Generate RISAF master key from RNG */
		if (stm32_rng_read(mkey, RISAF_KEY_SIZE_IN_BYTES) != 0) {
			panic();
		}

		stm32mp_pm_save_enc_mkey_in_context(mkey);
	}

	if (stm32mp2_risaf_write_master_key(RISAF4_INST, mkey) != 0) {
		panic();
	}
}

/*******************************************************************************
 * This function can be used by the platforms to update/use image
 * information for given `image_id`.
 ******************************************************************************/
int bl2_plat_handle_post_image_load(unsigned int image_id)
{
	int err = 0;
	bl_mem_params_node_t *bl_mem_params = get_bl_mem_params_node(image_id);
	bl_mem_params_node_t *bl32_mem_params __unused;
	bl_mem_params_node_t *pager_mem_params __unused;
	const struct dyn_cfg_dtb_info_t *config_info;
	unsigned int i;
	unsigned long long ddr_top __unused;
	const unsigned int image_ids[] = {
		BL31_IMAGE_ID,
		SOC_FW_CONFIG_ID,
		BL32_IMAGE_ID,
		BL33_IMAGE_ID,
		HW_CONFIG_ID,
	};
	uint32_t otp_idx __maybe_unused;
	uint32_t otp_len __maybe_unused;
	bool wakeup_standby;

	assert(bl_mem_params != NULL);

	switch (image_id) {
	case FW_CONFIG_ID:
		if ((stm32mp_check_closed_device() == STM32MP_CHIP_SEC_CLOSED) ||
		    stm32mp_is_auth_supported()) {
			prepare_encryption();
		}

		/* Set global DTB info for fixed fw_config information */
		set_config_info(STM32MP_FW_CONFIG_BASE, ~0UL, STM32MP_FW_CONFIG_MAX_SIZE,
				FW_CONFIG_ID);
		fconf_populate("FW_CONFIG", STM32MP_FW_CONFIG_BASE);

		wakeup_standby = stm32mp_is_wakeup_from_standby();

		/* Iterate through all the fw config IDs */
		for (i = 0U; i < ARRAY_SIZE(image_ids); i++) {
			bl_mem_params = get_bl_mem_params_node(image_ids[i]);
			assert(bl_mem_params != NULL);

			config_info = FCONF_GET_PROPERTY(dyn_cfg, dtb, image_ids[i]);
			if (config_info == NULL) {
				continue;
			}

			bl_mem_params->image_info.image_base = config_info->config_addr;
			bl_mem_params->image_info.image_max_size = config_info->config_max_size;

			/*
			 * If not going back from STANDBY (preserve the DDR in Self-Refresh)
			 * or partition not located in DDR
			 * the partitions must be loaded = remove the SKIP flag
			 */
			if (!wakeup_standby || (config_info->config_addr < STM32MP_DDR_BASE)) {
				bl_mem_params->image_info.h.attr &= ~IMAGE_ATTRIB_SKIP_LOADING;
			}

			switch (image_ids[i]) {
			case BL31_IMAGE_ID:
				bl_mem_params->ep_info.pc = config_info->config_addr;
				break;
			case BL32_IMAGE_ID:
				bl_mem_params->ep_info.pc = config_info->config_addr;

				/* In case of OPTEE, initialize address space with tos_fw addr */
				pager_mem_params = get_bl_mem_params_node(BL32_EXTRA1_IMAGE_ID);
				if (pager_mem_params != NULL) {
					pager_mem_params->image_info.image_base =
						config_info->config_addr;
					pager_mem_params->image_info.image_max_size =
						config_info->config_max_size;
				}
				break;

			case BL33_IMAGE_ID:
				bl_mem_params->ep_info.pc = config_info->config_addr;
				break;

			case HW_CONFIG_ID:
			case SOC_FW_CONFIG_ID:
				break;

			default:
				return -EINVAL;
			}
		}

#ifndef DECRYPTION_SUPPORT_none
		/* Load encryption key info before DT is unmapped */
		err = stm32_get_enc_key_otp_idx_len(&otp_idx, &otp_len);
		if (err) {
			panic();
		}
#endif
		mmap_remove_dynamic_region(DTB_BASE, DTB_LIMIT - DTB_BASE);
		inv_dcache_range(DTB_BASE, DTB_LIMIT - DTB_BASE);

		break;

	case BL32_IMAGE_ID:
		if (optee_header_is_valid(bl_mem_params->image_info.image_base)) {
			/* BL32 is OP-TEE header */
			bl_mem_params->ep_info.pc = bl_mem_params->image_info.image_base;
			pager_mem_params = get_bl_mem_params_node(BL32_EXTRA1_IMAGE_ID);
			assert(pager_mem_params != NULL);

			err = parse_optee_header(&bl_mem_params->ep_info,
						 &pager_mem_params->image_info,
						 NULL);
			if (err) {
				ERROR("OPTEE header parse error.\n");
				panic();
			}

			/* Set optee boot info from parsed header data */
			bl_mem_params->ep_info.args.arg0 = 0U; /* Unused */
			bl_mem_params->ep_info.args.arg1 = 0U; /* Unused */
			bl_mem_params->ep_info.args.arg2 = 0U; /* No DT supported */
		}
		break;

	case BL33_IMAGE_ID:
#if PSA_FWU_SUPPORT
		stm32_fwu_set_boot_idx();
#endif /* PSA_FWU_SUPPORT */
		break;

	default:
		/* Do nothing in default case */
		break;
	}

#if STM32MP_SDMMC || STM32MP_EMMC
	/*
	 * Invalidate remaining data read from MMC but not flushed by load_image_flush().
	 * We take the worst case which is 2 MMC blocks.
	 */
	if ((image_id != FW_CONFIG_ID) &&
	    ((bl_mem_params->image_info.h.attr & IMAGE_ATTRIB_SKIP_LOADING) == 0U)) {
		inv_dcache_range(bl_mem_params->image_info.image_base +
				 bl_mem_params->image_info.image_size,
				 2U * MMC_BLOCK_SIZE);
	}
#endif /* STM32MP_SDMMC || STM32MP_EMMC */

	return err;
}

void bl2_el3_plat_prepare_exit(void)
{
	flush_dcache_range(BSS_START, BSS_END - BSS_START);
	flush_dcache_range(DATA_START, DATA_END - DATA_START);

	stm32mp_io_exit();
	stm32mp2_security_setup();
}
