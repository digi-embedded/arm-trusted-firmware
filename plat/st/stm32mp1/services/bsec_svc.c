/*
 * Copyright (c) 2016-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <drivers/regulator.h>
#include <drivers/st/bsec.h>
#include <drivers/st/bsec2_reg.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stpmic1.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <services/std_svc.h>

#include <boot_api.h>
#include <stm32mp1_dbgmcu.h>
#include <stm32mp1_smc.h>

#include "bsec_svc.h"

/*
 * version of STM32_SMC_READ_ALL / STM32_SMC_WRITE_ALL service
 * This must be increased at each structure otp_exchange modification
 */
#define BSEC_SERVICE_VERSION		0x02U

/* Global status bitfield */
#define BSEC_STATE_SEC_OPEN	U(0x0)
#define BSEC_STATE_SEC_CLOSED	U(0x1)
#define BSEC_STATE_INVALID	U(0x2)

/* Bitfield definition status */
#define OTP_UPDATE_REQ			BIT(31)
#define OTP_ERROR_DETECTED		BIT(0)

/* Bitfield definition for LOCK status */
#define LOCK_PERM			BIT(30)
#define LOCK_SHADOW_R			BIT(29)
#define LOCK_SHADOW_W			BIT(28)
#define LOCK_SHADOW_P			BIT(27)
#define LOCK_ERROR			BIT(26)

struct otp_state {
	uint32_t value;
	uint32_t state;
};

struct otp_exchange {
	uint32_t version;
	uint32_t status;
	struct otp_state otp[STM32MP1_OTP_MAX_ID];
};

static uint32_t bsec_read_all_bsec(struct otp_exchange *exchange)
{
	uint32_t i;
	uint32_t result;
	uint32_t shadow_prog_lock[3];
	uint32_t shadow_read_lock[3];
	uint32_t shadow_write_lock[3];
	uint32_t permanent_lock[3];
	uint32_t bsec_base;
	uint32_t status;

	if (exchange == NULL) {
		return BSEC_ERROR;
	}

	exchange->version = BSEC_SERVICE_VERSION;

	status = bsec_get_status();
	if ((status & BSEC_MODE_INVALID_MASK) != 0U) {
		exchange->status = BSEC_STATE_INVALID;
	} else {
		if ((status & BSEC_MODE_SECURE_MASK) != 0U) {
			if (stm32mp_is_closed_device()) {
				exchange->status = BSEC_STATE_SEC_CLOSED;
			} else {
				exchange->status = BSEC_STATE_SEC_OPEN;
			}
		} else {
			/* OTP modes OPEN1 and OPEN2 are not supported */
			exchange->status = BSEC_STATE_INVALID;
		}
	}

	bsec_base = bsec_get_base();
	for (i = 0U; i < 3U; i++) {
		permanent_lock[i] = mmio_read_32(bsec_base + BSEC_WRLOCK(i));
		shadow_prog_lock[i] = mmio_read_32(bsec_base + BSEC_SPLOCK(i));
		shadow_read_lock[i] = mmio_read_32(bsec_base + BSEC_SRLOCK(i));
		shadow_write_lock[i] = mmio_read_32(bsec_base + BSEC_SWLOCK(i));
	}

	for (i = 0U; i <= STM32MP1_OTP_MAX_ID; i++) {
		uint32_t offset = i / __WORD_BIT;
		uint32_t bits = BIT(i % __WORD_BIT);

		exchange->otp[i].value = 0U;
		exchange->otp[i].state = 0U;

		result = bsec_shadow_register(i);
		if (result == BSEC_OK) {
			result = bsec_read_otp(&exchange->otp[i].value, i);
		}
		if (result != BSEC_OK) {
			exchange->otp[i].value = 0;
			exchange->otp[i].state |= OTP_ERROR_DETECTED;
		}
		if (permanent_lock[offset] & bits) {
			exchange->otp[i].state |= LOCK_PERM;
		}
		if (shadow_read_lock[offset] & bits) {
			exchange->otp[i].state |= LOCK_SHADOW_R;
		}
		if (shadow_write_lock[offset] & bits) {
			exchange->otp[i].state |= LOCK_SHADOW_W;
		}
		if (shadow_prog_lock[offset] & bits) {
			exchange->otp[i].state |= LOCK_SHADOW_P;
		}
	}

	return BSEC_OK;
}

static uint32_t bsec_write_all_bsec(struct otp_exchange *exchange,
				    uint32_t *ret_otp_value)
{
	uint32_t i;
	uint32_t ret;

	*ret_otp_value = 0U;

	if (exchange == NULL) {
		return BSEC_ERROR;
	}

	if (exchange->version != BSEC_SERVICE_VERSION) {
		return BSEC_ERROR;
	}

	for (i = 0U; i <= STM32MP1_OTP_MAX_ID; i++) {
		if ((exchange->otp[i].state & OTP_UPDATE_REQ) == 0U) {
			continue;
		}
		if (exchange->otp[i].value != 0U) {
			ret = bsec_program_otp(exchange->otp[i].value, i);
			if (ret == BSEC_OK) {
				ret = bsec_write_otp(exchange->otp[i].value, i);
			}
			if (ret != BSEC_OK) {
				ERROR("BSEC write failed on OTP%u\n", i);
				return ret;
			}
		}
		if ((exchange->otp[i].state & LOCK_PERM) != 0U) {
			ret = bsec_permanent_lock_otp(i);
			if (ret != BSEC_OK) {
				ERROR("BSEC permanent lock failed on OTP%u\n", i);
				return ret;
			}
		}
		if ((exchange->otp[i].state & LOCK_SHADOW_R) != 0U) {
			ret = bsec_set_sr_lock(i);
			if (ret != BSEC_OK) {
				ERROR("BSEC sr lock failed on OTP%u\n", i);
				return ret;
			}
		}
		if ((exchange->otp[i].state & LOCK_SHADOW_W) != 0U) {
			ret = bsec_set_sw_lock(i);
			if (ret != BSEC_OK) {
				ERROR("BSEC sw lock failed on OTP%u\n", i);
				return ret;
			}
		}
		if ((exchange->otp[i].state & LOCK_SHADOW_P) != 0U) {
			ret = bsec_set_sp_lock(i);
			if (ret != BSEC_OK) {
				ERROR("BSEC sp lock failed on OTP%u\n", i);
				return ret;
			}
		}
	}

	INFO("write all otp succeed\n");

	return BSEC_OK;
}

uint32_t bsec_main(uint32_t x1, uint32_t x2, uint32_t x3,
		   uint32_t *ret_otp_value)
{
	uint32_t result;
	uint32_t tmp_data = 0U;
	struct otp_exchange *otp_exch __unused;
	uintptr_t map_begin __unused;
	size_t map_size __unused = PAGE_SIZE;
	int ret __unused;

	if ((x1 != STM32_SMC_READ_ALL) && (x1 != STM32_SMC_WRITE_ALL) &&
	    (bsec_check_nsec_access_rights(x2) != BSEC_OK)) {
		return STM32_SMC_INVALID_PARAMS;
	}

	otp_exch = NULL;
	map_begin = 0U;

	if ((x1 == STM32_SMC_READ_ALL) || (x1 == STM32_SMC_WRITE_ALL)) {
		if (!stm32_boot_is_serial()) {
			return STM32_SMC_FAILED;
		}

		map_begin = round_down(x2, PAGE_SIZE);

		if (round_down(x2 + sizeof(struct otp_exchange), PAGE_SIZE) !=
		    map_begin) {
			/*
			 * Buffer end is in the next page, 2 pages need to be
			 * mapped.
			 */
			map_size += PAGE_SIZE;
		}

		ret = mmap_add_dynamic_region(map_begin,
					      map_begin,
					      map_size,
					      MT_MEMORY | MT_RW | MT_NS);
		assert(ret == 0);

		if (!ddr_is_nonsecured_area(map_begin, map_size)) {
			ret = mmap_remove_dynamic_region(map_begin, map_size);
			assert(ret == 0);

			return STM32_SMC_INVALID_PARAMS;
		}

		otp_exch = (struct otp_exchange *)(uintptr_t)x2;
	}

	switch (x1) {
	case STM32_SMC_READ_SHADOW:
		result = bsec_read_otp(ret_otp_value, x2);
		break;
	case STM32_SMC_PROG_OTP:
		*ret_otp_value = 0U;
		result = bsec_program_otp(x3, x2);
		break;
	case STM32_SMC_WRITE_SHADOW:
		*ret_otp_value = 0U;
		result = bsec_write_otp(x3, x2);
		break;
	case STM32_SMC_READ_OTP:
		*ret_otp_value = 0U;
		result = bsec_read_otp(&tmp_data, x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_shadow_register(x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_read_otp(ret_otp_value, x2);
		if (result != BSEC_OK) {
			break;
		}

		result = bsec_write_otp(tmp_data, x2);
		break;
	case STM32_SMC_READ_ALL:
		result = bsec_read_all_bsec(otp_exch);
		break;
	case STM32_SMC_WRITE_ALL:
		result = bsec_write_all_bsec(otp_exch, ret_otp_value);
		break;
	case STM32_SMC_WRLOCK_OTP:
		result = bsec_permanent_lock_otp(x2);
		break;
	default:
		return STM32_SMC_INVALID_PARAMS;
	}

	if ((x1 == STM32_SMC_READ_ALL) || (x1 == STM32_SMC_WRITE_ALL)) {
		ret = mmap_remove_dynamic_region(map_begin, map_size);
		assert(ret == 0);
	}

	return (result == BSEC_OK) ? STM32_SMC_OK : STM32_SMC_FAILED;
}
