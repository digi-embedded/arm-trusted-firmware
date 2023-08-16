/*
 * Copyright (c) 2015-2023, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_PRIVATE_H
#define STM32MP1_PRIVATE_H

#include <stdint.h>

#include <drivers/st/etzpc.h>

void configure_mmu(void);

void stm32mp_mask_timer(void);
void __dead2 stm32mp_wait_cpu_reset(void);

void stm32mp1_arch_security_setup(void);
void stm32mp1_security_setup(void);

bool stm32mp1_addr_inside_backupsram(uintptr_t addr);
bool stm32mp1_is_wakeup_from_standby(void);

#if defined(IMAGE_BL32)
enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode);
#endif

void stm32mp1_syscfg_init(void);
void stm32mp1_syscfg_enable_io_compensation_start(void);
void stm32mp1_syscfg_enable_io_compensation_finish(void);
void stm32mp1_syscfg_disable_io_compensation(void);
uint32_t stm32mp1_syscfg_get_chip_version(void);
uint32_t stm32mp1_syscfg_get_chip_dev_id(void);
#if STM32MP13
void stm32mp1_syscfg_boot_mode_enable(void);
void stm32mp1_syscfg_boot_mode_disable(void);
#endif
#if STM32MP15
static inline void stm32mp1_syscfg_boot_mode_enable(void){}
static inline void stm32mp1_syscfg_boot_mode_disable(void){}
#endif

void stm32mp1_deconfigure_uart_pins(void);

void stm32mp1_init_scmi_server(void);
void stm32mp1_pm_save_scmi_state(uint8_t *state, size_t size);
void stm32mp1_pm_restore_scmi_state(uint8_t *state, size_t size);

bool stm32mp_bkpram_get_access(void);

#if defined(IMAGE_BL32) && DEBUG
void stm32mp_dump_core_registers(bool fcore);
#endif
#endif /* STM32MP1_PRIVATE_H */
