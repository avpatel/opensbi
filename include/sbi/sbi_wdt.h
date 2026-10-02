/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Prajwal N <prajwal.n@oss.qualcomm.com>
 */

#ifndef __SBI_WDT_H__
#define __SBI_WDT_H__

/**
 * Read the watchdog attribute
 * @param attribute_id The wdt attribute id to be read
 * @param attribute_value The value of attribute returned
 * @return int error
 */
int sbi_watchdog_read_attribute(u32 attribute_id, u32* attribute_value);

/**
 * Write the watchdog attribute
 * @param attribute_id The wdt attribute id to be written
 * @param attribute_value The wdt attribute value to be written
 * @return int error
 */
int sbi_watchdog_write_attribute(u32 attribute_id, u32 attribute_value);

/**
 * Write the watchdog MSI message attribute
 * @param msi_addr_low The value of MSI low address
 * @param msi_addr_high The value of MSI high address
 * @param msi_data The value of MSI data
 * @return int error
 */
int sbi_watchdog_write_notif_msi_message(u32 msi_addr_low, u32 msi_addr_high, u32 msi_data);

/**
 * Start the watchdog
 * @param void
 * @return int error
 */
int sbi_watchdog_start(void);

/**
 * Stop the watchdog
 * @param void
 * @return int error
 */
int sbi_watchdog_stop(void);

/**
 * Pat the watchdog
 * @param void
 * @return int error
 */
int sbi_watchdog_pat(void);

/**
 * Initialize the watchdog
 * @param void
 * @return int error
 */
int sbi_watchdog_init(void);

/**
 * Remove the watchdog
 * @param void
 * @return int error
 */
int sbi_watchdog_exit(void);
#endif
