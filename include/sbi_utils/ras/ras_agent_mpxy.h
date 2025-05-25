/*
 * RISC-V RERI Registers Definitions
 *
 * Copyright (c) 2024 Ventana Micro Systems, Inc.
 *
 * Author(s):
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef __RAS_AGENT_MPXY_H
#define __RAS_AGENT_MPXY_H

/* RAS Agent Services on MPXY/RPMI */
#define RAS_ENABLE_NOTIFICATION		0x1
#define RAS_GET_NUM_ERR_SRCS		0x2
#define RAS_GET_ERR_SRCS_ID_LIST	0x3
#define RAS_GET_ERR_SRC_DESC		0x4

/* Used to generate EINJ table */
#define RAS_GET_EINJ_INFO		0x5
#define RAS_GET_EINJ_INST		0x6

/* Used during EINJ FFH read/write operations */
#define RAS_EINJ_READ_REG		0x7
#define RAS_EINJ_WRITE_REG		0x8

int ras_mpxy_init(const void *fdt, int nodeoff);

#endif
