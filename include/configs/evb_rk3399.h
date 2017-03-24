/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __EVB_RK3399_H
#define __EVB_RK3399_H

/*
 * SPL @ 32kB for ~130kB
 * ENV @ 240KB for 8kB
 * FIT payload (ATF, U-Boot, FDT) @ 256kB
 */
#define CONFIG_ENV_OFFSET (240 * 1024)

#include <configs/rk3399_common.h>

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV 1

#define SDRAM_BANK_SIZE			(2UL << 30)

#define CONFIG_SYS_WHITE_ON_BLACK

#endif
