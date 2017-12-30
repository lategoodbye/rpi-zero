/*
 * Copyright (C) 2017 Stefan Wahren <stefan.wahren@i2se.com>
 * Copyright (C) 2017 Tadeusz Kijkowski
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 */

#define BCM2836_REPARK_PHYS_BASE_OFFSET		0
#define BCM2836_REPARK_VIRT_BASE_OFFSET 	4
#define BCM2836_REPARK_CPU_STATUS_OFFSET	8

#define BCM2836_MAX_CPUS	4

#ifndef __ASSEMBLY__
asmlinkage void bcm2836_repark_loop(void);

struct bcm2836_arm_cpu_repark_data {
	u32 mailbox_rdclr_phys_base;
	void __iomem *mailbox_rdclr_virt_base;
	volatile u32 cpu_status[BCM2836_MAX_CPUS];
};

extern const struct smp_operations bcm2836_smp_ops;

#endif /* __ASSEMBLY__ */
