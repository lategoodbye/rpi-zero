/* CPU feature definitions for module loading, used by
 * module_cpu_feature_match(), see asm/cputable.h for powerpc CPU features
 *
 * Copyright 2016 Alastair D'Silva, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef __ASM_CPUFEATURE_H
#define __ASM_POWERPC_CPUFEATURE_H

#include <asm/cputable.h>

/* Keep these in step with powerpc/include/asm/cputable.h */
#define MAX_CPU_FEATURES (2 * 32)

#define PPC_MODULE_FEATURE_32				(ilog2(PPC_FEATURE_32))
#define PPC_MODULE_FEATURE_64				(ilog2(PPC_FEATURE_64))
#define PPC_MODULE_FEATURE_601_INSTR			(ilog2(PPC_FEATURE_601_INSTR))
#define PPC_MODULE_FEATURE_HAS_ALTIVEC			(ilog2(PPC_FEATURE_HAS_ALTIVEC))
#define PPC_MODULE_FEATURE_HAS_FPU			(ilog2(PPC_FEATURE_HAS_FPU))
#define PPC_MODULE_FEATURE_HAS_MMU			(ilog2(PPC_FEATURE_HAS_MMU))
#define PPC_MODULE_FEATURE_HAS_4xxMAC			(ilog2(PPC_FEATURE_HAS_4xxMAC))
#define PPC_MODULE_FEATURE_UNIFIED_CACHE		ilog2(PPC_FEATURE_UNIFIED_CACHE))
#define PPC_MODULE_FEATURE_HAS_SPE			(ilog2(PPC_FEATURE_HAS_SPE))
#define PPC_MODULE_FEATURE_HAS_EFP_SINGLE		(ilog2(PPC_FEATURE_HAS_EFP_SINGLE))
#define PPC_MODULE_FEATURE_HAS_EFP_DOUBLE		(ilog2(PPC_FEATURE_HAS_EFP_DOUBLE))
#define PPC_MODULE_FEATURE_NO_TB			(ilog2(PPC_FEATURE_NO_TB))
#define PPC_MODULE_FEATURE_POWER4			(ilog2(PPC_FEATURE_POWER4))
#define PPC_MODULE_FEATURE_POWER5			(ilog2(PPC_FEATURE_POWER5))
#define PPC_MODULE_FEATURE_POWER5_PLUS			(ilog2(PPC_FEATURE_POWER5_PLUS))
#define PPC_MODULE_FEATURE_CELL				(ilog2(PPC_FEATURE_CELL))
#define PPC_MODULE_FEATURE_BOOKE			(ilog2(PPC_FEATURE_BOOKE))
#define PPC_MODULE_FEATURE_SMT				(ilog2(PPC_FEATURE_SMT))
#define PPC_MODULE_FEATURE_ICACHE_SNOOP			(ilog2(PPC_FEATURE_ICACHE_SNOOP))
#define PPC_MODULE_FEATURE_ARCH_2_05			(ilog2(PPC_FEATURE_ARCH_2_05))
#define PPC_MODULE_FEATURE_PA6T				(ilog2(PPC_FEATURE_PA6T))
#define PPC_MODULE_FEATURE_HAS_DFP			(ilog2(PPC_FEATURE_HAS_DFP))
#define PPC_MODULE_FEATURE_POWER6_EXT			(ilog2(PPC_FEATURE_POWER6_EXT))
#define PPC_MODULE_FEATURE_ARCH_2_06			(ilog2(PPC_FEATURE_ARCH_2_06))
#define PPC_MODULE_FEATURE_HAS_VSX			(ilog2(PPC_FEATURE_HAS_VSX))
#define PPC_MODULE_FEATURE_PSERIES_PERFMON_COMPAT	(ilog2(PPC_FEATURE_PSERIES_PERFMON_COMPAT))
#define PPC_MODULE_FEATURE_TRUE_LE			(ilog2(PPC_FEATURE_TRUE_LE))
#define PPC_MODULE_FEATURE_PPC_LE			(ilog2(PPC_FEATURE_PPC_LE))

#define PPC_MODULE_FEATURE_ARCH_2_07			(32 + ilog2(PPC_FEATURE2_ARCH_2_07))
#define PPC_MODULE_FEATURE_HTM				(32 + ilog2(PPC_FEATURE2_HTM))
#define PPC_MODULE_FEATURE_DSCR				(32 + ilog2(PPC_FEATURE2_DSCR))
#define PPC_MODULE_FEATURE_EBB				(32 + ilog2(PPC_FEATURE2_EBB))
#define PPC_MODULE_FEATURE_ISEL				(32 + ilog2(PPC_FEATURE2_ISEL))
#define PPC_MODULE_FEATURE_TAR				(32 + ilog2(PPC_FEATURE2_TAR))
#define PPC_MODULE_FEATURE_VEC_CRYPTO			(32 + ilog2(PPC_FEATURE2_VEC_CRYPTO))
#define PPC_MODULE_FEATURE_HTM_NOSC			(32 + ilog2(PPC_FEATURE2_HTM_NOSC))
#define PPC_MODULE_FEATURE_ARCH_3_00			(32 + ilog2(PPC_FEATURE2_ARCH_3_00))
#define PPC_MODULE_FEATURE_HAS_IEEE128			(32 + ilog2(PPC_FEATURE2_HAS_IEEE128))

#define cpu_feature(x)		(x)

static inline bool cpu_have_feature(unsigned int num)
{
	if (num < 32)
		return !!(cur_cpu_spec->cpu_user_features & 1UL << num);
	else
		return !!(cur_cpu_spec->cpu_user_features2 & 1UL << (num - 32));
}

#endif /* __ASM_POWERPC_CPUFEATURE_H */
